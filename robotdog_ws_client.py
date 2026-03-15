#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robotdog_ws_client.py

机器狗本地 WebSocket 客户端：
  - 在独立线程中连接 Android App 本地 WebSocket 服务
  - 建立连接后打印连接成功日志
  - 收到 RobotDogTaskTurning 消息时：
      * 把 data 里的经纬度航点写入 JSON 文件
      * 激活 conda 环境并启动 zed_rtk_move.py
  - 处理服务端 ping 消息并回复 pong
  - 支持断线自动重连（指数退避）
"""

import json
import threading
import time
import subprocess
import os
import websocket


CONFIG_PATH = "config.json"


def _load_android_config():
    """从 config.json 读取 Android 端 host/port，WebSocket 用 ws_port(8888)，HTTP API 用 api_port(8080)"""
    config_path = CONFIG_PATH
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        host = cfg.get("android_host", "10.65.42.98")
        port = int(cfg.get("ws_port", 8888))
        return host, port
    except Exception:
        return "10.65.42.98", 8888


_ANDROID_HOST, _WS_PORT = _load_android_config()

# ================= 连接配置 =================
WS_URL = f"ws://{_ANDROID_HOST}:{_WS_PORT}"
RECONNECT_MIN_SECONDS = 3
RECONNECT_MAX_SECONDS = 10
OPEN_TIMEOUT_SECONDS = 10


# ================= ZED 导航脚本配置 =================
# 注意：以下路径和命令需要根据你在 Linux 上的实际环境修改
CONDA_ACTIVATE_CMD = "source ~/anaconda3/bin/activate && conda activate unitree"
ZED_SCRIPT_PATH = "rtk_cors_wifi.py"  # 修改成 zed_rtk_move.py 在机器人上的实际绝对路径或 ~ 路径
ZED_NETWORK_INTERFACE = "eno1"
WAYPOINTS_FILE = "tmp/robotdog_turning_waypoints.json"

# ================= 导航控制 =================
# 通过这些文件与导航进程通信，实现暂停/继续/返航等控制
PAUSE_STATE_FILE = "tmp/robotdog_nav_pause_state.json"
NAV_STATE_FILE = "tmp/robotdog_nav_state.json"


# ================= 全局状态 =================
stop_event = threading.Event()
connected_event = threading.Event()
_zed_process = None
_zed_process_lock = threading.Lock()


# ================= 工具函数 =================
def build_pong_message(timestamp):
    """
    生成服务端 ping 对应的 pong 消息。
    """
    return json.dumps(
        {
            "type": "pong",
            "timestamp": timestamp if timestamp is not None else int(time.time() * 1000),
        },
        ensure_ascii=False,
    )


def _launch_zed_navigation_process():
    """
    启动 ZED + RTK 导航脚本。

    会：
      1. 在 bash 中执行 conda 激活命令
      2. 运行 zed_rtk_move.py，并把网卡和航点文件路径作为参数传入：
         python zed_rtk_move.py eno1 /tmp/robotdog_turning_waypoints.json
    """
    global _zed_process

    with _zed_process_lock:
        if _zed_process is not None and _zed_process.poll() is None:
            print("[ZED] 导航程序已在运行，跳过本次启动")
            return

        cmd = (
            f"{CONDA_ACTIVATE_CMD} && "
            f"python {ZED_SCRIPT_PATH} {ZED_NETWORK_INTERFACE} {WAYPOINTS_FILE}"
        )

        print(f"[ZED] 即将启动导航进程: {cmd}")
        try:
            # 使用 bash -lc 以便支持 source / conda 等 shell 命令
            _zed_process = subprocess.Popen(
                ["bash", "-lc", cmd],
                stdout=None,
                stderr=None,
            )
        except Exception as exc:
            print(f"[ZED] 启动导航进程失败: {exc}")


def _handle_robotdog_task_turning(payload):
    """
    处理前端下发的 RobotDogTaskTurning 指令：
      - payload 形如：
        {
          "type": "RobotDogTaskTurning",
          "ackId": "...",
          "logId": "...",
          "data": [[lon, lat], [lon, lat], ...]
        }
      - 将 data 写入 JSON 文件，供 zed_rtk_move.py 读取
      - 然后启动 zed_rtk_move.py
    """
    data_raw = payload.get("data")
    target = payload.get("target")

    # data 既可能是数组，也可能是字符串形式的 JSON（"[[lon,lat], ...]"）
    if isinstance(data_raw, str):
        try:
            data = json.loads(data_raw)
        except Exception as exc:
            print(f"[WS] RobotDogTaskTurning data 解析失败: {exc} | raw={data_raw!r}")
            return
    else:
        data = data_raw

    if not isinstance(data, list) or not data:
        print("[WS] RobotDogTaskTurning 数据为空或格式错误，已忽略")
        return

    # 确保目录存在
    waypoints_dir = os.path.dirname(WAYPOINTS_FILE)
    if waypoints_dir and not os.path.exists(waypoints_dir):
        try:
            os.makedirs(waypoints_dir, exist_ok=True)
        except Exception as exc:
            print(f"[WS] 创建航点目录失败: {exc}")
            return

    try:
        with open(WAYPOINTS_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False)
        print(f"[WS] 已写入 {len(data)} 个航点到 {WAYPOINTS_FILE}")
    except Exception as exc:
        print(f"[WS] 写入航点文件失败: {exc}")
        return

    # 通知 rtk_cors.py 重新加载航点并进入巡航
    start_state = {
        "command": "startTask",
        "timestamp": time.time(),
        "waypoint_file": WAYPOINTS_FILE,
        "target": target,
    }
    try:
        with open(NAV_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(start_state, f, ensure_ascii=False)
        print(f"[WS] startTask 已写入启动指令到 {NAV_STATE_FILE}")
    except Exception as exc:
        print(f"[WS] startTask 写入失败: {exc}")

    _launch_zed_navigation_process()


def _handle_stop_task(payload):
    """
    处理前端下发的 stopTask 指令：
      - payload 形如：
        {
          "type": "stopTask",
          "ackId": "...",
          "data": {
              "continueType": true/false,
              "sn": "1581F8HXX233S00A05PG"
          }
        }
      - continueType == False: 暂停自动导航（狗停下，但后台线程继续）
      - continueType == True : 继续自动导航

    具体做法：将暂停状态写入 PAUSE_STATE_FILE，rtk.py 周期性读取该文件。
    """
    data = payload.get("data") or {}
    cont = data.get("continueType")

    if not isinstance(cont, bool):
        print("[WS] stopTask.continueType 缺失或类型错误，已忽略")
        return

    paused = not cont  # continueType=False -> paused=True

    state = {
        "paused": paused,
        "timestamp": time.time(),
        "sn": data.get("sn"),
    }

    try:
        with open(PAUSE_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(state, f, ensure_ascii=False)
        print(f"[WS] stopTask 已更新暂停状态: paused={paused}, 写入 {PAUSE_STATE_FILE}")
    except Exception as exc:
        print(f"[WS] stopTask 写入暂停状态失败: {exc}")


def _handle_back_task(payload):
    """
    处理前端下发的 backTask 指令：
      - payload 形如：
        {
          "type": "backTask",
          "ackId": "...",
          "data": {
              "navigationId": "...",
              "pathPoints": "...",
              "sn": "1581F8HXX233S00A05PG"
          }
        }
      - 将一个简单的返航命令写入 NAV_STATE_FILE，供 rtk_cors.py 读取
    """
    data = payload.get("data") or {}

    state = {
        "command": "backTask",
        "timestamp": time.time(),
        "navigationId": data.get("navigationId"),
        "pathPoints": data.get("pathPoints"),
        "sn": data.get("sn"),
    }

    try:
        with open(NAV_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(state, f, ensure_ascii=False)
        print(f"[WS] backTask 已写入返航指令到 {NAV_STATE_FILE}")
    except Exception as exc:
        print(f"[WS] backTask 写入返航指令失败: {exc}")


def _handle_finish_task(payload):
    """
    处理前端下发的 finishTask 指令：只要 type 为 finishTask 即结束任务。
    将命令写入 NAV_STATE_FILE，供 rtk_cors.py 读取并进入待机（STATE_IDLE）。
    """
    state = {
        "command": "finishTask",
        "timestamp": time.time(),
    }
    try:
        with open(NAV_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(state, f, ensure_ascii=False)
        print(f"[WS] finishTask 已写入任务结束指令到 {NAV_STATE_FILE}")
    except Exception as exc:
        print(f"[WS] finishTask 写入失败: {exc}")


# ================= WebSocket 回调 =================
def on_open(ws):
    connected_event.set()
    print(f"[WS] 连接成功: {WS_URL}")


def on_message(ws, message):
    print(f"[WS] 收到消息: {message}")

    try:
        payload = json.loads(message)
    except Exception:
        # 非 JSON 消息按原样打印即可，不做其他处理
        return

    if not isinstance(payload, dict):
        return

    msg_type = payload.get("type")

    # 心跳
    if msg_type == "ping":
        pong_msg = build_pong_message(payload.get("timestamp"))
        ws.send(pong_msg)
        print(f"[WS] 已回复心跳: {pong_msg}")
        return

    # 自动导航任务
    if msg_type == "RobotDogTaskTurning":
        _handle_robotdog_task_turning(payload)
        return

    # 暂停 / 继续 自动导航任务
    if msg_type == "stopTask":
        _handle_stop_task(payload)
        return

    # 返航任务
    if msg_type == "backTask":
        _handle_back_task(payload)
        return

    # 任务结束（进入待机，可再下发返航）
    if msg_type == "finishTask":
        _handle_finish_task(payload)
        return


def on_error(ws, error):
    print(f"[WS] 连接异常: {error}")


def on_close(ws, close_status_code, close_msg):
    connected_event.clear()
    print(f"[WS] 连接关闭: code={close_status_code}, msg={close_msg}")


# ================= WebSocket 线程 =================
def task_websocket():
    """
    长连接线程：
      - run_forever 负责维持单次连接生命周期
      - 连接断开后按指数退避重连
    """
    reconnect_wait = RECONNECT_MIN_SECONDS

    while not stop_event.is_set():
        ws_app = websocket.WebSocketApp(
            WS_URL,
            on_open=on_open,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close,
        )

        try:
            print(f"[WS] 尝试连接: {WS_URL}")
            ws_app.run_forever(
                ping_interval=20,
                ping_timeout=8,
                ping_payload="keepalive",
                http_proxy_host=None,
                http_proxy_port=None,
            )
        except Exception as exc:
            print(f"[WS] run_forever 异常: {exc}")

        if stop_event.is_set():
            break

        print(f"[WS] {reconnect_wait}s 后重连...")
        time.sleep(reconnect_wait)
        reconnect_wait = min(reconnect_wait * 2, RECONNECT_MAX_SECONDS)


# ================= 主程序 =================
if __name__ == "__main__":
    ws_thread = threading.Thread(target=task_websocket, daemon=True)
    ws_thread.start()

    # 等待首次连接结果（超时仅提示，不退出）
    if not connected_event.wait(timeout=OPEN_TIMEOUT_SECONDS):
        print("[WS] 首次连接超时，已进入后台自动重连模式")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[WS] 收到退出信号，准备停止...")
        stop_event.set()
        ws_thread.join(timeout=2)
        print("[WS] 已退出")
