#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
zed_only.py — 纯视觉导航（仅使用 ZED 相机，无 RTK/GPS）

坐标系约定（ZED RIGHT_HANDED_Z_UP）：
  - X 轴：向右，Y 轴：向前，Z 轴：向上
  - Yaw 正方向：逆时针（俯视），右手坐标系
  - 坐标原点：程序启动时 ZED 的初始位置

所有位置和航向均基于 ZED 视觉里程计，单位：米 / 度。
航点文件格式：[[x1, y1], [x2, y2], ...]（ZED 世界坐标系，米）
"""

import time
import sys
import datetime
import threading
import math
import json
import os

import pyzed.sl as sl
import matplotlib.pyplot as plt
from flask import Flask, jsonify
from flask_cors import CORS

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.b2.sport.sport_client import SportClient


# ================= ZED 坐标约定 =================
# yaw_raw: 逆时针为正（右手系）
# 导航内部统一使用顺时针航向（CW），故取 ZED_YAW_SIGN = -1
# current_heading_cw = ZED_YAW_SIGN * zed_yaw_raw
ZED_YAW_SIGN = -1

# ================= 全局变量 - ZED =================
zed_yaw_raw = None      # ZED 原始 yaw（度，CCW 正）
zed_tx = None           # ZED 当前 x 位置（右方向，米）
zed_ty = None           # ZED 当前 y 位置（前方向，米）
zed_ready = False       # ZED 是否已就绪

# ================= 机械狗状态机 =================
STATE_IDLE   = 1
STATE_CRUISE = 2
STATE_PAUSE  = 3
STATE_RETURN = 4
STATE_CHARGE = 5

robot_state = STATE_IDLE
current_wp_index = 0
waypoints = []
return_waypoints = []
return_index = 0
target_pause_wp_index = None
target_pause_done = False

# ================= 控制参数 =================
max_yaw_rate  = 0.4
max_vx        = 0.8
kp_dist       = 0.9
arrival_radius = 0.2
vx_current    = 0.0
# 正误差（当前顺时针偏多）→ 需逆时针修正 → 正 vyaw（sport_client 中 vy 正=左/CCW）
VYAW_SIGN = 1

# ================= 文件通信 =================
PAUSE_STATE_FILE = "tmp/robotdog_nav_pause_state.json"
NAV_STATE_FILE   = "tmp/robotdog_nav_state.json"


# ================= 工具函数 =================
def normalize_angle(angle):
    """归一化到 (-180, 180]"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


# ================= Flask 服务 =================
app = Flask(__name__)
CORS(app)


@app.route('/api/location', methods=['GET'])
def get_location():
    if zed_tx is not None and zed_ty is not None:
        return jsonify({
            'x':          zed_tx,
            'y':          zed_ty,
            'yaw_raw':    zed_yaw_raw,
            'heading_cw': ZED_YAW_SIGN * zed_yaw_raw if zed_yaw_raw is not None else None,
            'vx':         vx_current,
            'timestamp':  time.time(),
        })
    return jsonify({'error': 'ZED not ready'}), 503


def start_flask_server():
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


# ================= ZED 线程 =================
def task_zed():
    global zed_yaw_raw, zed_tx, zed_ty, zed_ready

    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.AUTO
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
    init_params.coordinate_units  = sl.UNIT.METER

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print("[ZED] 打开相机失败:", status)
        return

    tracking_params = sl.PositionalTrackingParameters()
    tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3
    status = zed.enable_positional_tracking(tracking_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print("[ZED] 启动定位失败:", status)
        return

    print("[ZED] 相机已启动，视觉里程计运行中...")

    pose = sl.Pose()

    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            tracking_state = zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)

            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                # ---- 位置（X=右, Y=前）----
                translation = pose.get_translation()
                zed_tx = translation.get()[0]
                zed_ty = translation.get()[1]

                # ---- 航向（四元数 → yaw）----
                orientation = pose.get_orientation()
                qx = orientation.get()[0]
                qy = orientation.get()[1]
                qz = orientation.get()[2]
                qw = orientation.get()[3]

                sin_yaw = 2 * (qw * qz + qx * qy)
                cos_yaw = 1 - 2 * (qy * qy + qz * qz)
                zed_yaw_raw = math.degrees(math.atan2(sin_yaw, cos_yaw))

                if not zed_ready:
                    zed_ready = True
                    print("[ZED] 位姿数据就绪，坐标原点已锁定")

        time.sleep(0.01)


# ================= 暂停控制 =================
def is_navigation_paused():
    try:
        with open(PAUSE_STATE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        return bool(data.get("paused"))
    except Exception:
        return False


def write_navigation_pause_state(paused, reason=None):
    state = {"paused": bool(paused), "timestamp": time.time()}
    if reason:
        state["reason"] = reason
    try:
        with open(PAUSE_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(state, f, ensure_ascii=False)
        print(f"[暂停] 已更新暂停状态 paused={bool(paused)}")
    except Exception as exc:
        print(f"[暂停] 写入失败: {exc}")


# ================= 状态机控制 =================
def trigger_return_mode():
    global robot_state, current_wp_index, return_waypoints, return_index

    if not waypoints:
        print("[返航] 无航点，忽略 backTask")
        return

    if current_wp_index <= 0:
        start_idx = 0
    elif current_wp_index >= len(waypoints):
        start_idx = len(waypoints) - 1
    else:
        start_idx = current_wp_index - 1

    segment = waypoints[: start_idx + 1]
    if not segment:
        print("[返航] 返航段为空")
        return

    return_waypoints = list(reversed(segment))
    return_index = 0
    robot_state = STATE_RETURN
    print(f"[返航] 从索引 {start_idx} 逆序返航，共 {len(return_waypoints)} 个点")


def trigger_start_task(waypoint_file):
    global robot_state, current_wp_index, return_waypoints, return_index, target_pause_done

    if not waypoint_file:
        waypoint_file = "tmp/robotdog_turning_waypoints.json"

    ok = load_waypoints_from_file(waypoint_file)
    if not ok:
        print(f"[导航] 航点加载失败，保持待机: {waypoint_file}")
        robot_state = STATE_IDLE
        return

    current_wp_index = 0
    return_index = 0
    return_waypoints = []
    target_pause_done = False
    robot_state = STATE_CRUISE
    print(f"[导航] startTask 已加载航点并开始巡航: {waypoint_file}")


def read_nav_state():
    global robot_state

    try:
        with open(NAV_STATE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return

    cmd = data.get("command")
    if cmd == "finishTask":
        robot_state = STATE_IDLE
        try:
            os.remove(NAV_STATE_FILE)
        except Exception:
            pass
        return
    if cmd == "startTask":
        trigger_start_task(data.get("waypoint_file"))
        try:
            os.remove(NAV_STATE_FILE)
        except Exception:
            pass
        return
    if cmd == "backTask":
        if robot_state != STATE_RETURN:
            trigger_return_mode()
        try:
            os.remove(NAV_STATE_FILE)
        except Exception:
            pass


# ================= 自动导航 =================
def navigate_to(target_x, target_y, sport_client):
    """
    ZED 纯视觉导航控制（X=右，Y=前坐标系）。

    0.9m 以外：三阶段锁头（粗锁/细锁/吸附） + 渐进加速，vyaw 转向
    0.9m 以内：vyaw=0（机头朝向不变），用 vx/vy 联合调整位置

    航向约定：
      - current_heading_cw = -zed_yaw_raw（ZED CCW → 导航 CW）
      - target_heading_cw  = atan2(dx, dy)（目标从 +Y 轴顺时针的角度）
      - error = normalize(current_heading_cw - target_heading_cw)
      - 正误差 → 当前朝向偏顺时针 → 需逆时针修正 → 正 vyaw ✓
    """
    global vx_current

    print(f"\n========== 开始导航 → 目标:({target_x:.3f}, {target_y:.3f}) ==========\n")

    yaw_gain_coarse = 0.08
    yaw_gain_fine   = 0.05
    yaw_gain_lock   = 0.03

    brake_strength  = 3.5
    yaw_acc_limit   = 0.05

    coarse_threshold = 40.0
    fine_threshold   = 12.0
    heading_scale    = 35.0

    accel_limit = 0.02
    min_vx      = 0.3

    last_vyaw   = 0.0
    vy_cmd      = 0.0
    vyaw        = 0.0
    print_timer = time.time()

    paused_last = False
    prev_cur_x  = None
    prev_cur_y  = None

    while True:
        # ---- 任务结束 ----
        read_nav_state()
        if robot_state == STATE_IDLE:
            sport_client.StopMove()
            vx_current = 0.0
            print("\n[导航] 收到任务结束指令，已停止并进入待机")
            return

        # ---- 暂停控制 ----
        if is_navigation_paused():
            if not paused_last:
                sport_client.StopMove()
                print("\n[导航] 收到暂停指令，已停下")
                last_vyaw = 0.0
                vx_current = 0.0
                prev_cur_x = None
                prev_cur_y = None
                paused_last = True
            time.sleep(0.05)
            continue
        else:
            if paused_last:
                print("\n[导航] 收到继续指令，恢复导航")
                paused_last = False

        if not zed_ready or zed_tx is None or zed_yaw_raw is None:
            time.sleep(0.05)
            continue

        cur_x = zed_tx
        cur_y = zed_ty

        dx = target_x - cur_x
        dy = target_y - cur_y
        dist = math.hypot(dx, dy)

        # 中点补偿（防止高速越点）
        midpoint_dist = None
        if prev_cur_x is not None and prev_cur_y is not None:
            mid_x = (cur_x + prev_cur_x) / 2.0
            mid_y = (cur_y + prev_cur_y) / 2.0
            midpoint_dist = math.hypot(target_x - mid_x, target_y - mid_y)

        if dist < arrival_radius or (
            midpoint_dist is not None and midpoint_dist < arrival_radius
        ):
            sport_client.StopMove()
            if midpoint_dist is not None and midpoint_dist < arrival_radius and dist >= arrival_radius:
                print(f"\n已到达目标点（中点补偿判定 {midpoint_dist:.2f}m）\n")
            else:
                print("\n已到达目标点\n")
            break

        # 目标方向的顺时针航向（从 ZED 前向 +Y 起算，顺时针为正）
        # atan2(dx, dy)：dx>0(右) → 90°，dy>0(前) → 0°，dx<0(左) → -90°
        target_heading_cw = math.degrees(math.atan2(dx, dy))

        # ZED yaw 转顺时针：CW = -CCW
        current_heading_cw = ZED_YAW_SIGN * zed_yaw_raw

        error = normalize_angle(current_heading_cw - target_heading_cw)
        abs_error = abs(error)

        if dist >= 0.9:
            # ---- 远距离阶段（≥ 0.9m）：三阶段锁头 + 渐进加速 ----
            if abs_error > coarse_threshold:
                yaw_gain   = yaw_gain_coarse
                stage      = "粗锁"
                allow_move = False
            elif abs_error > fine_threshold:
                yaw_gain   = yaw_gain_fine
                stage      = "细锁"
                allow_move = False
            else:
                yaw_gain   = yaw_gain_lock
                stage      = "吸附"
                allow_move = True

            # tanh 限幅
            vyaw_target = max_yaw_rate * math.tanh(yaw_gain * error / max_yaw_rate)

            # 动态刹车
            braking_angle = (last_vyaw ** 2) / (2 * brake_strength + 1e-6)
            brake_factor  = math.exp(-abs_error / (braking_angle + 0.5))
            vyaw_target  *= (1 - brake_factor)

            # 角加速度限制
            delta     = max(min(vyaw_target - last_vyaw, yaw_acc_limit), -yaw_acc_limit)
            vyaw      = last_vyaw + delta
            last_vyaw = vyaw
            vyaw_send = VYAW_SIGN * vyaw

            # 动态误差阈值（速度越快要求越严）
            dynamic_threshold = max(5.0, fine_threshold - vx_current * 6.0)
            if abs_error > dynamic_threshold:
                allow_move = False

            # ---- 前进速度 ----
            if allow_move:
                heading_factor = 1.0 / (1.0 + (error / heading_scale) ** 2)
                vx_target = max(min(kp_dist * dist, max_vx) * heading_factor, min_vx)
            else:
                vx_target = 0.0

            if vx_target > vx_current:
                vx_current = min(vx_current + accel_limit, vx_target)
            else:
                vx_current = vx_target

            sport_client.Move(vx_current, 0, vyaw_send)

        else:
            # ---- 近距离阶段（< 0.9m）：vyaw=0，vx/vy 联合调整位置 ----
            stage = "近点调整"
            last_vyaw = 0.0
            vyaw      = 0.0
            vyaw_send = 0.0

            # 将世界坐标系位置误差投影到机体坐标系
            # ZED 世界系：X=右，Y=前；yaw_raw 逆时针正
            # 机体前向在世界系：(-sin(yaw), cos(yaw))
            # 机体左向在世界系（vy正=左）：(-cos(yaw), -sin(yaw))
            #
            # 推导：
            #   vx_body = dot((dx,dy), forward) = -dx*sin(yaw) + dy*cos(yaw)
            #   vy_body = dot((dx,dy), left)    = -dx*cos(yaw) - dy*sin(yaw)
            yaw_rad = math.radians(zed_yaw_raw)
            vx_body =  -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
            vy_body =  -dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)

            close_kp    = 0.5
            close_min_v = 0.3
            close_max_v = 0.6
            vx_cmd = max(min(close_kp * vx_body, close_max_v), -close_max_v)
            vy_cmd = max(min(close_kp * vy_body, close_max_v), -close_max_v)
            # 最低速保护：非零分量不足 0.3m/s 时补足（机械狗过低速不响应）
            if 1e-4 < abs(vx_cmd) < close_min_v:
                vx_cmd = math.copysign(close_min_v, vx_cmd)
            if 1e-4 < abs(vy_cmd) < close_min_v:
                vy_cmd = math.copysign(close_min_v, vy_cmd)

            vx_current = math.hypot(vx_cmd, vy_cmd)
            sport_client.Move(vx_cmd, vy_cmd, 0.0)

        # ---- 状态输出 ----
        if time.time() - print_timer > 0.1:
            extra = f" | vy:{vy_cmd:6.3f}" if dist < 0.9 else ""
            print(
                f"\r阶段:{stage} | "
                f"距离:{dist:6.2f}m | "
                f"误差:{error:7.2f}° | "
                f"角速:{vyaw:6.3f} | "
                f"ZED_yaw(CCW):{zed_yaw_raw:7.2f}° | "
                f"目标方向(CW):{target_heading_cw:7.2f}° | "
                f"前速:{vx_current:6.3f}"
                f"{extra}",
                end=""
            )
            print_timer = time.time()

        prev_cur_x = cur_x
        prev_cur_y = cur_y
        time.sleep(0.05)


# ================= 地图显示线程 =================
def task_plot():
    plt.ion()
    fig, ax = plt.subplots()

    while True:
        ax.clear()

        # 原点（ZED 启动位置）
        ax.scatter(0, 0, c='red', s=100, label='Start(Origin)', zorder=5)

        # 航点
        if waypoints:
            xs = [p[0] for p in waypoints]
            ys = [p[1] for p in waypoints]
            ax.scatter(xs, ys, c='blue', s=60, label='Waypoints')
            for i, (x, y) in enumerate(waypoints):
                ax.annotate(str(i + 1), (x, y),
                            textcoords="offset points", xytext=(5, 5), fontsize=9)
            # 连线
            ax.plot(xs, ys, 'b--', linewidth=0.8, alpha=0.5)

        # 当前位置与朝向
        if zed_tx is not None and zed_ty is not None:
            ax.scatter(zed_tx, zed_ty, c='green', s=100, label='Dog', zorder=5)

            if zed_yaw_raw is not None:
                # 前向：(-sin(yaw), cos(yaw)) 在 ZED 世界系
                yaw_rad = math.radians(zed_yaw_raw)
                arrow_len = 0.3
                adx = arrow_len * (-math.sin(yaw_rad))
                ady = arrow_len *   math.cos(yaw_rad)
                ax.arrow(
                    zed_tx, zed_ty, adx, ady,
                    head_width=0.12, head_length=0.12,
                    fc='green', ec='green', linewidth=2
                )

        # 文字信息
        if zed_tx is not None and zed_yaw_raw is not None:
            info = (
                f"X(右): {zed_tx:7.3f} m\n"
                f"Y(前): {zed_ty:7.3f} m\n"
                f"Yaw CCW: {zed_yaw_raw:7.2f}°\n"
                f"Heading CW: {ZED_YAW_SIGN * zed_yaw_raw:7.2f}°"
            )
        else:
            info = "ZED not ready"

        ax.text(0.02, 0.98, info, transform=ax.transAxes, fontsize=11,
                verticalalignment='top',
                bbox=dict(facecolor='white', alpha=0.7))

        ax.set_xlabel("X / 右 (m)")
        ax.set_ylabel("Y / 前 (m)")
        ax.set_title("ZED-Only Navigation Map")
        ax.legend(loc='upper right')
        ax.grid(True)
        ax.axis("equal")
        plt.pause(0.5)


# ================= 航点加载 =================
def load_waypoints_from_file(waypoint_file):
    """
    从 JSON 文件加载航点（ZED 世界坐标，米）。
    文件格式：[[x1, y1], [x2, y2], ...]
    """
    global waypoints

    try:
        with open(waypoint_file, "r", encoding="utf-8") as f:
            points = json.load(f)
    except Exception as exc:
        print(f"[航点] 读取失败 {waypoint_file}: {exc}")
        return False

    if not isinstance(points, list):
        print("[航点] 文件格式错误，应为数组")
        return False

    waypoints.clear()
    for idx, item in enumerate(points):
        try:
            x = float(item[0])
            y = float(item[1])
            waypoints.append((x, y))
            print(f"[航点] {idx + 1}: (x={x:.3f}, y={y:.3f})")
        except Exception:
            print(f"[航点] 第 {idx} 个点格式错误: {item}")

    print(f"[航点] 共加载 {len(waypoints)} 个航点")
    return len(waypoints) > 0


# ================= 导航主循环 =================
def run_navigation_loop(sport_client):
    global robot_state, current_wp_index, return_index
    global target_pause_wp_index, target_pause_done

    print("\n========== 开始多航点导航（状态机常驻）==========")
    idle_logged = False

    while True:
        read_nav_state()

        if robot_state == STATE_CRUISE:
            idle_logged = False
            if not waypoints:
                print("[导航] 无可用航点，切回待机")
                robot_state = STATE_IDLE
                time.sleep(0.1)
                continue
            if current_wp_index >= len(waypoints):
                print(
                    f"\n{'='*50}\n"
                    f"[导航] 本次巡航完成（共 {len(waypoints)} 个航点）\n"
                    f"{'='*50}"
                )
                try:
                    ans = input("是否再次执行巡航？[y/N]：").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    ans = 'n'
                if ans == 'y':
                    current_wp_index = 0
                    target_pause_done = False
                    print("[导航] 重新开始巡航...\n")
                else:
                    print("[导航] 进入待机状态")
                    robot_state = STATE_IDLE
                time.sleep(0.1)
                continue

            x, y = waypoints[current_wp_index]
            print(
                f"\n>>> [CRUISE] 前往航点 {current_wp_index + 1}/{len(waypoints)} "
                f"目标:({x:.3f}, {y:.3f})"
            )
            navigate_to(x, y, sport_client)

            if (
                target_pause_wp_index is not None
                and not target_pause_done
                and current_wp_index == target_pause_wp_index
            ):
                write_navigation_pause_state(True, reason="target_nearest_waypoint")
                target_pause_done = True
                print(f"[target] 到达目标航点 {target_pause_wp_index + 1}，已自动暂停")

            current_wp_index += 1

        elif robot_state == STATE_RETURN:
            idle_logged = False
            if not return_waypoints or return_index >= len(return_waypoints):
                print("[返航] 返航完成，进入待机")
                robot_state = STATE_IDLE
                time.sleep(0.1)
                continue

            x, y = return_waypoints[return_index]
            print(
                f"\n>>> [RETURN] 前往返航点 {return_index + 1}/{len(return_waypoints)} "
                f"目标:({x:.3f}, {y:.3f})"
            )
            navigate_to(x, y, sport_client)
            return_index += 1

        elif robot_state == STATE_CHARGE:
            print("[状态] 充电模式（占位）")
            time.sleep(0.2)

        elif robot_state == STATE_IDLE:
            if not idle_logged:
                print("[状态] 待机，等待指令...")
                idle_logged = True
            time.sleep(0.2)

        else:
            time.sleep(0.1)


# ================= 主程序 =================
if __name__ == "__main__":
    """
    用法：
      1) python3 zed_only.py eno1
         → 手动采集航点（在 ZED 坐标系中记录）

      2) python3 zed_only.py eno1 waypoints.json
         → 从文件加载航点并自动导航
    """
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface [waypoint_json_file]")
        sys.exit(-1)

    network_interface = sys.argv[1]
    waypoint_file = sys.argv[2] if len(sys.argv) >= 3 else None

    ChannelFactoryInitialize(0, network_interface)

    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    # 启动后台线程
    threading.Thread(target=task_zed,           daemon=True).start()
    threading.Thread(target=start_flask_server, daemon=True).start()
    threading.Thread(target=task_plot,          daemon=True).start()

    print("ZED-Only 导航已启动，API: http://0.0.0.0:5000/api/location")
    print("等待 ZED 就绪...")
    while not zed_ready:
        time.sleep(0.2)
    print("[主程序] ZED 就绪，坐标原点已确定\n")

    # ==========================
    # 航点来源：文件 / 手动
    # ==========================
    if waypoint_file is not None:
        print(f"[主程序] 使用文件航点: {waypoint_file}")
        ok = load_waypoints_from_file(waypoint_file)
        if not ok:
            print("[主程序] 航点加载失败，退出")
            sys.exit(1)
    else:
        print("========== 航点采集模式 ==========")
        print("将机器人移动到目标点，按 Enter 记录，输入 q 结束采集\n")

        while True:
            cmd = input("记录航点 or q退出：")
            if cmd.lower() == 'q':
                break
            if not zed_ready or zed_tx is None:
                print("[警告] ZED 未就绪，请稍候")
                continue

            x, y = zed_tx, zed_ty
            waypoints.append((x, y))
            print(
                f"航点 {len(waypoints)} 已记录: "
                f"(x={x:.3f}, y={y:.3f})  "
                f"yaw={zed_yaw_raw:.2f}°"
            )

        if waypoints:
            default_path = (
                f"waypoints_zed_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            )
            save_path = input(
                f"保存航点到文件（回车使用默认 {default_path}）："
            ).strip()
            if not save_path:
                save_path = default_path
            try:
                with open(save_path, "w", encoding="utf-8") as f:
                    json.dump([[x, y] for x, y in waypoints], f,
                              ensure_ascii=False, indent=2)
                print(f"[航点] 已保存 {len(waypoints)} 个航点到 {save_path}")
            except Exception as exc:
                print(f"[航点] 保存失败: {exc}")
        else:
            print("[航点] 未采集任何航点")

    # ==========================
    # 开始导航（状态机驱动）
    # ==========================
    if waypoints:
        current_wp_index = 0
        return_index = 0
        robot_state = STATE_CRUISE
    else:
        robot_state = STATE_IDLE

    run_navigation_loop(sport_client)
