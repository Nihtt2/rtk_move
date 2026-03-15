#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test.py — 通过 4G 模块串口读取的 RTK+ZED 融合导航

与 rtk.py 的通信方式一致：差分/定位数据由 4G 模块侧提供，
本程序仅通过串口读取 AGRICA，不再依赖主机外网 CORS/NTRIP。

RTK + ZED 融合导航：
  - RTK 航向角仅用于首次启动校准，建立 ZED 相对于真北的偏移量
  - ZED 相机航向角（校准后）作为主要转向控制来源
  - RTK GPS 坐标用于位置/距离计算

坐标约定：
  ZED RIGHT_HANDED_Z_UP 中 yaw 正方向 = 逆时针（俯视）
  罗盘/RTK 航向正方向 = 顺时针
  因此校准公式：calibrated_heading = (offset - zed_yaw) % 360
  若实测方向相反，将 ZED_YAW_SIGN 改为 +1
"""

import time
import sys
import datetime
import serial
import threading
import math
import pyzed.sl as sl
import matplotlib.pyplot as plt
from pyproj import Transformer, Geod
from flask import Flask, jsonify
from flask_cors import CORS
import json
import os
import urllib.request
import urllib.error

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.b2.sport.sport_client import SportClient

# ================= ZED 坐标约定 =================
# RIGHT_HANDED_Z_UP: X=右, Y=前, Z=上；俯视时正 yaw = 逆时针，与罗盘顺时针相反，故符号取 -1
# 如果实测校准后转向方向相反，将此值改为 1
ZED_YAW_SIGN = -1

# ================= 串口 =================
serport = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
serport.flushInput()
time.sleep(0.5)

# ================= 全局变量 - RTK =================
last_utm_x = None
last_utm_y = None
origin_utm_x = None
origin_utm_y = None
filtered_heading = None     # RTK 滤波航向，仅用于校准阶段
gga_quality = 0
rtk_heading_status = 0   # RTK 航向状态，0=无效，4/5=双天线固定解
latest_fields = None
lon = None
lat = None
waypoints = []
gps_waypoints = []  # 手动记录航点时，同时保存对应的经纬度用于上传给前端

# ================= 机械狗状态机 =================
STATE_IDLE = 1      # 待机
STATE_CRUISE = 2    # 正常巡航
STATE_PAUSE = 3     # 暂停（目前主要由 PAUSE_STATE_FILE 控制）
STATE_RETURN = 4    # 返航
STATE_CHARGE = 5    # 充电模式（预留）

robot_state = STATE_IDLE
current_wp_index = 0
return_waypoints = []   # 返航用航点序列（UTM）
return_index = 0        # 当前返航目标索引
target_pause_wp_index = None   # RobotDogTaskTurning.target 对应的最近航点索引
target_pause_done = False      # 是否已触发过一次目标点自动暂停

# ================= 全局变量 - ZED =================
zed_yaw_raw = None              # ZED 原始 yaw（度，RIGHT_HANDED_Z_UP）
zed_calibrated_heading = None   # 校准后的 ZED 航向（真北顺时针 0-360°）
zed_calibration_offset = None   # 校准偏移量：offset = rtk_ref - ZED_YAW_SIGN * zed_ref
zed_calibrated = False          # 是否已完成 RTK->ZED 校准

# ================= 全局变量 - ZED定位 =================

stable_rtk_x = None
stable_rtk_y = None

fused_utm_x = None
fused_utm_y = None

zed_ref_tx = None
zed_ref_ty = None
zed_last_raw_yaw = None  # 用于检测“仅旋转”时避免 ZED 位置漂移

# ================= 控制参数 =================
alpha = 0.2
kp_dist = 0.9
max_yaw_rate = 0.4
max_vx = 0.8
arrival_radius = 0.2
vx_current = 0.0
# 若实际转向与预期相反（靠近目标时一直转圈），改为 -1
VYAW_SIGN = 1

# ================= 导航暂停控制 =================
# 与 WebSocket 客户端通过文件通信，实现“狗停下但后台线程不停止”
PAUSE_STATE_FILE = "tmp/robotdog_nav_pause_state.json"

# backTask 等导航状态控制命令通过此文件传递
NAV_STATE_FILE = "tmp/robotdog_nav_state.json"

# ================= 路线上传接口配置 =================
CONFIG_PATH = "config.json"


def _load_android_config():
    """从 config.json 读取 Android 端 host 和 api_port(8080)，失败时回退到默认值"""
    config_path = CONFIG_PATH
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        host = cfg.get("android_host", "10.65.42.98")
        port = int(cfg.get("api_port", 8080))
        return host, port
    except Exception:
        return "10.65.42.98", 8080


_ANDROID_HOST, _API_PORT = _load_android_config()
ROUTE_UPLOAD_URL = f"http://{_ANDROID_HOST}:{_API_PORT}/api/robot-route/upload"


# ================= 工具函数 =================
def normalize_angle(angle):
    """将角度归一化到 (-180, 180]"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def get_utm_epsg(lon, lat):
    zone = int((lon + 180) / 6) + 1
    if lat >= 0:
        return 32600 + zone
    else:
        return 32700 + zone


def compute_route_length_meters(gps_coords):
    """根据经纬度列表计算路径总长度（米），WGS84 大地线。点数<2 返回 0。

    约定 gps_coords 中每个点为 [lon, lat]。
    """
    if not gps_coords or len(gps_coords) < 2:
        return 0.0
    geod = Geod(ellps="WGS84")
    total = 0.0
    for i in range(len(gps_coords) - 1):
        lon1, lat1 = float(gps_coords[i][0]), float(gps_coords[i][1])
        lon2, lat2 = float(gps_coords[i + 1][0]), float(gps_coords[i + 1][1])
        _, _, dist = geod.inv(lon1, lat1, lon2, lat2)
        total += dist
    return total


# ================= AGRICA 解析 =================
def parse_agrica(line):
    try:
        if "#AGRICA" not in line:
            return None
        data_part = line.split(";")[1]
        data_part = data_part.split("*")[0]
        return data_part.split(",")
    except:
        return None


# ================= Flask 后端服务 =================
app = Flask(__name__)
CORS(app)


@app.route('/api/rtk_location', methods=['GET'])
def get_rtk_location():
    global lon, lat, gga_quality
    if lon is not None and lat is not None:
        return jsonify({
            'lon': lon,
            'lat': lat,
            'quality': gga_quality,
            'timestamp': time.time(),
            'heading': zed_calibrated_heading,
            'vx': vx_current
        })
    else:
        return jsonify({'error': 'RTK data not available'}), 503


def start_flask_server():
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


# ================= RTK 线程（串口读取 AGRICA）=================
def task_rtk():
    global last_utm_x, last_utm_y, origin_utm_x, origin_utm_y
    global filtered_heading, gga_quality, rtk_heading_status, latest_fields, lon, lat
    global stable_rtk_x, stable_rtk_y
    global fused_utm_x, fused_utm_y
    global zed_ref_tx, zed_ref_ty
    global zed_last_raw_yaw

    while True:
        line = serport.readline().decode(errors='ignore').strip()
        fields = parse_agrica(line)
        if not fields:
            continue

        latest_fields = fields

        try:
            rtk_status = int(fields[8])
            heading_status = int(fields[9])
            heading = float(fields[19])
            lat = float(fields[29])
            lon = float(fields[30])
        except:
            continue

        gga_quality = rtk_status
        rtk_heading_status = heading_status

        # 仅在 heading_status 4/5（双天线固定解）时更新 RTK 滤波航向
        if heading_status in [4, 5]:
            if filtered_heading is None:
                filtered_heading = heading
            else:
                diff = normalize_angle(heading - filtered_heading)
                filtered_heading = filtered_heading + alpha * diff
                filtered_heading %= 360

        epsg = get_utm_epsg(lon, lat)
        transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
        utm_x, utm_y = transformer.transform(lon, lat)

        last_utm_x = utm_x
        last_utm_y = utm_y
        
        if rtk_status == 4:

            stable_rtk_x = utm_x
            stable_rtk_y = utm_y

            fused_utm_x = utm_x
            fused_utm_y = utm_y

            # 重置ZED参考点（防止跳变）
            zed_ref_tx = None
            zed_ref_ty = None
            zed_last_raw_yaw = None

        if origin_utm_x is None and rtk_status == 4:
            origin_utm_x = utm_x
            origin_utm_y = utm_y
            print("\n[RTK] 地图原点已设定")


# ================= ZED 线程 =================
def task_zed():
    global zed_yaw_raw, zed_calibrated_heading
    global fused_utm_x, fused_utm_y
    global stable_rtk_x, stable_rtk_y
    global zed_ref_tx, zed_ref_ty
    global gga_quality
    global zed_calibration_offset
    global zed_last_raw_yaw

    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.AUTO
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
    init_params.coordinate_units = sl.UNIT.METER

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

    print("[ZED] 相机已启动，等待 RTK 校准...")

    pose = sl.Pose()

    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            tracking_state = zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)

            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                orientation = pose.get_orientation()

                qx = orientation.get()[0]
                qy = orientation.get()[1]
                qz = orientation.get()[2]
                qw = orientation.get()[3]

                # quaternion → yaw
                sin_yaw = 2 * (qw * qz + qx * qy)
                cos_yaw = 1 - 2 * (qy * qy + qz * qz)

                raw_yaw = math.degrees(math.atan2(sin_yaw, cos_yaw))
                zed_yaw_raw = raw_yaw
                
                # ================= 融合位置计算 =================

                translation = pose.get_translation()

                tx = translation.get()[0]
                ty = translation.get()[1]

                # RTK不是固定解时，用ZED桥接
                if gga_quality != 4 and stable_rtk_x is not None and zed_calibration_offset is not None:

                    if zed_ref_tx is None:

                        zed_ref_tx = tx
                        zed_ref_ty = ty

                    dx_zed = tx - zed_ref_tx   # ZED X = 向右位移
                    dy_zed = ty - zed_ref_ty   # ZED Y = 向前位移

                    # 旋转到 UTM（东、北）
                    # ZED RIGHT_HANDED_Z_UP: X=右, Y=前, Z=上
                    # 机头朝向 H₀ = zed_calibration_offset（真北顺时针 0–360°）
                    # 前向(Y) → (sin(H₀), cos(H₀))；右侧(X) → (cos(H₀), -sin(H₀))
                    # dE = dx_zed*cos(H₀) + dy_zed*sin(H₀),  dN = -dx_zed*sin(H₀) + dy_zed*cos(H₀)
                    h0_rad = math.radians(zed_calibration_offset)

                    dx_utm = (
                        dx_zed * math.cos(h0_rad)
                        + dy_zed * math.sin(h0_rad)
                    )
                    dy_utm = (
                        -dx_zed * math.sin(h0_rad)
                        + dy_zed * math.cos(h0_rad)
                    )

                    # 仅旋转时 ZED 视觉容易漂移，导致“距离越判越远”；此时不更新位置
                    delta_yaw = abs(normalize_angle(raw_yaw - zed_last_raw_yaw)) if zed_last_raw_yaw is not None else 0
                    linear_m = math.hypot(dx_zed, dy_zed)
                    if zed_last_raw_yaw is None or delta_yaw <= 3.0 or linear_m >= 0.02:
                        fused_utm_x = stable_rtk_x + dx_utm
                        fused_utm_y = stable_rtk_y + dy_utm
                    zed_last_raw_yaw = raw_yaw

                # 校准完成后实时更新校准后航向
                if zed_calibration_offset is not None:
                    zed_calibrated_heading = (ZED_YAW_SIGN * raw_yaw + zed_calibration_offset) % 360

        time.sleep(0.01)


# ================= ZED 校准（一次性，主线程调用）=================
def calibrate_zed_with_rtk():
    """
    等待 RTK 航向稳定后，以 RTK 真北方向校准 ZED 航向偏移量。

    校准偏移公式：
        offset = rtk_heading_ref - ZED_YAW_SIGN * zed_yaw_ref

    校准后实时航向：
        calibrated_heading = (ZED_YAW_SIGN * zed_yaw_raw + offset) % 360
    """
    global zed_calibration_offset, zed_calibrated

    print("\n========== ZED 航向校准 ==========")
    print("等待 RTK 双天线航向就绪（heading_status=4/5）...")

    while filtered_heading is None:
        time.sleep(0.2)

    print(f"[校准] RTK 航向初步就绪: {filtered_heading:.2f}°，再等 3 秒稳定滤波...")
    time.sleep(3)

    print("等待 ZED 位姿就绪...")
    while zed_yaw_raw is None:
        time.sleep(0.1)

    rtk_ref = filtered_heading
    zed_ref = zed_yaw_raw

    # 计算偏移量，使得校准后航向在此刻等于 rtk_ref
    zed_calibration_offset = (rtk_ref - ZED_YAW_SIGN * zed_ref) % 360
    zed_calibrated = True

    print(f"[校准] 完成！")
    print(f"       RTK 参考航向  : {rtk_ref:.2f}°")
    print(f"       ZED 原始 yaw  : {zed_ref:.2f}°（ZED_YAW_SIGN={ZED_YAW_SIGN}）")
    print(f"       计算偏移量    : {zed_calibration_offset:.2f}°")
    print(f"       校准验证航向  : {(ZED_YAW_SIGN * zed_ref + zed_calibration_offset) % 360:.2f}° （应≈{rtk_ref:.2f}°）")
    print("       后续转向将由 ZED 主控，RTK 仅提供位置\n")


def calibrate_zed_by_walking(sport_client, walk_dist=5.0, vx=0.4, yaw_gain=0.08):
    """
    当 RTK heading_status==0 无法提供航向时，通过行走校准：
    1. 记录起点坐标和 ZED yaw
    2. 机械狗直走 walk_dist 米（用 ZED 保持直线）
    3. 根据起点、终点 UTM 坐标计算真北航向角
    4. 用该航向替代 RTK 校准 ZED
    5. 退回 walk_dist 米
    """
    global zed_calibration_offset, zed_calibrated, filtered_heading

    print("\n========== 行走校准（RTK 航向无效）==========")
    print(f"将直走 {walk_dist}m，通过坐标计算真北航向以校准 ZED...")

    # 等待位置和 ZED 就绪
    while fused_utm_x is None or fused_utm_y is None or origin_utm_x is None:
        print("[行走校准] 等待 RTK 位置就绪...")
        time.sleep(0.3)
    while zed_yaw_raw is None:
        print("[行走校准] 等待 ZED 位姿就绪...")
        time.sleep(0.1)

    # 记录起点
    x1 = fused_utm_x
    y1 = fused_utm_y
    zed_yaw_start = zed_yaw_raw
    print(f"[行走校准] 起点: UTM({x1:.2f}, {y1:.2f}), ZED_yaw={zed_yaw_start:.2f}°")

    def _walk_straight(dist_target, forward=True):
        """沿当前方向走 dist_target 米，forward=True 为前进"""
        sign = 1 if forward else -1
        start_x = fused_utm_x
        start_y = fused_utm_y
        ref_yaw = zed_yaw_raw
        last_print = time.time()
        while True:
            dx = fused_utm_x - start_x
            dy = fused_utm_y - start_y
            dist = math.hypot(dx, dy)
            if dist >= dist_target:
                sport_client.StopMove()
                break
            # 用 ZED 保持直线：偏航修正
            yaw_err = normalize_angle(ref_yaw - zed_yaw_raw) if zed_yaw_raw is not None else 0
            vyaw = max(min(yaw_gain * yaw_err, max_yaw_rate), -max_yaw_rate)
            sport_client.Move(sign * vx, 0, vyaw)
            if time.time() - last_print > 0.5:
                print(f"\r[行走校准] 已走 {dist:.2f}m / {dist_target}m", end="")
                last_print = time.time()
            time.sleep(0.05)
        print()

    # 前进 walk_dist 米
    print(f"[行走校准] 开始前进 {walk_dist}m...")
    _walk_straight(walk_dist, forward=True)
    time.sleep(0.3)  # 稳定

    # 记录终点并计算真北航向
    x2 = fused_utm_x
    y2 = fused_utm_y
    dx = x2 - x1
    dy = y2 - y1
    dist_actual = math.hypot(dx, dy)

    if dist_actual < 1.0:
        print(f"[行走校准] 错误：实际仅移动 {dist_actual:.2f}m，校准取消")
        sport_client.StopMove()
        return False

    # UTM: x=东, y=北。真北航向（顺时针0-360°）：atan2(dx, dy)
    heading_true_north = (90 -math.degrees(math.atan2(dy, dx)) ) % 360
    zed_calibration_offset = (heading_true_north - ZED_YAW_SIGN * zed_yaw_start) % 360
    zed_calibrated = True
    filtered_heading = heading_true_north  # 供备用

    print(f"[行走校准] 终点: UTM({x2:.2f}, {y2:.2f}), 实际移动 {dist_actual:.2f}m")
    print(f"[行走校准] 计算真北航向: {heading_true_north:.2f}°")
    print(f"[行走校准] ZED 偏移量: {zed_calibration_offset:.2f}°，校准完成")

    # 退回 walk_dist 米
    print(f"[行走校准] 退回 {walk_dist}m...")
    _walk_straight(walk_dist, forward=False)
    sport_client.StopMove()
    print("[行走校准] 已退回起点\n")
    return True


# ================= 获取当前导航航向 =================
def get_current_heading():
    """
    返回当前用于导航的航向角（0-360°，真北顺时针）。
    校准完成后优先使用 ZED 校准航向，否则回退到 RTK 航向。
    """
    if zed_calibrated and zed_calibrated_heading is not None:
        return zed_calibrated_heading
    return filtered_heading


def is_navigation_paused():
    """
    从 PAUSE_STATE_FILE 读取暂停状态。
    文件形如：{"paused": true/false, "timestamp": ...}
    """
    try:
        with open(PAUSE_STATE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        return bool(data.get("paused"))
    except Exception:
        return False


def write_navigation_pause_state(paused, reason=None):
    """
    写入导航暂停状态文件，供 navigate_to() 周期读取。
    """
    state = {
        "paused": bool(paused),
        "timestamp": time.time(),
    }
    if reason:
        state["reason"] = reason
    try:
        with open(PAUSE_STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(state, f, ensure_ascii=False)
        print(f"[暂停] 已更新暂停状态 paused={bool(paused)}")
    except Exception as exc:
        print(f"[暂停] 写入暂停状态失败: {exc}")


def parse_target_lonlat(raw_target):
    """
    解析 startTask 携带的 target，支持:
      - 字符串 JSON: "[lon, lat]"
      - 数组: [lon, lat]
    严格空判定：仅缺失/None/"" 视为空。
    """
    if raw_target is None or raw_target == "":
        return None

    target_obj = raw_target
    if isinstance(raw_target, str):
        try:
            target_obj = json.loads(raw_target)
        except Exception:
            print(f"[target] target 字符串解析失败: {raw_target}")
            return None

    if not isinstance(target_obj, list) or len(target_obj) < 2:
        print(f"[target] target 格式无效: {target_obj}")
        return None

    try:
        lon_val = float(target_obj[0])
        lat_val = float(target_obj[1])
        return lon_val, lat_val
    except Exception:
        print(f"[target] target 坐标类型无效: {target_obj}")
        return None


def find_nearest_waypoint_index(target_lon, target_lat):
    """
    根据经纬度 target 计算距离最近的航点索引（基于局部坐标）。
    """
    if not waypoints:
        return None
    if origin_utm_x is None or origin_utm_y is None:
        return None

    try:
        epsg = get_utm_epsg(target_lon, target_lat)
        transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
        utm_x, utm_y = transformer.transform(target_lon, target_lat)
        target_x = utm_x - origin_utm_x
        target_y = utm_y - origin_utm_y
    except Exception as exc:
        print(f"[target] target 坐标转换失败: {exc}")
        return None

    nearest_idx = None
    nearest_dist = None
    for idx, (x, y) in enumerate(waypoints):
        dist = math.hypot(target_x - x, target_y - y)
        if nearest_dist is None or dist < nearest_dist:
            nearest_dist = dist
            nearest_idx = idx

    return nearest_idx


def trigger_return_mode():
    """
    切换到返航模式：从当前目标点的前一个点开始，沿既有航点逆序返回。
    """
    global robot_state, current_wp_index, return_waypoints, return_index, waypoints

    if not waypoints:
        print("[返航] 当前无航点，忽略 backTask")
        return

    # 当前目标点的前一个点；如果已经超过末尾，则从最后一个点开始
    if current_wp_index <= 0:
        start_idx = 0
    elif current_wp_index >= len(waypoints):
        start_idx = len(waypoints) - 1
    else:
        start_idx = current_wp_index - 1

    segment = waypoints[: start_idx + 1]
    if not segment:
        print("[返航] 返航段为空，忽略 backTask")
        return

    return_waypoints = list(reversed(segment))
    return_index = 0
    robot_state = STATE_RETURN

    print(
        f"[返航] 接收到 backTask，"
        f"从航点索引 {start_idx} 开始逆序返航，共 {len(return_waypoints)} 个点"
    )


def trigger_start_task(waypoint_file, raw_target=None):
    """
    切换到巡航模式：从 waypoint_file 重新加载航点并重置索引。
    """
    global robot_state, current_wp_index, return_waypoints, return_index
    global target_pause_wp_index, target_pause_done

    if not waypoint_file:
        waypoint_file = "tmp/robotdog_turning_waypoints.json"

    ok = load_waypoints_from_file(waypoint_file)
    if not ok:
        print(f"[导航] startTask 航点加载失败，保持待机: {waypoint_file}")
        robot_state = STATE_IDLE
        return

    current_wp_index = 0
    return_index = 0
    return_waypoints = []
    target_pause_done = False

    target_pause_wp_index = None
    parsed_target = parse_target_lonlat(raw_target)
    if parsed_target is not None:
        nearest_idx = find_nearest_waypoint_index(parsed_target[0], parsed_target[1])
        if nearest_idx is not None:
            target_pause_wp_index = nearest_idx
            print(
                f"[target] 已设置目标暂停航点索引: {target_pause_wp_index + 1}/{len(waypoints)}"
            )
        else:
            print("[target] 未能计算最近航点索引，按无 target 处理")

    robot_state = STATE_CRUISE
    print(f"[导航] 接收到 startTask，已重置索引并开始巡航: {waypoint_file}")


def read_nav_state():
    """
    从 NAV_STATE_FILE 读取导航状态指令，处理 finishTask/backTask/startTask。
    """
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
        trigger_start_task(data.get("waypoint_file"), data.get("target"))
        try:
            os.remove(NAV_STATE_FILE)
        except Exception:
            pass
        return
    if cmd == "backTask":
        if robot_state != STATE_RETURN:
            trigger_return_mode()
        # 标记已处理：删除文件，避免重复触发
        try:
            os.remove(NAV_STATE_FILE)
        except Exception:
            pass


def upload_route_to_frontend(route_name, gps_coords):
    """
    调用 Android 端 /api/robot-route/upload 接口，上报机器狗当前记录的路线。

    gps_coords: [[lon, lat], [lon, lat], ...]
    """
    if not gps_coords:
        print("[路线] 无可用经纬度航点，跳过上传")
        return

    # routeName 最长 15 个字符，超出则截断
    if not route_name:
        route_name = "自定义路线"
    if len(route_name) > 15:
        route_name = route_name[:15]

    body_obj = {
        "routeName": route_name,
        "routeLength": "%.2f" % compute_route_length_meters(gps_coords),
        "coordinates": [[float(lon), float(lat)] for lon, lat in gps_coords],
    }

    data = json.dumps(body_obj, ensure_ascii=False).encode("utf-8")

    req = urllib.request.Request(
        ROUTE_UPLOAD_URL,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    print(f"[路线] 正在上传路线到前端: {ROUTE_UPLOAD_URL}")

    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            resp_body = resp.read().decode("utf-8", errors="ignore")
            print(f"[路线] 上传成功，HTTP {resp.status}: {resp_body}")
    except urllib.error.HTTPError as e:
        try:
            err_body = e.read().decode("utf-8", errors="ignore")
        except Exception:
            err_body = ""
        print(f"[路线] 上传失败 HTTP {e.code}: {err_body}")
    except Exception as e:
        print(f"[路线] 上传失败: {e}")


# ================= 自动导航 =================
def navigate_to(target_x, target_y, sport_client):
    """
    三阶段锁头 + 渐进加速 + 动态阈值导航控制。
    航向来源：ZED 校准航向（主）/ RTK 航向（备用）
    """
    global vx_current
    print("\n========== 开始导航 ==========\n")

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
    print_timer = time.time()

    paused_last = False  # 上一次循环是否处于暂停状态
    prev_cur_x = None
    prev_cur_y = None

    while True:
        # ========== 任务结束控制（finishTask -> STATE_IDLE） ==========
        read_nav_state()
        if robot_state == STATE_IDLE:
            sport_client.StopMove()
            vx_current = 0.0
            print("\n[导航] 收到任务结束指令，已停止并进入待机")
            return

        # ========== 暂停控制 ==========
        if is_navigation_paused():
            if not paused_last:
                # 首次进入暂停：立即停下
                sport_client.StopMove()
                print("\n[导航] 收到暂停指令，狗已停下，但 RTK/ZED 仍在运行")
                last_vyaw = 0.0
                vx_current = 0.0
                prev_cur_x = None
                prev_cur_y = None
                paused_last = True
            time.sleep(0.05)
            continue
        else:
            if paused_last:
                print("\n[导航] 收到继续指令，恢复自动导航")
                paused_last = False

        current_heading = get_current_heading()

        if last_utm_x is None or origin_utm_x is None or current_heading is None:
            prev_cur_x = None
            prev_cur_y = None
            time.sleep(0.05)
            continue

        cur_x = fused_utm_x - origin_utm_x
        cur_y = fused_utm_y - origin_utm_y

        dx = target_x - cur_x
        dy = target_y - cur_y
        dist = math.hypot(dx, dy)

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

        # 目标航向（真北顺时针，0-360°）
        angle = math.degrees(math.atan2(dy, dx))
        target_heading = (90 - angle) % 360

        error = normalize_angle(current_heading - target_heading)
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

            # 动态误差阈值（速度越快要求越严格）
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
            # ---- 近距离阶段（< 0.9m）：停止转向，vx/vy 联合调整位置 ----
            # vyaw 归零，保持当前机头朝向不变
            stage = "近点调整"
            last_vyaw = 0.0
            vyaw = 0.0
            vyaw_send = 0.0

            # 将世界坐标系（UTM：x=东, y=北）位置误差投影到机体坐标系
            # 机头朝向 H（真北顺时针 0-360°）
            # 机体前向：dx*sin(H) + dy*cos(H)
            # 机体左向（vy正=左）：-dx*cos(H) + dy*sin(H)
            H_rad = math.radians(current_heading)
            vx_body = dx * math.sin(H_rad) + dy * math.cos(H_rad)
            vy_body = -dx * math.cos(H_rad) + dy * math.sin(H_rad)

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
            src = "ZED" if (zed_calibrated and zed_calibrated_heading is not None) else "RTK"
            midpoint_text = (
                f" | 中点距:{midpoint_dist:6.2f}m"
                if midpoint_dist is not None else ""
            )
            if dist < 0.9:
                extra = f" | vy:{vy_cmd:6.3f}"
            else:
                extra = ""
            print(
                f"\r阶段:{stage} | "
                f"距离:{dist:6.2f}m | "
                f"误差:{error:7.2f}° | "
                f"角速:{vyaw:6.3f} | "
                f"航向[{src}]:{current_heading:7.2f}° | "
                f"目标:{target_heading:7.2f}° | "
                f"前速:{vx_current:6.3f}"
                f"{extra}"
                f"{midpoint_text}",
                end=""
            )
            print_timer = time.time()

        prev_cur_x = cur_x
        prev_cur_y = cur_y

        time.sleep(0.05)

# ================= 地图显示线程 =================

def task_plot():

    global last_utm_x, last_utm_y
    global origin_utm_x, origin_utm_y
    global filtered_heading
    global lon, lat
    global zed_yaw_raw
    global zed_calibration_offset
    global waypoints

    plt.ion()

    fig, ax = plt.subplots()

    while True:

        ax.clear()

        # 原点
        if origin_utm_x is not None:

            ax.scatter(
                0, 0,
                c='red',
                s=100,
                label='Origin'
            )

        # 航点
        if waypoints:

            xs = [p[0] for p in waypoints]
            ys = [p[1] for p in waypoints]

            ax.scatter(
                xs, ys,
                c='blue',
                s=60,
                label='Waypoints'
            )

        # 当前点
        if fused_utm_x is not None and origin_utm_x is not None:

            cur_x = fused_utm_x - origin_utm_x
            cur_y = fused_utm_y - origin_utm_y

            ax.scatter(
                cur_x, cur_y,
                c='green',
                s=100,
                label='Dog'
            )
            
        
        
        # ================= 朝向箭头 =================

        # 优先使用 ZED 校准航向
        if zed_yaw_raw is not None and zed_calibration_offset is not None:

            heading = (ZED_YAW_SIGN * zed_yaw_raw + zed_calibration_offset) % 360

        elif filtered_heading is not None:

            heading = filtered_heading

        else:

            heading = None


        if heading is not None:

            # 转换为数学坐标角度（X轴为0°逆时针）
            angle_rad = math.radians(90 - heading)

            arrow_length = 0.2   # 箭头长度（米）

            dx = arrow_length * math.cos(angle_rad)
            dy = arrow_length * math.sin(angle_rad)

            ax.arrow(
                cur_x,
                cur_y,
                dx,
                dy,
                head_width=0.2,
                head_length=0.2,
                fc='green',
                ec='green',
                linewidth=2
            )
        
        
        
        ax.set_title("Navigation Map")


        # ================= 航向角显示 =================

        zed_text = (
            f"ZED_raw: {zed_yaw_raw:7.2f}°"
            if zed_yaw_raw is not None
            else "ZED_raw:   N/A"
        )

        rtk_text = (
            f"RTK_heading: {filtered_heading:7.2f}°"
            if filtered_heading is not None
            else "RTK_heading:   N/A"
        )

        offset_text = (
            f"Offset : {zed_calibration_offset:7.2f}°"
            if zed_calibration_offset is not None
            else "Offset :   N/A"
        )

        if zed_yaw_raw is not None and zed_calibration_offset is not None:

            fused = (ZED_YAW_SIGN * zed_yaw_raw + zed_calibration_offset) % 360

            fused_text = f"ZED_verti: {fused:7.2f}°"

        else:

            fused_text = "ZED_verti:   N/A"


        info_text = (
            zed_text + "\n" +
            rtk_text + "\n" +
            offset_text + "\n" +
            fused_text
        )

        ax.text(
            0.02,
            0.98,
            info_text,
            transform=ax.transAxes,
            fontsize=12,
            verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.7)
        )


        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")

        ax.legend()

        ax.grid(True)

        ax.axis("equal")

        plt.pause(0.5)


def load_waypoints_from_file(waypoint_file):
    """
    从 JSON 文件加载经纬度航点：
    文件内容形如：
      [
        [lon, lat],
        [lon, lat],
        ...
      ]
    然后：
      - 等待 RTK 把 origin_utm_x / origin_utm_y 设好
      - 把每个经纬度转成 UTM，再减去原点，得到 (x, y) 航点
    """
    global waypoints, origin_utm_x, origin_utm_y

    try:
        with open(waypoint_file, "r", encoding="utf-8") as f:
            gps_points = json.load(f)
    except Exception as exc:
        print(f"[航点] 读取航点文件失败 {waypoint_file}: {exc}")
        return False

    if not isinstance(gps_points, list):
        print("[航点] 航点文件格式错误，应为数组")
        return False

    # 等待 RTK 原点就绪
    while origin_utm_x is None or origin_utm_y is None:
        print("[航点] 等待 RTK 原点就绪...")
        time.sleep(0.5)

    waypoints.clear()

    for idx, item in enumerate(gps_points):
        try:
            # data 里是 [lon, lat]
            lon_val = float(item[0])
            lat_val = float(item[1])
        except Exception:
            print(f"[航点] 第 {idx} 个点格式错误: {item}")
            continue

        epsg = get_utm_epsg(lon_val, lat_val)
        transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
        utm_x, utm_y = transformer.transform(lon_val, lat_val)

        x = utm_x - origin_utm_x
        y = utm_y - origin_utm_y
        waypoints.append((x, y))

        print(
            f"[航点] 已添加 {idx + 1}: "
            f"lat={lat_val:.6f}, lon={lon_val:.6f} -> (x={x:.2f}, y={y:.2f})"
        )

    print(f"[航点] 共加载 {len(waypoints)} 个航点")
    return len(waypoints) > 0       


def run_navigation_loop(sport_client):
    """
    基于状态机的导航主循环：
      - STATE_CRUISE：按顺序依次前往 waypoints 中的航点
      - STATE_RETURN：按照 return_waypoints 中的航点逆序返航
      - STATE_IDLE：保持待机并持续读取状态，等待下一条指令
    """
    global robot_state, current_wp_index, return_index, waypoints, return_waypoints
    global target_pause_wp_index, target_pause_done

    print("\n========== 开始多航点导航（状态机常驻） ==========")
    idle_logged = False

    while True:
        # 检查是否有新的导航控制指令（例如 backTask）
        read_nav_state()

        if robot_state == STATE_CRUISE:
            idle_logged = False
            if not waypoints:
                print("[导航] 巡航状态下无可用航点，切回待机")
                robot_state = STATE_IDLE
                time.sleep(0.1)
                continue
            if current_wp_index >= len(waypoints):
                print("[导航] 巡航完成，进入待机状态")
                robot_state = STATE_IDLE
                time.sleep(0.1)
                continue

            x, y = waypoints[current_wp_index]
            print(
                f"\n>>> [CRUISE] 前往航点 {current_wp_index + 1}/{len(waypoints)} "
                f"目标:({x:.2f}, {y:.2f})"
            )
            navigate_to(x, y, sport_client)

            if (
                target_pause_wp_index is not None
                and not target_pause_done
                and current_wp_index == target_pause_wp_index
            ):
                write_navigation_pause_state(True, reason="target_nearest_waypoint")
                target_pause_done = True
                print(
                    f"[target] 到达最近航点 {target_pause_wp_index + 1}，已自动进入暂停"
                )

            current_wp_index += 1

        elif robot_state == STATE_RETURN:
            idle_logged = False
            if not return_waypoints or return_index >= len(return_waypoints):
                print("[返航] 返航完成，进入待机状态")
                robot_state = STATE_IDLE
                time.sleep(0.1)
                continue

            x, y = return_waypoints[return_index]
            print(
                f"\n>>> [RETURN] 前往返航点 {return_index + 1}/{len(return_waypoints)} "
                f"目标:({x:.2f}, {y:.2f})"
            )
            navigate_to(x, y, sport_client)
            return_index += 1

        elif robot_state == STATE_CHARGE:
            # 预留：未来与充电/回桩逻辑对接
            print("[状态] 当前处于充电模式（占位），暂不执行导航")
            time.sleep(0.2)
            continue

        elif robot_state == STATE_IDLE:
            if not idle_logged:
                print("[状态] 当前为待机状态，等待下一条指令")
                idle_logged = True
            time.sleep(0.2)
            continue

        else:
            # 未知状态，简单休眠避免空转
            time.sleep(0.1)


# ================= 主程序 =================
if __name__ == "__main__":

    # 支持两种用法：
    #   1) python test.py eno1
    #        -> 手动采集航点
    #   2) python test.py eno1 /tmp/robotdog_turning_waypoints.json
    #        -> 从 JSON 文件加载经纬度航点，自动导航
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface [waypoint_json_file]")
        sys.exit(-1)

    network_interface = sys.argv[1]
    waypoint_file = sys.argv[2] if len(sys.argv) >= 3 else None

    ChannelFactoryInitialize(0, network_interface)

    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    # 启动各后台线程（4G 模块侧提供差分，本程序只读串口）
    threading.Thread(target=task_rtk,           daemon=True).start()
    threading.Thread(target=task_zed,           daemon=True).start()
    threading.Thread(target=start_flask_server, daemon=True).start()
    threading.Thread(target=task_plot,         daemon=True).start()

    print("RTK 后端服务已启动（4G 模块串口模式），API: http://0.0.0.0:5000/api/rtk_location")

    # 启动后等待 10 秒，检查 RTK 航向状态
    print("\n[校准] 等待 10 秒，检测 RTK heading_status...")
    heading_always_zero = True
    for i in range(16):  # 20 * 0.5s = 10s
        time.sleep(0.5)
        if rtk_heading_status not in (0, None):
            heading_always_zero = False
        if (i + 1) % 4 == 0:
            print(f"  {int((i + 1) * 0.5)}s, heading_status={rtk_heading_status}")

    if heading_always_zero:
        # RTK 航向一直无效，通过行走 5m 用坐标计算真北航向校准 ZED
        ok = calibrate_zed_by_walking(sport_client, walk_dist=3.0)
        if not ok:
            print("[校准] 行走校准失败，请检查 RTK 定位与 ZED 状态后重试")
            sys.exit(1)
    else:
        # RTK 航向有效，用 RTK 校准 ZED
        calibrate_zed_with_rtk()

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
        print("移动机器人到目标点，按 Enter 记录，输入 q 结束采集\n")

        while True:
            cmd = input("记录航点 or q退出：")
            if cmd.lower() == 'q':
                break

            if last_utm_x is None or origin_utm_x is None:
                print("[警告] RTK 未就绪，请稍候")
                continue

            x = fused_utm_x - origin_utm_x
            y = fused_utm_y - origin_utm_y
            waypoints.append((x, y))
            print(f"航点 {len(waypoints)} 已记录: ({x:.2f}, {y:.2f})")
            # 同步记录经纬度，用于路线上传与保存 JSON
            if lon is not None and lat is not None:
                gps_waypoints.append([lon, lat])
                print(f"当前路径长度: {compute_route_length_meters(gps_waypoints):.2f} m")

        # 手动采集模式下，采集结束后将路线上传给前端
        if gps_waypoints:
            print("\n========== 路线上传 ==========")
            route_name = input("请输入本次路线名称（不超过15个字符，回车默认“自定义路线”）：").strip()
            upload_route_to_frontend(route_name, gps_waypoints)
        else:
            print("[路线] 未采集到任何经纬度航点，跳过上传")

        # 将航点保存到本地 JSON（与 load_waypoints_from_file 格式一致：[[lon, lat], ...]）
        if gps_waypoints:
            default_path = f"waypoints_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            save_path = input(f"保存航点到文件（回车使用默认 {default_path}）：").strip()
            if not save_path:
                save_path = default_path
            try:
                with open(save_path, "w", encoding="utf-8") as f:
                    json.dump(gps_waypoints, f, ensure_ascii=False, indent=2)
                print(f"[航点] 已保存 {len(gps_waypoints)} 个航点到 {save_path}")
            except Exception as exc:
                print(f"[航点] 保存失败 {save_path}: {exc}")

    # ==========================
    # 开始导航（状态机驱动）
    # ==========================
    if waypoints:
        # 初始化状态机为巡航模式，从第一个航点开始
        current_wp_index = 0
        return_index = 0
        robot_state = STATE_CRUISE
    else:
        # 无初始航点时保持待机，等待 startTask 指令
        robot_state = STATE_IDLE

    run_navigation_loop(sport_client)
