#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
combined_navigation.py

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
import serial
import threading
import math
import pyzed.sl as sl
import matplotlib.pyplot as plt
from pyproj import Transformer
from flask import Flask, jsonify
from flask_cors import CORS
import json
import os
import urllib.request
import urllib.error

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.b2.sport.sport_client import SportClient

# ================= ZED 坐标约定 =================
# RIGHT_HANDED_Z_UP: 正 yaw = 逆时针, 与罗盘顺时针相反, 故符号取 -1
# 如果实测校准后转向方向相反, 将此值改为 1
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
latest_fields = None
lon = None
lat = None
waypoints = []
gps_waypoints = []  # 手动记录航点时，同时保存对应的经纬度用于上传给前端

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

# ================= 控制参数 =================
alpha = 0.2
kp_dist = 0.9
max_yaw_rate = 0.4
max_vx = 1.0
arrival_radius = 0.2
vx_current = 0.0

# ================= 导航暂停控制 =================
# 与 WebSocket 客户端通过文件通信，实现“狗停下但后台线程不停止”
PAUSE_STATE_FILE = "/tmp/robotdog_nav_pause_state.json"

# ================= 路线上传接口配置 =================
CONFIG_PATH = "/home/nvidia/Desktop/config.json"


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


# ================= RTK 线程 =================
def task_rtk():
    global last_utm_x, last_utm_y, origin_utm_x, origin_utm_y
    global filtered_heading, gga_quality, latest_fields, lon, lat
    global stable_rtk_x, stable_rtk_y
    global fused_utm_x, fused_utm_y
    global zed_ref_tx, zed_ref_ty

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

                    dx_zed = tx - zed_ref_tx
                    dy_zed = ty - zed_ref_ty

                    # 旋转到UTM坐标系
                    # ZED RIGHT_HANDED_Z_UP: X=前向, Y=左方
                    # 启动时相机朝向 H₀=zed_calibration_offset（真北顺时针）
                    # ZED X轴 → 方位H₀方向; ZED Y轴 → H₀左偏90°方向
                    # dE = dx_zed*sin(H₀) - dy_zed*cos(H₀)
                    # dN = dx_zed*cos(H₀) + dy_zed*sin(H₀)
                    theta = math.radians(zed_calibration_offset - 90)

                    dx_utm = (
                        dx_zed * math.sin(theta)
                        - dy_zed * math.cos(theta)
                    )

                    dy_utm = (
                        dx_zed * math.cos(theta)
                        + dy_zed * math.sin(theta)
                    )

                    fused_utm_x = stable_rtk_x + dx_utm
                    fused_utm_y = stable_rtk_y + dy_utm

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


def upload_route_to_frontend(route_name, gps_coords):
    """
    调用 Android 端 /api/robot-route/upload 接口，上报机器狗当前记录的路线。

    gps_coords: [[lat, lon], [lat, lon], ...]
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
        "coordinates": [[float(lat), float(lon)] for lat, lon in gps_coords],
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
    print_timer = time.time()

    paused_last = False  # 上一次循环是否处于暂停状态

    while True:
        # ========== 暂停控制 ==========
        if is_navigation_paused():
            if not paused_last:
                # 首次进入暂停：立即停下
                sport_client.StopMove()
                print("\n[导航] 收到暂停指令，狗已停下，但 RTK/ZED 仍在运行")
                last_vyaw = 0.0
                vx_current = 0.0
                paused_last = True
            time.sleep(0.05)
            continue
        else:
            if paused_last:
                print("\n[导航] 收到继续指令，恢复自动导航")
                paused_last = False

        current_heading = get_current_heading()

        if last_utm_x is None or origin_utm_x is None or current_heading is None:
            time.sleep(0.05)
            continue

        cur_x = fused_utm_x - origin_utm_x
        cur_y = fused_utm_y - origin_utm_y

        dx = target_x - cur_x
        dy = target_y - cur_y
        dist = math.hypot(dx, dy)

        if dist < arrival_radius:
            sport_client.StopMove()
            print("\n已到达目标点\n")
            break

        # 目标航向（真北顺时针，0-360°）
        angle = math.degrees(math.atan2(dy, dx))
        target_heading = (90 - angle + 180) % 360

        error = normalize_angle(current_heading - target_heading)
        abs_error = abs(error)

        # ---- 三阶段锁头 ----
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

        sport_client.Move(vx_current, 0, vyaw)

        # ---- 状态输出 ----
        if time.time() - print_timer > 0.1:
            src = "ZED" if (zed_calibrated and zed_calibrated_heading is not None) else "RTK"
            print(
                f"\r阶段:{stage} | "
                f"距离:{dist:6.2f}m | "
                f"误差:{error:7.2f}° | "
                f"角速:{vyaw:6.3f} | "
                f"航向[{src}]:{current_heading:7.2f}° | "
                f"目标:{target_heading:7.2f}° | "
                f"前速:{vx_current:6.3f}",
                end=""
            )
            print_timer = time.time()

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
        [lat, lon],
        [lat, lon],
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
            # data 里是 [lat, lon]
            lat_val = float(item[0])
            lon_val = float(item[1])
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

 
# ================= 主程序 =================
if __name__ == "__main__":

    # 支持两种用法：
    #   1) python zed_rtk_move.py eno1
    #        -> 手动采集航点（你原来的模式）
    #   2) python zed_rtk_move.py eno1 /tmp/robotdog_turning_waypoints.json
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

    # 启动各后台线程
    threading.Thread(target=task_rtk,          daemon=True).start()
    threading.Thread(target=task_zed,          daemon=True).start()
    threading.Thread(target=start_flask_server, daemon=True).start()
    #threading.Thread(target=task_plot,         daemon=True).start()

    print("RTK 后端服务已启动，API: http://0.0.0.0:5000/api/rtk_location")

    # 首次启动：用 RTK 真北校准 ZED 航向偏移（阻塞直到校准完成）
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

            # 记录对应的经纬度，用于之后上传给前端
            if lat is not None and lon is not None:
                gps_waypoints.append([lat, lon])

        # 手动采集模式下，采集结束后将路线上传给前端
        if gps_waypoints:
            print("\n========== 路线上传 ==========")
            route_name = input("请输入本次路线名称（不超过15个字符，回车默认“自定义路线”）：").strip()
            upload_route_to_frontend(route_name, gps_waypoints)
        else:
            print("[路线] 未采集到任何经纬度航点，跳过上传")

    # ==========================
    # 开始导航
    # ==========================
    print("\n========== 开始多航点导航 ==========")

    for i, (x, y) in enumerate(waypoints):
        print(f"\n>>> 前往航点 {i + 1}/{len(waypoints)}  目标:({x:.2f}, {y:.2f})")
        navigate_to(x, y, sport_client)
        time.sleep(0.1)

    # ==========================
    # 自动返航
    # ==========================
    print("\n>>> 返航原点")
    navigate_to(0, 0, sport_client)

    print("\n任务完成")