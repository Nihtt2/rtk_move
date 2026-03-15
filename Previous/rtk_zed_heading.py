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

# ================= 全局变量 - ZED =================
zed_yaw_raw = None              # ZED 原始 yaw（度，RIGHT_HANDED_Z_UP）
zed_calibrated_heading = None   # 校准后的 ZED 航向（真北顺时针 0-360°）
zed_calibration_offset = None   # 校准偏移量：offset = rtk_ref - ZED_YAW_SIGN * zed_ref
zed_calibrated = False          # 是否已完成 RTK->ZED 校准

# ================= 控制参数 =================
alpha = 0.2
kp_dist = 0.3
max_yaw_rate = 0.4
max_vx = 1.0
arrival_radius = 0.2


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
            'timestamp': time.time()
        })
    else:
        return jsonify({'error': 'RTK data not available'}), 503


def start_flask_server():
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


# ================= RTK 线程 =================
def task_rtk():
    global last_utm_x, last_utm_y, origin_utm_x, origin_utm_y
    global filtered_heading, gga_quality, latest_fields, lon, lat

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

        if origin_utm_x is None and rtk_status == 4:
            origin_utm_x = utm_x
            origin_utm_y = utm_y
            print("\n[RTK] 地图原点已设定")


# ================= ZED 线程 =================
def task_zed():
    global zed_yaw_raw, zed_calibrated_heading

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


# ================= 自动导航 =================
def navigate_to(target_x, target_y, sport_client):
    """
    三阶段锁头 + 渐进加速 + 动态阈值导航控制。
    航向来源：ZED 校准航向（主）/ RTK 航向（备用）
    """
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
    vx_current  = 0.0
    print_timer = time.time()

    while True:
        current_heading = get_current_heading()

        if last_utm_x is None or origin_utm_x is None or current_heading is None:
            time.sleep(0.05)
            continue

        cur_x = last_utm_x - origin_utm_x
        cur_y = last_utm_y - origin_utm_y

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
        if 'waypoints' in globals() and len(waypoints) > 0:

            xs = [p[0] for p in waypoints]
            ys = [p[1] for p in waypoints]

            ax.scatter(
                xs, ys,
                c='blue',
                s=60,
                label='Waypoints'
            )

        # 当前点
        if last_utm_x is not None and origin_utm_x is not None:

            cur_x = last_utm_x - origin_utm_x
            cur_y = last_utm_y - origin_utm_y

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
# ================= 主程序 =================
if __name__ == "__main__":

    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)

    ChannelFactoryInitialize(0, sys.argv[1])

    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    # 启动各后台线程
    threading.Thread(target=task_rtk,          daemon=True).start()
    threading.Thread(target=task_zed,          daemon=True).start()
    threading.Thread(target=start_flask_server, daemon=True).start()
    threading.Thread(target=task_plot, daemon=True).start()

    print("RTK 后端服务已启动，API: http://0.0.0.0:5000/api/rtk_location")

    # 首次启动：用 RTK 真北校准 ZED 航向偏移（阻塞直到校准完成）
    calibrate_zed_with_rtk()

    # ==========================
    # 多航点采集
    # ==========================
    waypoints = []

    print("========== 航点采集模式 ==========")
    print("移动机器人到目标点，按 Enter 记录，输入 q 结束采集\n")

    while True:
        cmd = input("记录航点 or q退出：")
        if cmd.lower() == 'q':
            break

        if last_utm_x is None or origin_utm_x is None:
            print("[警告] RTK 未就绪，请稍候")
            continue

        x = last_utm_x - origin_utm_x
        y = last_utm_y - origin_utm_y
        waypoints.append((x, y))
        print(f"航点 {len(waypoints)} 已记录: ({x:.2f}, {y:.2f})")

    # ==========================
    # 开始导航
    # ==========================
    print("\n========== 开始多航点导航 ==========")

    for i, (x, y) in enumerate(waypoints):
        print(f"\n>>> 前往航点 {i + 1}/{len(waypoints)}  目标:({x:.2f}, {y:.2f})")
        navigate_to(x, y, sport_client)
        time.sleep(3)

    # ==========================
    # 自动返航
    # ==========================
    print("\n>>> 返航原点")
    navigate_to(0, 0, sport_client)

    print("\n任务完成")
