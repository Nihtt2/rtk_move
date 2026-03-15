#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import serial
import datetime
import threading
import math
from pyproj import Transformer
from flask import Flask, jsonify
from flask_cors import CORS

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.b2.sport.sport_client import SportClient

# ================= 串口 =================
serport = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
serport.flushInput()
time.sleep(0.5)

# ================= 全局变量 =================
last_utm_x = None
last_utm_y = None
origin_utm_x = None
origin_utm_y = None
filtered_heading = None
gga_quality = 0
latest_fields = None
lon = None
lat = None

# ================= 控制参数 =================
alpha = 0.2
kp_yaw = 0.02
kp_dist = 0.3
max_yaw_rate = 0.4
max_vx = 1
arrival_radius = 0.2


# ================= 工具函数 =================
def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def get_utm_epsg(lon, lat):
    zone = int((lon + 180) / 6) + 1
    if lat >= 0:
        epsg = 32600 + zone
    else:
        epsg = 32700 + zone
    return epsg


# ================= AGRICA 解析 =================
def parse_agrica(line):
    try:
        if "#AGRICA" not in line:
            return None

        data_part = line.split(";")[1]
        data_part = data_part.split("*")[0]
        fields = data_part.split(",")

        return fields
    except:
        return None


# ================= Flask后端服务 =================
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
        return jsonify({
            'error': 'RTK data not available'
        }), 503

def start_flask_server():
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


# ================= RTK线程 =================
def task2():
    global last_utm_x, last_utm_y
    global origin_utm_x, origin_utm_y
    global filtered_heading, gga_quality
    global latest_fields
    global lon, lat

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

        if  heading_status in [4, 5]:
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
            print("地图原点已设定")



# ================= 自动导航（改进版） =================
def navigate_to(target_x, target_y, sport_client):
    """
    三阶段锁头 + 渐进加速 + 动态阈值导航控制

    控制结构：
    1. 粗锁阶段：快速大角度修正
    2. 细锁阶段：减速精细修正
    3. 吸附阶段：指数衰减贴合
    4. 起步渐进加速
    5. 动态误差阈值
    """

    print("\n========== 开始导航 ==========\n")

    # ============================
    # 可调参数
    # ============================

    yaw_gain_coarse = 0.08     # 粗锁比例增益
    yaw_gain_fine = 0.05       # 细锁比例增益
    yaw_gain_lock = 0.03       # 吸附阶段增益

    brake_strength = 3.5
    yaw_acc_limit = 0.05

    coarse_threshold = 40.0
    fine_threshold = 12.0

    heading_scale = 35.0

    accel_limit = 0.02         # 前进加速度限制（渐进起步）
    min_vx = 0.3

    last_vyaw = 0.0
    vx_current = 0.0
    print_timer = time.time()

    while True:

        if last_utm_x is None or filtered_heading is None:
            continue

        # ============================
        # 当前坐标
        # ============================

        cur_x = last_utm_x - origin_utm_x
        cur_y = last_utm_y - origin_utm_y

        dx = target_x - cur_x
        dy = target_y - cur_y
        dist = math.hypot(dx, dy)

        if dist < arrival_radius:
            sport_client.StopMove()
            print("\n已到达目标点\n")
            break

        # ============================
        # 计算目标航向
        # ============================

        angle = math.degrees(math.atan2(dy, dx))
        target_heading = 90 - angle
        if target_heading < 0:
            target_heading += 360
        target_heading = (target_heading + 180) % 360

        error = normalize_angle(filtered_heading - target_heading)

        abs_error = abs(error)

        # =====================================================
        # 三阶段锁头控制
        # =====================================================

        if abs_error > coarse_threshold:
            # -------- 粗锁阶段 --------
            yaw_gain = yaw_gain_coarse
            stage = "粗锁"
            allow_move = False

        elif abs_error > fine_threshold:
            # -------- 细锁阶段 --------
            yaw_gain = yaw_gain_fine
            stage = "细锁"
            allow_move = False

        else:
            # -------- 吸附阶段 --------
            yaw_gain = yaw_gain_lock
            stage = "吸附"
            allow_move = True

        # 基础比例
        vyaw_target = yaw_gain * error

        # tanh 限幅（连续函数）
        vyaw_target = max_yaw_rate * math.tanh(vyaw_target / max_yaw_rate)

        # 动态刹车距离
        braking_angle = (last_vyaw ** 2) / (2 * brake_strength + 1e-6)
        brake_factor = math.exp(-abs_error / (braking_angle + 0.5))
        vyaw_target *= (1 - brake_factor)

        # 角加速度限制（防过冲）
        delta = vyaw_target - last_vyaw
        delta = max(min(delta, yaw_acc_limit), -yaw_acc_limit)

        vyaw = last_vyaw + delta
        last_vyaw = vyaw

        # =====================================================
        # 动态误差阈值（速度越快要求越严格）
        # =====================================================

        dynamic_threshold = fine_threshold - vx_current * 6.0
        dynamic_threshold = max(5.0, dynamic_threshold)

        if abs_error > dynamic_threshold:
            allow_move = False

        # =====================================================
        # 前进速度控制（渐进加速）
        # =====================================================

        if allow_move:

            heading_factor = 1.0 / (1.0 + (error / heading_scale) ** 2)

            vx_target = min(kp_dist * dist, max_vx) * heading_factor

            if dist > arrival_radius:
                vx_target = max(vx_target, min_vx)

        else:
            vx_target = 0.0

        # 渐进加速模型（避免突跳）
        if vx_target > vx_current:
            vx_current += accel_limit
            vx_current = min(vx_current, vx_target)
        else:
            vx_current = vx_target

        # =====================================================
        # 发送控制
        # =====================================================

        sport_client.Move(vx_current, 0, vyaw)

        # =====================================================
        # 状态输出
        # =====================================================

        if time.time() - print_timer > 0.1:
            print(
                f"\r阶段: {stage} | "
                f"距离: {dist:6.2f} m | "
                f"误差: {error:7.2f}° | "
                f"角速: {vyaw:6.3f} | "
                f"RTK:{filtered_heading:7.2f}° | "
                f"目标:{target_heading:7.2f}° | "
                f"前速: {vx_current:6.3f}",
                end=""
            )
            print_timer = time.time()

        time.sleep(0.05)





# ================= 主程序 =================
if __name__ == "__main__":

    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)

    ChannelFactoryInitialize(0, sys.argv[1])

    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    threading.Thread(target=task2, daemon=True).start()
    threading.Thread(target=start_flask_server, daemon=True).start()

    print("RTK后端服务已启动，API地址: http://0.0.0.0:5000/api/rtk_location")


# ==========================
# 多航点采集
# ==========================

    waypoints = []

    print("\n========== 航点采集模式 ==========")
    print("移动机器人到目标点")
    print("按 Enter 记录航点")
    print("输入 q + Enter 结束采集\n")

    while True:

        cmd = input("记录航点 or q退出：")

        if cmd.lower() == 'q':
            break

        if last_utm_x is None:
            print("RTK 未就绪")
            continue

        x = last_utm_x - origin_utm_x
        y = last_utm_y - origin_utm_y

        waypoints.append((x, y))

        print(f"航点 {len(waypoints)} 已记录: {x:.2f}, {y:.2f}")


# ==========================
# 开始导航
# ==========================

    print("\n========== 开始多航点导航 ==========")

    for i, (x, y) in enumerate(waypoints):

        print(f"\n>>> 前往航点 {i+1}/{len(waypoints)}")

        navigate_to(x, y, sport_client)

        time.sleep(3)


# ==========================
# 自动返航
# ==========================

    print("\n>>> 返航")

    navigate_to(0, 0, sport_client)

    print("\n任务完成")
