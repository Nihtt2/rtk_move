#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import math
import signal

import cv2
import numpy as np
import pyzed.sl as sl
from pupil_apriltags import Detector
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib import transforms

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.b2.sport.sport_client import SportClient


# ================= 物理尺寸建模（可根据实测微调） =================
# 狗身长、宽（单位：米），相机在狗的最前端中间
DOG_LENGTH = 1.2
DOG_WIDTH = 0.6

# 充电座实测尺寸（单位：米）
CHARGER_WIDTH = 0.39   # 左右方向宽度
CHARGER_LENGTH = 0.68  # 前后方向长度（深度）

# 充电座被近似为一个长方体，其“几何中心”在内部
# AprilTag 贴在充电座正前方面的正上方，且投影在平面上与前沿中点对齐
# 因此，从 Tag 所在的前沿位置往“里”走 CHARGER_LENGTH/2 即到达几何中心
CHARGER_BACK_OFFSET = CHARGER_LENGTH / 2.0

#修正系数
BASELINE = 0.115

# 用于 Ctrl+C 优雅退出的全局标志
shutdown_requested = False


def _signal_handler(signum, frame):
    global shutdown_requested
    shutdown_requested = True
    print("\n收到中断信号，正在退出... (请稍候)")


def init_zed_camera():
    """
    初始化 ZED 相机，仅用于取图做 AprilTag 识别。
    与 apriltag.py 保持相同参数，保证位姿解算一致。
    """
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.camera_fps = 60
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print("无法打开 ZED 相机:", status)
        sys.exit(1)

    return zed


def init_apriltag_detector(zed: sl.Camera):
    """
    使用 ZED 相机当前标定参数自动读取内参（fx, fy, cx, cy），
    兼容 ZED SDK 不同版本的 API。
    """
    cam_info = zed.get_camera_information()

    # ZED SDK 3.x: cam_info.calibration_parameters.left_cam
    # ZED SDK 4.x: cam_info.camera_configuration.calibration_parameters.left_cam
    if hasattr(cam_info, "calibration_parameters"):
        calib = cam_info.calibration_parameters.left_cam
    else:
        calib = cam_info.camera_configuration.calibration_parameters.left_cam

    fx = calib.fx
    fy = calib.fy
    cx = calib.cx
    cy = calib.cy
    camera_params = [fx, fy, cx, cy]

    # apriltag 实际边长（米），与原脚本保持一致
    tag_size = 0.181

    detector = Detector(
        families="tagStandard41h12",
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )

    print(f"使用 ZED 内参: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")

    return detector, camera_params, tag_size


def init_sport_client(network_interface: str) -> SportClient:
    """
    初始化 Unitree 运动客户端。
    network_interface 例如 'eno1'。
    """
    ChannelFactoryInitialize(0, network_interface)

    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    return sport_client


def select_best_detection(detections):
    """
    当前假设只有一个 AprilTag。
    若存在多个，则选择距离相机最近的那个（tz 最小）。
    """
    if not detections:
        return None

    # tz 为相机坐标系下沿前向的距离，越小越近
    # detection.pose_t 的每一项是 0 维 ndarray，需要用 .item() 取标量
    return min(
        detections,
        key=lambda d: d.pose_t[2].item() if d.pose_t is not None else 1e9,
    )


def follow_apriltag(zed: sl.Camera, sport_client: SportClient, detector: Detector, camera_params, tag_size: float) -> bool:
    """
    让机械狗自动靠近并正对 AprilTag：
      - 向右为 X+，向前为 Z+，向上为 Y+
      - 根据 tx、tz 做简单的 P 控制前进与转向
      - 最终距离接近 target_dist 且 yaw 误差很小则停止
    返回 True 表示到达并成功充电位，False 表示用户中断或异常退出。
    """
    image = sl.Mat()
    charging_success = False  # 是否因到达充电位而退出

# ================= Matplotlib 可视化初始化（Tag 固定，狗相对运动） =================
    plt.ion()
    fig, ax = plt.subplots()
    plt.subplots_adjust(top=0.88)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X (m, 右为+)")
    ax.set_ylabel("Z (m, 前为+)")
    ax.set_title("Dog pose relative to AprilTag (Top View)")

    # 让 Tag 固定在原点 (0,0)
    tag_x, tag_z = 0.0, 0.0

    # 只显示 Tag 中心（绿色 +）
    tag_scatter = ax.scatter(
        [tag_x], [tag_z],
        c="green",
        s=80,
        marker="+",
        label="Tag (fixed)"
    )

    # ================= 狗（当前） =================
    dog_rect = Rectangle(
        (-DOG_WIDTH / 2.0, -DOG_LENGTH / 2.0),
        DOG_WIDTH,
        DOG_LENGTH,
        edgecolor="blue",
        facecolor="none",
        linewidth=2,
        label="Dog (current)",
    )
    ax.add_patch(dog_rect)

    dog_center_scatter = ax.scatter([], [], c="blue", s=40, marker="o")

    # ================= 预测狗（虚线） =================
    dog_pred_rect = Rectangle(
        (-DOG_WIDTH / 2.0, -DOG_LENGTH / 2.0),
        DOG_WIDTH,
        DOG_LENGTH,
        edgecolor="blue",
        facecolor="none",
        linewidth=2,
        linestyle="--",
        label="Dog (pred aligned)",
    )
    ax.add_patch(dog_pred_rect)

    dog_pred_center_scatter = ax.scatter([], [], c="blue", s=40, marker="x")

    # ================= 文本显示 =================
    info_text = fig.text(
        0.02, 0.97, "",   # 这是相对于整个窗口
        ha="left",
        va="top",
        fontsize=6
    )

    ax.legend(loc="upper right")

    # 初始显示范围
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-2.5, 1.5)

    # ================= 控制参数（可按现场效果微调） =================
    # 目标：把“狗的几何中心”与“充电座几何中心”重合
    # 已知：
    #   - 相机在狗最前端中间
    #   - 狗的几何中心在相机后方 DOG_LENGTH / 2 处
    #   - AprilTag 在充电座前沿正上方，充电座几何中心在 Tag 后方 CHARGER_BACK_OFFSET 处
    #
    # 在完全对正（X 对齐、朝向对齐）时，沿相机 Z 轴方向有：
    #   camera --(tz)--> Tag --(向后 CHARGER_BACK_OFFSET)--> 充电座中心
    #
    # 期望：camera 到充电座中心的距离 = camera 到狗中心的距离 = DOG_LENGTH / 2
    #   所以： tz - CHARGER_BACK_OFFSET = DOG_LENGTH / 2
    #   推导： tz_target = DOG_LENGTH / 2 + CHARGER_BACK_OFFSET 
    target_dist = DOG_LENGTH / 2.0 + CHARGER_BACK_OFFSET  # 期望与 AprilTag 的前向距离（米） 可能要改
    dist_tol = 0.08          # 距离误差容忍（米）
    yaw_tol_deg = 5.0        # 最终对准时的角度误差容忍（度）

    max_vx = 0.5             # 最大前进速度（m/s）
    kp_v = 0.6               # 前进速度比例系数

    max_yaw_rate = 0.6       # 最大角速度（rad/s）
    kp_yaw = 1.0             # 角速度比例系数

    enable_strafe = True     # 是否开启横移（vy）用于消除 tx（更利于“中心对齐”）
    max_vy = 0.35            # 最大横移速度（m/s）
    kp_vy = 0.8              # 横移比例系数（基于 tx）
    x_tol = 0.02             # 横向对齐容忍（m）

    inc_tol_deg = 14.0        # “相机->Tag连线”与“Tag平面法向”的夹角容忍（度）

    search_yaw = 0.3         # 丢失目标时的原地自转角速度（rad/s）
    last_seen_time = 0.0

    print("开始 AprilTag 跟随控制，按 q 退出。按 Ctrl+C 亦可退出。")

    try:
        while True:
            if shutdown_requested:
                print("退出标志已置位，停止跟随。")
                break
            if zed.grab() != sl.ERROR_CODE.SUCCESS:
                continue

            zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            detections = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=camera_params,
                tag_size=tag_size,
            )

            detection = select_best_detection(detections) #如果要多个tag，这里改成循环

            vx_cmd = 0.0
            vy_cmd = 0.0
            vyaw_cmd = 0.0

            if detection is None or detection.pose_t is None:
                # 没看到 AprilTag：轻微自转搜索
                if time.time() - last_seen_time > 0.3:
                    vx_cmd = 0.0
                    vyaw_cmd = search_yaw
                cv2.putText(
                    frame,
                    "No tag detected",
                    (40, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2,
                )
            else:
                last_seen_time = time.time()

                # 位置（相机坐标系：右为 X+，前为 Z+）
                pose_t = detection.pose_t
                # 与 apriltag.py 中一致的取值方式：每个元素是 0 维 ndarray
                tx = pose_t[0].item()
                #修正 
                tx = tx - BASELINE/2

                ty = pose_t[1].item()
                tz = pose_t[2].item()

                # ========== 用 pose_R 计算“正对”误差（Tag平面法向相关）==========
                # 相机坐标：X 右、Y 上、Z 前
                # 目标：让 Tag 平面法向在相机坐标系中尽可能指向 +Z（正对）
                pose_R = detection.pose_R
                normal_yaw_error = None
                normal_yaw_error_deg = None
                inc_angle_deg = None

                if pose_R is not None:
                    R = np.array(pose_R, dtype=float)
                    if R.shape == (3, 3):
                        # 常见约定：R 的列向量是 Tag 坐标轴在相机坐标系下的方向
                        n = R[:, 2].copy()  # Tag 法向在相机坐标系
                        # 统一法向指向相机前方（z>0），避免正反不确定导致跳变
                        if n[2] < 0:
                            n = -n

                        n_unit = n / (float(np.linalg.norm(n)) + 1e-9)  #n_unit 是 AprilTag 平面的单位法向量，表示 Tag 平面“正面朝向哪里”

                        # 法向相对相机前向的“偏航”误差：0 表示正对
                        normal_yaw_error = math.atan2(float(n_unit[0]), float(n_unit[2]))
                        normal_yaw_error_deg = math.degrees(normal_yaw_error)  #偏转角度

                        # “相机->Tag连线” 与 “Tag法向” 的夹角：0 表示在法向线上（入射角为0）
                        d = np.array([tx, ty, tz], dtype=float)
                        d_unit = d / (float(np.linalg.norm(d)) + 1e-9)
                        dot_nd = float(np.clip(np.dot(n_unit, d_unit), -1.0, 1.0))
                        inc_angle_deg = math.degrees(math.acos(dot_nd))

                center = detection.center

                # 在图像上画框和 ID
                pt1, pt2, pt3, pt4 = detection.corners
                pts = np.array([pt1, pt2, pt3, pt4], dtype=np.int32).reshape((-1, 1, 2))
                frame = cv2.polylines(frame, [pts], isClosed=True, color=(0, 165, 255), thickness=4)

                cv2.putText(
                    frame,
                    f"ID: {detection.tag_id}",
                    (int(center[0]), int(center[1])),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2,
                )

                cv2.putText(
                    frame,
                    f"Pos[m]: X={tx:.2f} Y={ty:.2f} Z={tz:.2f}",
                    (int(center[0]), int(center[1]) + 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 0),
                    2,
                )

                # 仅用 tx、tz 计算水平偏角：tag 在相机右侧时 yaw_error > 0
                yaw_error = math.atan2(tx, tz)
                yaw_error_deg = math.degrees(yaw_error)

                # 前向距离误差：希望 tz → target_dist
                dist_error = tz - target_dist

                # 到达判定：距离够近且角度已对正
                ok_x = abs(tx) < x_tol
                ok_yaw_tx = abs(yaw_error_deg) < yaw_tol_deg
                ok_yaw_normal = (normal_yaw_error_deg is None) or (abs(normal_yaw_error_deg) < yaw_tol_deg)
                ok_inc = (inc_angle_deg is None) or (inc_angle_deg < inc_tol_deg)
                arrived = (abs(dist_error) < dist_tol) and ok_x and ok_yaw_tx and ok_yaw_normal and ok_inc

                if arrived:
                    vx_cmd = 0.0
                    vyaw_cmd = 0.0
                    time.sleep(2)
                    sport_client.StandDown()
                    cv2.putText(
                        frame,
                        "Arrived",
                        (int(center[0]), int(center[1]) - 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 255, 255),
                        2,
                    )
                    charging_success = True
                    break  # 到达充电位，退出循环并结束程序
                else:
                    # 转向控制：优先用“法向偏航误差”让视线垂直 Tag 平面
                    # SportClient.Move 中正角速度为左转，因此想向右转时取负号。
                    yaw_for_control = normal_yaw_error if normal_yaw_error is not None else yaw_error
                    vyaw_cmd = -kp_yaw * yaw_for_control
                    vyaw_cmd = max(min(vyaw_cmd, max_yaw_rate), -max_yaw_rate)

                    # 横移控制：用 vy 消除 tx（中心对齐更稳定）
                    if enable_strafe:
                        # tx>0 表示 Tag 在右侧；vy>0 通常是向左，因此取负号让其向右横移
                        vy_cmd = -kp_vy * tx
                        vy_cmd = max(min(vy_cmd, max_vy), -max_vy)
                        if abs(vy_cmd) < 0.1:
                            vy_cmd = math.copysign(0.1, vy_cmd)

                    # 距离控制：只在距离大于目标距离时前进
                    if dist_error > 0:
                        vx_cmd = kp_v * dist_error
                        vx_cmd = min(vx_cmd, max_vx)
                        if abs(vx_cmd) <0.1:
                            vx_cmd = math.copysign(0.1, vx_cmd)

                        # 若偏角太大，先以转向为主，减小前进速度
                        heading_factor = max(0.0, math.cos(yaw_for_control))
                        vx_cmd *= heading_factor
                    else:
                        vx_cmd = 0.0
                        

                cv2.putText(
                    frame,
                    f"tx={tx:.2f} tz={tz:.2f} | yaw(tx)={yaw_error_deg:.1f}deg | "
                    f"yaw(n)={(normal_yaw_error_deg if normal_yaw_error_deg is not None else float('nan')):.1f}deg | "
                    f"inc={(inc_angle_deg if inc_angle_deg is not None else float('nan')):.1f}deg",
                    (40, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                )

                cv2.putText(
                    frame,
                    f"cmd vx={vx_cmd:.2f} vy={vy_cmd:.2f} vyaw={vyaw_cmd:.2f}",
                    (40, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                )

                # ========== 更新 Matplotlib 俯视图 ==========
                if detection is not None and detection.pose_t is not None:
                    # ---- 1) 取原始测量（建议入射角等仍用 raw）----
                    t = np.array([tx, ty, tz], dtype=float).reshape(3, 1)  # Tag 在 camera 中的位置（Tag->Cam）

                    # ---- 2) 用 R 求相机在 Tag 坐标系的位置：p_tag_cam = -R^T * t ----
                    cam_x = -tx  # fallback：如果 R 不可用，先用近似
                    cam_z = -tz
                    dog_yaw = 0.0
                    dog_yaw_deg = 0.0

                    if pose_R is not None:
                        R_ct = np.array(pose_R, dtype=float)  # Tag->Cam
                        if R_ct.shape == (3, 3):
                            R_tc = R_ct.T  # Cam->Tag

                            p_tag_cam = (-R_tc @ t).flatten()  # 相机原点在 Tag 坐标系
                            cam_x = float(p_tag_cam[0])
                            cam_z = float(p_tag_cam[2])

                            # ---- 3) 用“相机前向轴”求狗真实 yaw（姿态）----
                            # 相机前向轴（相机系）是 +Z = [0,0,1]
                            cam_forward_in_tag = (R_tc @ np.array([[0.0], [0.0], [1.0]])).flatten()
                            fx = float(cam_forward_in_tag[0])
                            fz = float(cam_forward_in_tag[2])

                            dog_yaw = -math.atan2(fx, fz)        # Tag 坐标系下的 yaw
                            dog_yaw_deg = math.degrees(dog_yaw)

                    # ---- 4) 狗前向单位向量（Tag 的 X-Z 平面）----
                    fwd_x = math.sin(dog_yaw)
                    fwd_z = math.cos(dog_yaw)

                    # ---- 5) 狗中心：相机在狗头，狗中心在后方 DOG_LENGTH/2 ----
                    dog_center_x = cam_x - fwd_x * (DOG_LENGTH / 2.0)
                    dog_center_z = cam_z - fwd_z * (DOG_LENGTH / 2.0)

                    # ---- 6) 预测“回正对齐后”的位置（虚线）----
                    # 这里保持你原来的预测定义：回正到 yaw=0，并且距离对齐 target_dist
                    cam_pred_x = 0.0
                    cam_pred_z = -target_dist
                    dog_pred_yaw = 0.0
                    dog_pred_center_x = cam_pred_x
                    dog_pred_center_z = cam_pred_z - (DOG_LENGTH / 2.0)

                    # ---- 7) 更新当前狗矩形 + 旋转 ----
                    dog_rect.set_xy((dog_center_x - DOG_WIDTH/2.0, dog_center_z - DOG_LENGTH/2.0))
                    dog_rect.set_transform(
                        transforms.Affine2D().rotate_around(dog_center_x, dog_center_z, dog_yaw) + ax.transData
                    )
                    dog_center_scatter.set_offsets(np.array([[dog_center_x, dog_center_z]]))

                    # ---- 8) 更新预测狗（虚线） ----
                    dog_pred_rect.set_xy((dog_pred_center_x - DOG_WIDTH/2.0, dog_pred_center_z - DOG_LENGTH/2.0))
                    dog_pred_rect.set_transform(
                        transforms.Affine2D().rotate_around(dog_pred_center_x, dog_pred_center_z, dog_pred_yaw) + ax.transData
                    )
                    dog_pred_center_scatter.set_offsets(np.array([[dog_pred_center_x, dog_pred_center_z]]))

                    # ---- 9) 文本：持续显示中心坐标 + 姿态 ----

                    info_text.set_text(
                        "Tag center: (0.00, 0.00)\n"
                        f"Cam in Tag: ({cam_x:.2f}, {cam_z:.2f})\n"
                        f"Dog center: ({dog_center_x:.2f}, {dog_center_z:.2f}) | dog_yaw={dog_yaw_deg:.1f}deg\n"
                        f"Pred dog center: ({dog_pred_center_x:.2f}, {dog_pred_center_z:.2f}) | pred_yaw=0.0deg"
                    )

                    # ---- 10) 自动缩放 ----
                    xs = [0.0, dog_center_x, dog_pred_center_x]
                    zs = [0.0, dog_center_z, dog_pred_center_z]
                    pad = 0.8
                    ax.set_xlim(min(xs) - pad, max(xs) + pad)
                    ax.set_ylim(min(zs) - pad, max(zs) + pad)

                fig.canvas.draw_idle()
                plt.pause(0.001)

            # 发送运动指令
            sport_client.Move(vx_cmd, vy_cmd, vyaw_cmd)

            cv2.imshow("ZED AprilTag Follow", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("收到 q，停止跟随。")
                break

    finally:
        sport_client.StopMove()
        cv2.destroyAllWindows()
        plt.close(fig)
        plt.ioff()

    return charging_success


def main():
    global shutdown_requested
    if len(sys.argv) < 2:
        print(f"用法: python3 {sys.argv[0]} <networkInterface>")
        print("例如: python3 rtk_apriltag_follow.py eno1")
        sys.exit(1)

    # 注册 Ctrl+C 处理，便于优雅退出
    signal.signal(signal.SIGINT, _signal_handler)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _signal_handler)

    network_interface = sys.argv[1]

    # 初始化硬件与检测器
    sport_client = init_sport_client(network_interface)
    zed = init_zed_camera()
    detector, camera_params, tag_size = init_apriltag_detector(zed)
    # sport_client.ClassicWalk(true)

    try:
        success = follow_apriltag(zed, sport_client, detector, camera_params, tag_size)
        if success:
            print("成功充电。")
            sys.exit(0)
    except KeyboardInterrupt:
        print("\n键盘中断，退出。")
        shutdown_requested = True
    finally:
        zed.close()
        print("已关闭 ZED 相机，程序结束。")
    sys.exit(1)  # 非“到达充电位”退出


if __name__ == "__main__":
    main()


