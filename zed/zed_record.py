#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
zed_record.py — ZED 相机视频录制工具

支持两种录制格式：
  1. SVO（ZED 原生格式）：保留深度、IMU 等全部传感器数据，可用 ZED SDK 回放
  2. MP4（普通视频）  ：通过 OpenCV 导出左目彩色画面，兼容性好

用法：
  python3 zed_record.py                        # 交互式选择参数
  python3 zed_record.py -o output.svo          # 指定 SVO 输出文件
  python3 zed_record.py -o output.mp4          # 指定 MP4 输出文件
  python3 zed_record.py -o out.svo --no-preview  # 无预览（无头环境）

录制期间：
  - 按 q 或 Ctrl+C 停止录制
  - 预览窗口实时显示帧率、录制时长和帧数
"""

import time
import sys
import argparse
import datetime
import signal
import cv2
import numpy as np
import pyzed.sl as sl


# ================= 信号处理（Ctrl+C 优雅退出）=================
_stop_flag = False

def _signal_handler(sig, frame):
    global _stop_flag
    _stop_flag = True

signal.signal(signal.SIGINT, _signal_handler)


# ================= 分辨率映射 =================
RESOLUTION_MAP = {
    "auto": sl.RESOLUTION.AUTO,
    "2k":   sl.RESOLUTION.HD2K,
    "1080": sl.RESOLUTION.HD1080,
    "720":  sl.RESOLUTION.HD720,
    "vga":  sl.RESOLUTION.VGA,
}

# SVO 压缩格式映射
COMPRESSION_MAP = {
    "lossless": sl.SVO_COMPRESSION_MODE.LOSSLESS,
    "h264":     sl.SVO_COMPRESSION_MODE.H264,
    "h265":     sl.SVO_COMPRESSION_MODE.H265,
}


def parse_args():
    parser = argparse.ArgumentParser(description="ZED 相机视频录制")
    parser.add_argument(
        "-o", "--output", default=None,
        help="输出文件路径（.svo 或 .mp4/.avi），默认按时间戳自动命名"
    )
    parser.add_argument(
        "-r", "--resolution", default="720",
        choices=RESOLUTION_MAP.keys(),
        help="录制分辨率（默认 720p）"
    )
    parser.add_argument(
        "--fps", type=int, default=30,
        help="目标帧率（默认 30fps）"
    )
    parser.add_argument(
        "--compression", default="h264",
        choices=COMPRESSION_MAP.keys(),
        help="SVO 压缩格式（仅 .svo 格式有效，默认 h264）"
    )
    parser.add_argument(
        "--no-preview", action="store_true",
        help="关闭实时预览窗口（无头/SSH 环境使用）"
    )
    return parser.parse_args()


def get_output_path(output_arg):
    """若未指定输出路径，按时间戳自动生成 .svo 文件名"""
    if output_arg:
        return output_arg
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"zed_{ts}.svo"


def is_svo(path):
    return path.lower().endswith(".svo")


def record_svo(zed, output_path, compression_mode, show_preview):
    """录制 SVO 格式（保留全部传感器数据）"""
    rec_params = sl.RecordingParameters()
    rec_params.video_filename = output_path
    rec_params.compression_mode = compression_mode

    status = zed.enable_recording(rec_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[录制] 启动 SVO 录制失败: {status}")
        return False

    print(f"[录制] SVO 录制已开始 → {output_path}")
    print("       按 q（预览窗口聚焦时）或 Ctrl+C 停止\n")

    image = sl.Mat()
    runtime_params = sl.RuntimeParameters()

    frame_count = 0
    start_time  = time.time()

    try:
        while not _stop_flag:
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                frame_count += 1
                elapsed = time.time() - start_time
                fps_real = frame_count / elapsed if elapsed > 0 else 0

                if show_preview:
                    zed.retrieve_image(image, sl.VIEW.LEFT)
                    frame = image.get_data()[:, :, :3]   # BGRA → BGR
                    frame = np.ascontiguousarray(frame[:, :, ::-1])  # RGB → BGR for cv2

                    # 叠加录制信息
                    _draw_overlay(frame, elapsed, frame_count, fps_real, output_path)
                    cv2.imshow("ZED Recording (press q to stop)", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                else:
                    if frame_count % 30 == 0:
                        print(
                            f"\r[录制] {_fmt_time(elapsed)} | "
                            f"帧数: {frame_count} | "
                            f"实时帧率: {fps_real:.1f} fps",
                            end=""
                        )
    finally:
        zed.disable_recording()
        if show_preview:
            cv2.destroyAllWindows()
        elapsed = time.time() - start_time
        print(f"\n\n[录制] 已停止。总时长: {_fmt_time(elapsed)}，总帧数: {frame_count}")
        print(f"[录制] SVO 文件已保存: {output_path}")

    return True


def record_mp4(zed, output_path, fps_target, show_preview):
    """录制普通视频格式（左目彩色画面，通过 OpenCV 编码）"""
    # 获取实际分辨率
    cam_info = zed.get_camera_information()
    width  = cam_info.camera_configuration.resolution.width
    height = cam_info.camera_configuration.resolution.height

    ext = output_path.lower().split(".")[-1]
    fourcc_map = {
        "mp4": cv2.VideoWriter_fourcc(*"mp4v"),
        "avi": cv2.VideoWriter_fourcc(*"XVID"),
        "mkv": cv2.VideoWriter_fourcc(*"X264"),
    }
    fourcc = fourcc_map.get(ext, cv2.VideoWriter_fourcc(*"mp4v"))

    writer = cv2.VideoWriter(output_path, fourcc, fps_target, (width, height))
    if not writer.isOpened():
        print(f"[录制] 无法创建视频文件: {output_path}")
        return False

    print(f"[录制] MP4 录制已开始 → {output_path}  ({width}×{height} @ {fps_target}fps)")
    print("       按 q（预览窗口聚焦时）或 Ctrl+C 停止\n")

    image = sl.Mat()
    runtime_params = sl.RuntimeParameters()

    frame_count = 0
    start_time  = time.time()

    try:
        while not _stop_flag:
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image, sl.VIEW.LEFT)
                frame_rgba = image.get_data()
                frame_bgr  = cv2.cvtColor(frame_rgba, cv2.COLOR_BGRA2BGR)

                writer.write(frame_bgr)
                frame_count += 1
                elapsed  = time.time() - start_time
                fps_real = frame_count / elapsed if elapsed > 0 else 0

                if show_preview:
                    preview = frame_bgr.copy()
                    _draw_overlay(preview, elapsed, frame_count, fps_real, output_path)
                    cv2.imshow("ZED Recording (press q to stop)", preview)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                else:
                    if frame_count % 30 == 0:
                        print(
                            f"\r[录制] {_fmt_time(elapsed)} | "
                            f"帧数: {frame_count} | "
                            f"实时帧率: {fps_real:.1f} fps",
                            end=""
                        )
    finally:
        writer.release()
        if show_preview:
            cv2.destroyAllWindows()
        elapsed = time.time() - start_time
        print(f"\n\n[录制] 已停止。总时长: {_fmt_time(elapsed)}，总帧数: {frame_count}")
        print(f"[录制] 视频文件已保存: {output_path}")

    return True


# ================= 工具函数 =================
def _fmt_time(seconds):
    """将秒数格式化为 HH:MM:SS"""
    h = int(seconds // 3600)
    m = int((seconds % 3600) // 60)
    s = int(seconds % 60)
    return f"{h:02d}:{m:02d}:{s:02d}"


def _draw_overlay(frame, elapsed, frame_count, fps_real, output_path):
    """在帧上叠加录制状态信息"""
    h, w = frame.shape[:2]

    # 红色录制指示点
    cv2.circle(frame, (30, 30), 12, (0, 0, 220), -1)
    cv2.putText(frame, "REC", (50, 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 220), 2)

    # 录制信息文字（右上角）
    lines = [
        f"Time : {_fmt_time(elapsed)}",
        f"Frame: {frame_count}",
        f"FPS  : {fps_real:.1f}",
    ]
    for i, line in enumerate(lines):
        y = 30 + i * 28
        cv2.putText(frame, line, (w - 220, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

    # 底部文件名
    fname = output_path.split("/")[-1].split("\\")[-1]
    cv2.putText(frame, fname, (10, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)


# ================= 主程序 =================
def main():
    args = parse_args()
    output_path = get_output_path(args.output)
    show_preview = not args.no_preview

    print("=" * 55)
    print("  ZED 相机录制工具")
    print("=" * 55)
    print(f"  输出文件  : {output_path}")
    print(f"  分辨率    : {args.resolution.upper()}")
    print(f"  目标帧率  : {args.fps} fps")
    if is_svo(output_path):
        print(f"  压缩格式  : {args.compression.upper()}（SVO）")
    print(f"  实时预览  : {'开启' if show_preview else '关闭'}")
    print("=" * 55)

    # ---- 初始化 ZED ----
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = RESOLUTION_MAP[args.resolution]
    init_params.camera_fps        = args.fps
    init_params.coordinate_units  = sl.UNIT.METER

    print("\n[ZED] 正在打开相机...")
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ZED] 打开相机失败: {status}")
        sys.exit(1)

    cam_info = zed.get_camera_information()
    res = cam_info.camera_configuration.resolution
    print(f"[ZED] 相机已就绪：{res.width}×{res.height} @ {args.fps}fps\n")

    try:
        if is_svo(output_path):
            record_svo(
                zed, output_path,
                COMPRESSION_MAP[args.compression],
                show_preview
            )
        else:
            record_mp4(zed, output_path, args.fps, show_preview)
    finally:
        zed.close()
        print("[ZED] 相机已关闭")


if __name__ == "__main__":
    main()
