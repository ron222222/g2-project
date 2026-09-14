#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
18_xyz_locator.py
智元 G2 头部 RGBD 像素点转三维坐标测试 V2

已适配当前 GDK 2.6.3 的 CameraIntrinsic 数据结构：
    intrinsic.intrinsic = [fx, fy, cx, cy]

功能：
1. 获取 kHeadColor 彩色图像与 kHeadDepth 深度图像
2. 鼠标点击彩色图像中的产品中心
3. 按分辨率比例将彩色图坐标映射到深度图坐标
4. 使用点击点附近 7x7 区域有效深度的中位数
5. 根据深度相机内参计算深度相机坐标系下的 X、Y、Z

注意：
- 本程序只读取和计算，不控制机器人。
- 输出 XYZ 属于头部深度相机坐标系，不是 base_link 坐标系。
- 下一阶段需要使用 TF/外参转换到 base_link。
- ESC 退出。
"""

import time
import cv2
import numpy as np
import agibot_gdk

WINDOW_NAME = "G2 Head RGBD XYZ Locator"
DEPTH_RADIUS = 3
MIN_DEPTH_RAW = 50.0
MAX_DEPTH_RAW = 10000.0

clicked_x = -1
clicked_y = -1
last_result = None


def mouse_callback(event, x, y, flags, param):
    global clicked_x, clicked_y
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_x = x
        clicked_y = y
        print(f"\n点击彩色图像像素: ({x}, {y})")


def decode_color(image):
    """解码彩色图像。"""
    data = np.frombuffer(image.data, dtype=np.uint8)

    # JPEG/PNG 等压缩格式
    frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if frame is not None:
        return frame

    # 未压缩 BGR/RGB 的兼容处理
    expected_size = image.width * image.height * 3
    if image.bit_depth == 8 and data.size == expected_size:
        frame = data.reshape((image.height, image.width, 3))
        try:
            if image.color_format == agibot_gdk.ColorFormat.RGB:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception:
            pass
        return frame

    return None


def decode_depth(image):
    """解码 16 位或 32 位深度图像。"""
    try:
        if image.bit_depth == 16:
            depth = np.frombuffer(image.data, dtype=np.uint16)
        elif image.bit_depth == 32:
            depth = np.frombuffer(image.data, dtype=np.float32)
        else:
            # 当前 G2 深度通常为 Z16，未知时优先尝试 uint16
            depth = np.frombuffer(image.data, dtype=np.uint16)

        return depth.reshape((image.height, image.width))
    except Exception as exc:
        print(f"深度图解码失败: {exc}")
        return None


def read_depth_intrinsics(camera):
    """
    当前 GDK 返回：
        CameraIntrinsic.intrinsic = [fx, fy, cx, cy]
    """
    camera_intrinsic = camera.get_camera_intrinsic(
        agibot_gdk.CameraType.kHeadDepth
    )

    if not hasattr(camera_intrinsic, "intrinsic"):
        raise RuntimeError("CameraIntrinsic 中不存在 intrinsic 字段")

    values = list(camera_intrinsic.intrinsic)
    if len(values) < 4:
        raise RuntimeError(
            f"CameraIntrinsic.intrinsic 数据不足，实际内容: {values}"
        )

    fx = float(values[0])
    fy = float(values[1])
    cx = float(values[2])
    cy = float(values[3])

    if fx <= 0.0 or fy <= 0.0:
        raise RuntimeError(f"相机焦距无效: fx={fx}, fy={fy}")

    print("HeadDepth 相机内参读取成功:")
    print(f"  fx = {fx:.9f}")
    print(f"  fy = {fy:.9f}")
    print(f"  cx = {cx:.9f}")
    print(f"  cy = {cy:.9f}")
    print(f"  distortion = {list(camera_intrinsic.distortion)}")

    return fx, fy, cx, cy


def get_median_depth(depth_map, u, v, radius=DEPTH_RADIUS):
    """获取点击点邻域内有效深度的中位数。"""
    height, width = depth_map.shape[:2]

    x0 = max(0, u - radius)
    x1 = min(width, u + radius + 1)
    y0 = max(0, v - radius)
    y1 = min(height, v + radius + 1)

    patch = depth_map[y0:y1, x0:x1].astype(np.float64)
    valid = patch[np.isfinite(patch)]
    valid = valid[
        (valid > MIN_DEPTH_RAW) &
        (valid < MAX_DEPTH_RAW)
    ]

    if valid.size == 0:
        return None, 0

    return float(np.median(valid)), int(valid.size)


def depth_to_meters(raw_depth):
    """
    当前实测深度值约为 1577，按常见 Z16 毫米单位换算为 1.577 m。
    若返回值小于等于 20，则按米制浮点深度处理。
    """
    if raw_depth > 20.0:
        return raw_depth * 0.001, "mm"
    return raw_depth, "m"


def main():
    global last_result

    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK 初始化失败")
        return

    camera = agibot_gdk.Camera()
    time.sleep(3.0)

    try:
        fx, fy, cx, cy = read_depth_intrinsics(camera)

        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, mouse_callback)

        xyz_text = "Click product center"
        pixel_text = ""
        size_text = ""

        while True:
            color_image = camera.get_latest_image(
                agibot_gdk.CameraType.kHeadColor,
                1000.0
            )
            depth_image = camera.get_latest_image(
                agibot_gdk.CameraType.kHeadDepth,
                1000.0
            )

            if color_image is None or depth_image is None:
                continue

            frame = decode_color(color_image)
            depth_map = decode_depth(depth_image)

            if frame is None or depth_map is None:
                continue

            color_h, color_w = frame.shape[:2]
            depth_h, depth_w = depth_map.shape[:2]
            size_text = (
                f"Color={color_w}x{color_h} "
                f"Depth={depth_w}x{depth_h}"
            )

            if clicked_x >= 0 and clicked_y >= 0:
                # 彩色图与深度图分辨率不同时，按比例映射像素位置
                depth_u = int(round(clicked_x * depth_w / color_w))
                depth_v = int(round(clicked_y * depth_h / color_h))
                depth_u = max(0, min(depth_w - 1, depth_u))
                depth_v = max(0, min(depth_h - 1, depth_v))

                raw_depth, sample_count = get_median_depth(
                    depth_map, depth_u, depth_v
                )

                cv2.circle(
                    frame,
                    (clicked_x, clicked_y),
                    7,
                    (0, 0, 255),
                    -1
                )

                if raw_depth is not None:
                    z_m, raw_unit = depth_to_meters(raw_depth)

                    # 针孔相机反投影公式
                    x_m = (depth_u - cx) * z_m / fx
                    y_m = (depth_v - cy) * z_m / fy

                    xyz_text = (
                        f"Camera XYZ(m): "
                        f"{x_m:.3f}, {y_m:.3f}, {z_m:.3f}"
                    )
                    pixel_text = (
                        f"RGB({clicked_x},{clicked_y}) "
                        f"Depth({depth_u},{depth_v}) "
                        f"raw={raw_depth:.1f}{raw_unit}"
                    )

                    current_result = (
                        clicked_x,
                        clicked_y,
                        depth_u,
                        depth_v,
                        round(raw_depth, 3),
                        round(x_m, 6),
                        round(y_m, 6),
                        round(z_m, 6)
                    )

                    if current_result != last_result:
                        print(f"深度图像素: ({depth_u}, {depth_v})")
                        print(f"有效深度样本数: {sample_count}")
                        print(f"原始深度: {raw_depth:.3f} {raw_unit}")
                        print("头部深度相机坐标系 XYZ，单位 m:")
                        print(f"  X = {x_m:.6f}")
                        print(f"  Y = {y_m:.6f}")
                        print(f"  Z = {z_m:.6f}")
                        print("注意：当前 XYZ 尚未转换到 base_link。")
                        last_result = current_result
                else:
                    xyz_text = "Invalid depth near clicked point"
                    pixel_text = (
                        f"RGB({clicked_x},{clicked_y}) "
                        f"Depth({depth_u},{depth_v})"
                    )

            cv2.putText(
                frame,
                xyz_text,
                (15, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.70,
                (0, 255, 0),
                2
            )
            cv2.putText(
                frame,
                pixel_text,
                (15, 68),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 255, 255),
                2
            )
            cv2.putText(
                frame,
                size_text,
                (15, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 0),
                2
            )

            cv2.imshow(WINDOW_NAME, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break

    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as exc:
        print(f"程序异常: {exc}")
        raise
    finally:
        cv2.destroyAllWindows()
        try:
            camera.close_camera()
        except Exception:
            pass
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
