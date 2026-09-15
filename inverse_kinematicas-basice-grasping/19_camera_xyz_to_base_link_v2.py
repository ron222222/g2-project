#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
19_camera_xyz_to_base_link_v2.py
智元 G2 头部 RGBD 产品坐标转换到 base_link 测试程序 V2

V2 改进：
1. HeadColor / HeadDepth 空帧自动重试
2. get_latest_image() 异常捕获，不因单帧失败退出
3. 连续失败统计、限频提示与相机对象自动重建
4. RGB 与 Depth 时间戳差检查
5. 7x7 邻域深度中位数滤波
6. 输出 Camera XYZ、Base XYZ 和桌面高度校验

安全说明：
- 本程序只读取传感器并计算坐标，不发送任何运动命令。
- 进入机械臂控制前，必须用已知桌面高度验证 Base Z。
- 如果 TABLE-Z CHECK 显示 WARNING，请勿将坐标发送给机械臂。
- ESC 退出。
"""

import time
import cv2
import numpy as np
import agibot_gdk

WINDOW_NAME = "G2 Camera XYZ to Base Link V2"

# 深度读取参数
DEPTH_RADIUS = 3
MIN_DEPTH_RAW = 50.0
MAX_DEPTH_RAW = 10000.0

# 已知桌面高度，用于检查 base_link Z
TABLE_HEIGHT_M = 0.750
TABLE_Z_TOLERANCE_M = 0.150

# 相机容错参数
CAMERA_TIMEOUT_MS = 1000.0
RETRY_SLEEP_S = 0.05
MAX_CONSECUTIVE_FAILURES = 20
MAX_CAMERA_REBUILDS = 5
REBUILD_WAIT_S = 2.0
LOG_EVERY_N_FAILURES = 5

# RGB / Depth 时间同步提示阈值
MAX_TIMESTAMP_DELTA_MS = 150.0

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
    data = np.frombuffer(image.data, dtype=np.uint8)

    frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if frame is not None:
        return frame

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
    try:
        if image.bit_depth == 16:
            arr = np.frombuffer(image.data, dtype=np.uint16)
        elif image.bit_depth == 32:
            arr = np.frombuffer(image.data, dtype=np.float32)
        else:
            arr = np.frombuffer(image.data, dtype=np.uint16)

        return arr.reshape((image.height, image.width))
    except Exception as exc:
        print(f"深度图解码失败: {exc}")
        return None


def read_depth_intrinsics(camera):
    intrinsic_obj = camera.get_camera_intrinsic(
        agibot_gdk.CameraType.kHeadDepth
    )

    if not hasattr(intrinsic_obj, "intrinsic"):
        raise RuntimeError("CameraIntrinsic 中不存在 intrinsic 字段")

    values = list(intrinsic_obj.intrinsic)
    if len(values) < 4:
        raise RuntimeError(f"CameraIntrinsic.intrinsic 数据不足: {values}")

    fx, fy, cx, cy = map(float, values[:4])
    if fx <= 0.0 or fy <= 0.0:
        raise RuntimeError(f"相机焦距无效: fx={fx}, fy={fy}")

    print("HeadDepth 相机内参:")
    print(f"  fx={fx:.9f}, fy={fy:.9f}, cx={cx:.9f}, cy={cy:.9f}")
    return fx, fy, cx, cy


def get_median_depth(depth_map, u, v, radius=DEPTH_RADIUS):
    h, w = depth_map.shape[:2]
    x0, x1 = max(0, u-radius), min(w, u+radius+1)
    y0, y1 = max(0, v-radius), min(h, v+radius+1)

    patch = depth_map[y0:y1, x0:x1].astype(np.float64)
    valid = patch[np.isfinite(patch)]
    valid = valid[(valid > MIN_DEPTH_RAW) & (valid < MAX_DEPTH_RAW)]

    if valid.size == 0:
        return None, 0

    return float(np.median(valid)), int(valid.size)


def depth_to_meters(raw_depth):
    # 当前 G2 实测深度约 1579，按毫米解释。
    if raw_depth > 20.0:
        return raw_depth * 0.001, "mm"
    return raw_depth, "m"


def normalize_quaternion(q):
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise RuntimeError("TF 四元数模长接近 0")
    return q / norm


def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    x, y, z, w = normalize_quaternion([qx, qy, qz, qw])

    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def transform_to_matrix(transform):
    rotation = quaternion_to_rotation_matrix(
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    )

    translation = np.array([
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ], dtype=np.float64)

    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def transform_point(matrix, point_xyz):
    point_h = np.array([
        point_xyz[0], point_xyz[1], point_xyz[2], 1.0
    ], dtype=np.float64)
    result = matrix @ point_h
    return result[:3]


def read_base_to_head_rgbd_transform(tf_interface):
    transform = tf_interface.get_tf_from_sensor(
        agibot_gdk.SensorExtrinsicType.kHeadRGBDToHeadLink3
    )

    print("base_link 到 Head RGBD 的 TF:")
    print(
        "  translation = "
        f"[{transform.translation.x:.6f}, "
        f"{transform.translation.y:.6f}, "
        f"{transform.translation.z:.6f}]"
    )
    print(
        "  quaternion = "
        f"[{transform.rotation.x:.6f}, "
        f"{transform.rotation.y:.6f}, "
        f"{transform.rotation.z:.6f}, "
        f"{transform.rotation.w:.6f}]"
    )

    return transform_to_matrix(transform)


def table_height_check(base_xyz):
    delta = abs(float(base_xyz[2]) - TABLE_HEIGHT_M)
    if delta <= TABLE_Z_TOLERANCE_M:
        return "TABLE-Z CHECK: PASS", delta
    return "TABLE-Z CHECK: WARNING", delta


def safe_get_latest_image(camera, camera_type, camera_name):
    """捕获 GDK 空帧异常，失败时返回 None。"""
    try:
        image = camera.get_latest_image(camera_type, CAMERA_TIMEOUT_MS)
        if image is None:
            return None
        if not hasattr(image, "data"):
            return None
        return image
    except Exception as exc:
        return None


def close_camera_safely(camera):
    if camera is None:
        return
    try:
        camera.close_camera()
    except Exception:
        pass


def rebuild_camera(old_camera, rebuild_no):
    """关闭旧对象并重新创建 Camera。"""
    print(
        f"连续空帧达到 {MAX_CONSECUTIVE_FAILURES} 次，"
        f"正在重建 Camera 对象，第 {rebuild_no}/{MAX_CAMERA_REBUILDS} 次..."
    )
    close_camera_safely(old_camera)
    time.sleep(REBUILD_WAIT_S)

    new_camera = agibot_gdk.Camera()
    time.sleep(REBUILD_WAIT_S)
    print("Camera 对象已重建，继续尝试读取。")
    return new_camera


def main():
    global last_result

    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK 初始化失败")
        return

    camera = None
    tf_interface = None

    try:
        camera = agibot_gdk.Camera()
        tf_interface = agibot_gdk.TF()
        time.sleep(3.0)

        fx, fy, cx, cy = read_depth_intrinsics(camera)
        base_to_camera_matrix = read_base_to_head_rgbd_transform(tf_interface)

        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, mouse_callback)

        camera_text = "Click product center"
        base_text = ""
        detail_text = ""
        check_text = ""
        status_text = "Camera stream: OK"

        consecutive_failures = 0
        total_failures = 0
        rebuild_count = 0

        while True:
            color_image = safe_get_latest_image(
                camera,
                agibot_gdk.CameraType.kHeadColor,
                "HeadColor"
            )

            depth_image = safe_get_latest_image(
                camera,
                agibot_gdk.CameraType.kHeadDepth,
                "HeadDepth"
            )

            if color_image is None or depth_image is None:
                consecutive_failures += 1
                total_failures += 1

                if total_failures == 1 or total_failures % LOG_EVERY_N_FAILURES == 0:
                    missing = []
                    if color_image is None:
                        missing.append("HeadColor")
                    if depth_image is None:
                        missing.append("HeadDepth")
                    print(
                        f"相机空帧/读取失败: {','.join(missing)}, "
                        f"连续失败={consecutive_failures}, 总失败={total_failures}"
                    )

                status_text = (
                    f"Camera retry {consecutive_failures}/"
                    f"{MAX_CONSECUTIVE_FAILURES}"
                )

                if consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                    if rebuild_count >= MAX_CAMERA_REBUILDS:
                        raise RuntimeError(
                            "相机连续读取失败，且 Camera 对象重建次数已达到上限。"
                            "请确认其他相机程序已关闭，并重新启动本程序。"
                        )

                    rebuild_count += 1
                    camera = rebuild_camera(camera, rebuild_count)
                    fx, fy, cx, cy = read_depth_intrinsics(camera)
                    consecutive_failures = 0

                # 即使空帧也维持 GUI 响应
                blank = np.zeros((400, 640, 3), dtype=np.uint8)
                cv2.putText(
                    blank,
                    status_text,
                    (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 0, 255),
                    2
                )
                cv2.imshow(WINDOW_NAME, blank)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    break

                time.sleep(RETRY_SLEEP_S)
                continue

            # 两路图像都有效，重置连续失败计数
            consecutive_failures = 0
            status_text = f"Camera stream: OK, recoveries={rebuild_count}"

            frame = decode_color(color_image)
            depth_map = decode_depth(depth_image)

            if frame is None or depth_map is None:
                total_failures += 1
                if total_failures % LOG_EVERY_N_FAILURES == 0:
                    print("图像解码失败，正在自动重试。")
                time.sleep(RETRY_SLEEP_S)
                continue

            color_h, color_w = frame.shape[:2]
            depth_h, depth_w = depth_map.shape[:2]

            rgb_timestamp = int(color_image.timestamp_ns)
            depth_timestamp = int(depth_image.timestamp_ns)
            timestamp_delta_ms = abs(rgb_timestamp - depth_timestamp) / 1_000_000.0

            if clicked_x >= 0 and clicked_y >= 0:
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

                    camera_xyz = np.array([
                        (depth_u - cx) * z_m / fx,
                        (depth_v - cy) * z_m / fy,
                        z_m,
                    ], dtype=np.float64)

                    base_xyz = transform_point(
                        base_to_camera_matrix,
                        camera_xyz
                    )

                    check_text, z_delta = table_height_check(base_xyz)

                    camera_text = (
                        f"Camera XYZ(m): "
                        f"{camera_xyz[0]:.3f}, "
                        f"{camera_xyz[1]:.3f}, "
                        f"{camera_xyz[2]:.3f}"
                    )
                    base_text = (
                        f"Base XYZ(m): "
                        f"{base_xyz[0]:.3f}, "
                        f"{base_xyz[1]:.3f}, "
                        f"{base_xyz[2]:.3f}"
                    )
                    detail_text = (
                        f"RGB({clicked_x},{clicked_y}) "
                        f"Depth({depth_u},{depth_v}) "
                        f"raw={raw_depth:.1f}{raw_unit} "
                        f"dt={timestamp_delta_ms:.1f}ms"
                    )

                    current_result = (
                        clicked_x,
                        clicked_y,
                        round(raw_depth, 3),
                        tuple(np.round(camera_xyz, 6)),
                        tuple(np.round(base_xyz, 6)),
                    )

                    if current_result != last_result:
                        print(f"深度图像素: ({depth_u}, {depth_v})")
                        print(f"有效深度样本数: {sample_count}")
                        print(f"RGB/Depth 时间戳差: {timestamp_delta_ms:.3f} ms")
                        if timestamp_delta_ms > MAX_TIMESTAMP_DELTA_MS:
                            print("警告：RGB/Depth 时间戳差较大，坐标可能不稳定。")
                        print(f"原始深度: {raw_depth:.3f} {raw_unit}")
                        print("头部深度相机坐标系 XYZ，单位 m:")
                        print(f"  X={camera_xyz[0]:.6f}")
                        print(f"  Y={camera_xyz[1]:.6f}")
                        print(f"  Z={camera_xyz[2]:.6f}")
                        print("base_link 坐标系 XYZ，单位 m:")
                        print(f"  X={base_xyz[0]:.6f}")
                        print(f"  Y={base_xyz[1]:.6f}")
                        print(f"  Z={base_xyz[2]:.6f}")
                        print(
                            f"桌面高度参考={TABLE_HEIGHT_M:.3f} m, "
                            f"Base Z偏差={z_delta:.3f} m"
                        )
                        print(check_text)
                        print("当前程序只计算坐标，不控制机器人。")
                        last_result = current_result
                else:
                    camera_text = "Invalid depth near clicked point"
                    base_text = ""
                    detail_text = (
                        f"RGB({clicked_x},{clicked_y}) "
                        f"Depth({depth_u},{depth_v})"
                    )
                    check_text = ""

            cv2.putText(
                frame,
                camera_text,
                (15, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 255, 0),
                2
            )
            cv2.putText(
                frame,
                base_text,
                (15, 68),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 200, 255),
                2
            )
            cv2.putText(
                frame,
                detail_text,
                (15, 101),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 255, 0),
                2
            )

            check_color = (
                (0, 255, 0)
                if "PASS" in check_text
                else (0, 0, 255)
            )
            cv2.putText(
                frame,
                check_text,
                (15, 134),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                check_color,
                2
            )
            cv2.putText(
                frame,
                status_text,
                (15, 167),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 180, 0),
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
        close_camera_safely(camera)
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
