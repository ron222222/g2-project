#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
19_camera_xyz_to_base_link_v3.py
智元 G2：头部 RGBD 坐标转换到 base_link，V3

V3 关键修正：
  不再把 kHeadRGBDToHeadLink3 的静态外参直接当作 base_link 到相机的完整变换。
  正确链路按以下两段组合：

    相机坐标 -> head_link3 -> base_link

    p_base = T_base_head * T_head_camera * p_camera

其中：
  T_base_head   = TF.get_tf_from_base_link("head_link3")
  T_head_camera = TF.get_tf_from_sensor(kHeadRGBDToHeadLink3)

程序只计算坐标，不控制机器人。
ESC 退出。
"""

import time
import cv2
import numpy as np
import agibot_gdk

WINDOW_NAME = "G2 Camera XYZ to Base Link V3"
DEPTH_RADIUS = 3
MIN_DEPTH_RAW = 50.0
MAX_DEPTH_RAW = 10000.0
TABLE_HEIGHT_M = 0.750
TABLE_Z_TOLERANCE_M = 0.150
CAMERA_TIMEOUT_MS = 1000.0
RETRY_SLEEP_S = 0.05
MAX_CONSECUTIVE_FAILURES = 20
LOG_EVERY_N_FAILURES = 5
MAX_TIMESTAMP_DELTA_MS = 150.0

clicked_x = -1
clicked_y = -1
last_result = None


def mouse_callback(event, x, y, flags, param):
    global clicked_x, clicked_y
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_x, clicked_y = x, y
        print(f"\n点击彩色图像像素: ({x}, {y})")


def decode_color(image):
    data = np.frombuffer(image.data, dtype=np.uint8)
    frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if frame is not None:
        return frame
    expected = image.width * image.height * 3
    if image.bit_depth == 8 and data.size == expected:
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
        if image.bit_depth == 32:
            data = np.frombuffer(image.data, dtype=np.float32)
        else:
            data = np.frombuffer(image.data, dtype=np.uint16)
        return data.reshape((image.height, image.width))
    except Exception as exc:
        print(f"深度图解码失败: {exc}")
        return None


def read_depth_intrinsics(camera):
    obj = camera.get_camera_intrinsic(agibot_gdk.CameraType.kHeadDepth)
    values = list(obj.intrinsic)
    if len(values) < 4:
        raise RuntimeError(f"CameraIntrinsic.intrinsic 数据不足: {values}")
    fx, fy, cx, cy = map(float, values[:4])
    print("HeadDepth 相机内参:")
    print(f"  fx={fx:.9f}, fy={fy:.9f}, cx={cx:.9f}, cy={cy:.9f}")
    return fx, fy, cx, cy


def median_depth(depth_map, u, v, radius=DEPTH_RADIUS):
    h, w = depth_map.shape[:2]
    patch = depth_map[
        max(0, v-radius):min(h, v+radius+1),
        max(0, u-radius):min(w, u+radius+1)
    ].astype(np.float64)
    valid = patch[np.isfinite(patch)]
    valid = valid[(valid > MIN_DEPTH_RAW) & (valid < MAX_DEPTH_RAW)]
    if valid.size == 0:
        return None, 0
    return float(np.median(valid)), int(valid.size)


def depth_to_meters(raw):
    return (raw * 0.001, "mm") if raw > 20.0 else (raw, "m")


def quaternion_to_matrix(x, y, z, w):
    q = np.array([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise RuntimeError("四元数模长接近0")
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ], dtype=np.float64)


def transform_to_matrix(t):
    m = np.eye(4, dtype=np.float64)
    m[:3, :3] = quaternion_to_matrix(
        t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
    )
    m[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return m


def print_transform(name, t):
    print(name)
    print(
        f"  translation=[{t.translation.x:.6f}, "
        f"{t.translation.y:.6f}, {t.translation.z:.6f}]"
    )
    print(
        f"  quaternion=[{t.rotation.x:.6f}, {t.rotation.y:.6f}, "
        f"{t.rotation.z:.6f}, {t.rotation.w:.6f}]"
    )


def build_camera_to_base_matrix(tf_api):
    # 动态变换：head_link3 在 base_link 中的位置和方向
    base_to_head_tf = tf_api.get_tf_from_base_link("head_link3")

    # 静态外参：头部 RGBD 到 head_link3
    camera_to_head_tf = tf_api.get_tf_from_sensor(
        agibot_gdk.SensorExtrinsicType.kHeadRGBDToHeadLink3
    )

    print_transform("T_base_head：base_link -> head_link3", base_to_head_tf)
    print_transform("T_head_camera：Head RGBD -> head_link3", camera_to_head_tf)

    t_base_head = transform_to_matrix(base_to_head_tf)
    t_head_camera = transform_to_matrix(camera_to_head_tf)

    # p_base = T_base_head * T_head_camera * p_camera
    t_base_camera = t_base_head @ t_head_camera
    print("组合变换完成：T_base_camera = T_base_head @ T_head_camera")
    print("T_base_camera=")
    print(np.array2string(t_base_camera, precision=6, suppress_small=True))
    return t_base_camera


def transform_point(matrix, xyz):
    point = np.array([xyz[0], xyz[1], xyz[2], 1.0], dtype=np.float64)
    return (matrix @ point)[:3]


def safe_get(camera, camera_type):
    try:
        return camera.get_latest_image(camera_type, CAMERA_TIMEOUT_MS)
    except Exception:
        return None


def table_check(base_xyz):
    delta = abs(float(base_xyz[2]) - TABLE_HEIGHT_M)
    state = "PASS" if delta <= TABLE_Z_TOLERANCE_M else "WARNING"
    return state, delta


def main():
    global last_result

    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK初始化失败")
        return

    camera = agibot_gdk.Camera()
    tf_api = agibot_gdk.TF()
    time.sleep(3.0)

    try:
        fx, fy, cx, cy = read_depth_intrinsics(camera)
        t_base_camera = build_camera_to_base_matrix(tf_api)

        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, mouse_callback)

        camera_text = "Click product or table"
        base_text = ""
        detail_text = ""
        check_text = ""
        failures = 0

        while True:
            color = safe_get(camera, agibot_gdk.CameraType.kHeadColor)
            depth = safe_get(camera, agibot_gdk.CameraType.kHeadDepth)

            if color is None or depth is None:
                failures += 1
                if failures == 1 or failures % LOG_EVERY_N_FAILURES == 0:
                    print(f"相机空帧，自动重试，连续失败={failures}")
                if failures >= MAX_CONSECUTIVE_FAILURES:
                    print("连续空帧较多，请关闭其他相机程序后重新运行。")
                    failures = 0
                time.sleep(RETRY_SLEEP_S)
                continue

            failures = 0
            frame = decode_color(color)
            depth_map = decode_depth(depth)
            if frame is None or depth_map is None:
                time.sleep(RETRY_SLEEP_S)
                continue

            color_h, color_w = frame.shape[:2]
            depth_h, depth_w = depth_map.shape[:2]
            dt_ms = abs(int(color.timestamp_ns)-int(depth.timestamp_ns))/1_000_000.0

            if clicked_x >= 0 and clicked_y >= 0:
                u = int(round(clicked_x * depth_w / color_w))
                v = int(round(clicked_y * depth_h / color_h))
                u = max(0, min(depth_w-1, u))
                v = max(0, min(depth_h-1, v))

                raw, count = median_depth(depth_map, u, v)
                cv2.circle(frame, (clicked_x, clicked_y), 7, (0, 0, 255), -1)

                if raw is not None:
                    z, unit = depth_to_meters(raw)
                    camera_xyz = np.array([
                        (u-cx)*z/fx,
                        (v-cy)*z/fy,
                        z
                    ], dtype=np.float64)
                    base_xyz = transform_point(t_base_camera, camera_xyz)
                    state, z_delta = table_check(base_xyz)

                    camera_text = (
                        f"Camera XYZ(m): {camera_xyz[0]:.3f}, "
                        f"{camera_xyz[1]:.3f}, {camera_xyz[2]:.3f}"
                    )
                    base_text = (
                        f"Base XYZ(m): {base_xyz[0]:.3f}, "
                        f"{base_xyz[1]:.3f}, {base_xyz[2]:.3f}"
                    )
                    detail_text = (
                        f"RGB({clicked_x},{clicked_y}) Depth({u},{v}) "
                        f"raw={raw:.1f}{unit} dt={dt_ms:.1f}ms"
                    )
                    check_text = f"TABLE-Z CHECK: {state} delta={z_delta:.3f}m"

                    result = (
                        clicked_x, clicked_y, round(raw, 3),
                        tuple(np.round(camera_xyz, 6)),
                        tuple(np.round(base_xyz, 6))
                    )
                    if result != last_result:
                        print(f"深度图像素=({u},{v})，有效样本={count}，dt={dt_ms:.3f}ms")
                        if dt_ms > MAX_TIMESTAMP_DELTA_MS:
                            print("警告：RGB/Depth时间戳差较大。")
                        print(f"原始深度={raw:.3f}{unit}")
                        print(
                            "Camera XYZ(m)="
                            f"[{camera_xyz[0]:.6f}, {camera_xyz[1]:.6f}, {camera_xyz[2]:.6f}]"
                        )
                        print(
                            "Base XYZ(m)="
                            f"[{base_xyz[0]:.6f}, {base_xyz[1]:.6f}, {base_xyz[2]:.6f}]"
                        )
                        print(
                            f"桌面参考Z={TABLE_HEIGHT_M:.3f}m，"
                            f"偏差={z_delta:.3f}m，{state}"
                        )
                        print("当前程序只计算坐标，不控制机器人。")
                        last_result = result
                else:
                    camera_text = "Invalid depth near clicked point"
                    base_text = ""
                    detail_text = f"RGB({clicked_x},{clicked_y}) Depth({u},{v})"
                    check_text = ""

            cv2.putText(frame, camera_text, (15,35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0,255,0), 2)
            cv2.putText(frame, base_text, (15,68),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0,200,255), 2)
            cv2.putText(frame, detail_text, (15,101),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255,255,0), 2)
            color_check = (0,255,0) if "PASS" in check_text else (0,0,255)
            cv2.putText(frame, check_text, (15,134),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_check, 2)
            cv2.putText(frame, "READ-ONLY: robot motion disabled", (15,167),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255,180,0), 2)

            cv2.imshow(WINDOW_NAME, frame)
            if cv2.waitKey(1) & 0xFF == 27:
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
