#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
19_camera_xyz_to_base_link.py
智元 G2 头部 RGBD 产品坐标转换到 base_link 测试程序

功能：
1. 读取 kHeadColor 与 kHeadDepth
2. 鼠标点击彩色图像中的产品中心
3. 使用 HeadDepth 内参计算头部深度相机坐标系 XYZ
4. 使用 TF.get_tf_from_sensor(kHeadRGBDToHeadLink3) 返回的变换，
   将相机坐标转换到 base_link 坐标系
5. 显示并打印 Camera XYZ 与 Base XYZ

安全说明：
- 本程序只读取传感器和计算坐标，不发送任何机器人运动命令。
- 在进入抓取控制前，请用已知桌面高度 0.750 m 验证 Base Z。
- 如果点击桌面/产品后 Base Z 明显偏离 0.750 m，请不要用于机械臂控制，
  需要进一步确认 TF 方向、RGB/Depth 对齐以及桌面高度参考系。
- ESC 退出。
"""

import time
import math
import cv2
import numpy as np
import agibot_gdk

WINDOW_NAME = "G2 Camera XYZ to Base Link"
DEPTH_RADIUS = 3
MIN_DEPTH_RAW = 50.0
MAX_DEPTH_RAW = 10000.0
TABLE_HEIGHT_M = 0.750
TABLE_Z_TOLERANCE_M = 0.150

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
    # 当前 G2 实测原始深度约 1579，按毫米解释。
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
    """四元数 [x,y,z,w] 转 3x3 旋转矩阵。"""
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
    """
    根据 GDK 2.6.3 文档，get_tf_from_sensor() 返回从 base_link 到
    指定传感器的变换。这里使用头部 RGBD 传感器枚举。
    """
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
        return "TABLE-Z CHECK: PASS"
    return "TABLE-Z CHECK: WARNING"


def main():
    global last_result

    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK 初始化失败")
        return

    camera = agibot_gdk.Camera()
    tf_interface = agibot_gdk.TF()
    time.sleep(3.0)

    try:
        fx, fy, cx, cy = read_depth_intrinsics(camera)
        base_to_camera_matrix = read_base_to_head_rgbd_transform(tf_interface)

        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, mouse_callback)

        camera_text = "Click product center"
        base_text = ""
        detail_text = ""
        check_text = ""

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

                    check_text = table_height_check(base_xyz)
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
                        f"raw={raw_depth:.1f}{raw_unit}"
                    )

                    current_result = (
                        clicked_x, clicked_y,
                        round(raw_depth, 3),
                        tuple(np.round(camera_xyz, 6)),
                        tuple(np.round(base_xyz, 6)),
                    )

                    if current_result != last_result:
                        print(f"深度图像素: ({depth_u}, {depth_v})")
                        print(f"有效深度样本数: {sample_count}")
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
                            f"Base Z偏差={abs(base_xyz[2]-TABLE_HEIGHT_M):.3f} m"
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
                frame, camera_text, (15, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                (0, 255, 0), 2
            )
            cv2.putText(
                frame, base_text, (15, 68),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                (0, 200, 255), 2
            )
            cv2.putText(
                frame, detail_text, (15, 101),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 0), 2
            )
            check_color = (
                (0, 255, 0)
                if "PASS" in check_text
                else (0, 0, 255)
            )
            cv2.putText(
                frame, check_text, (15, 134),
                cv2.FONT_HERSHEY_SIMPLEX, 0.60,
                check_color, 2
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
