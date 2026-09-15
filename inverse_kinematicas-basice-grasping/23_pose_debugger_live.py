#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
23_pose_debugger_live.py
智元 G2 实时位姿与安全状态调试器，READ ONLY

运行环境：
  /usr/bin/python3.10
  ROS 2 Humble
  agibot_gdk 2.6.3

实时显示：
  RGB Pixel -> Camera XYZ -> Base XYZ
  arm_r_end_link 当前 Base XYZ
  PreGrasp XYZ（目标上方 150 mm）
  右手当前位置到 PreGrasp 的 DX / DY / DZ / Distance
  Motion mode / error_code / collision_count
  arm_r_end_link 对应 Wrench 的 Force / Torque
  静止基线与 Delta Force / Delta Torque

安全边界：
  1. 本程序不调用任何运动控制接口。
  2. 本程序不控制机械臂和夹爪。
  3. 仅用于验证目标、末端位置及力/力矩监测数据。
  4. ESC 退出。
  5. R 键重新采集右臂 Wrench 基线。
  6. S 键保存当前调试快照到 pose_debug_snapshot.json。
"""

import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import agibot_gdk

WINDOW_NAME = "G2 Live Pose Debugger - READ ONLY"
SNAPSHOT_PATH = Path(__file__).with_name("pose_debug_snapshot.json")

HEAD_FRAME = "head_link3"
RIGHT_END_FRAME = "arm_r_end_link"

PREGRASP_OFFSET_Z_M = 0.150
TABLE_HEIGHT_M = 0.750
TABLE_Z_TOLERANCE_M = 0.150

DEPTH_RADIUS = 3
MIN_DEPTH_RAW = 50.0
MAX_DEPTH_RAW = 10000.0
CAMERA_TIMEOUT_MS = 1000.0
RETRY_SLEEP_S = 0.05
MAX_TIMESTAMP_DELTA_MS = 150.0

BASELINE_SAMPLE_COUNT = 30
STATUS_PRINT_INTERVAL_S = 1.0

clicked_x = -1
clicked_y = -1
latest_snapshot = None


def mouse_callback(event, x, y, flags, param):
    global clicked_x, clicked_y
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_x = x
        clicked_y = y
        print()
        print(f"点击目标像素: RGB({x}, {y})")


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
        if image.bit_depth == 32:
            data = np.frombuffer(image.data, dtype=np.float32)
        else:
            data = np.frombuffer(image.data, dtype=np.uint16)
        return data.reshape((image.height, image.width))
    except Exception as exc:
        print(f"深度图解码失败: {exc}")
        return None


def read_depth_intrinsics(camera):
    intrinsic_obj = camera.get_camera_intrinsic(
        agibot_gdk.CameraType.kHeadDepth
    )
    values = list(intrinsic_obj.intrinsic)
    if len(values) < 4:
        raise RuntimeError(f"CameraIntrinsic.intrinsic 数据不足: {values}")

    fx, fy, cx, cy = map(float, values[:4])
    if fx <= 0.0 or fy <= 0.0:
        raise RuntimeError(f"相机焦距无效: fx={fx}, fy={fy}")

    print("HeadDepth 相机内参:")
    print(f"  fx={fx:.9f}")
    print(f"  fy={fy:.9f}")
    print(f"  cx={cx:.9f}")
    print(f"  cy={cy:.9f}")
    return fx, fy, cx, cy


def median_depth(depth_map, u, v, radius=DEPTH_RADIUS):
    height, width = depth_map.shape[:2]
    x0 = max(0, u - radius)
    x1 = min(width, u + radius + 1)
    y0 = max(0, v - radius)
    y1 = min(height, v + radius + 1)

    patch = depth_map[y0:y1, x0:x1].astype(np.float64)
    valid = patch[np.isfinite(patch)]
    valid = valid[(valid > MIN_DEPTH_RAW) & (valid < MAX_DEPTH_RAW)]

    if valid.size == 0:
        return None, 0

    return float(np.median(valid)), int(valid.size)


def depth_to_meters(raw_depth):
    if raw_depth > 20.0:
        return raw_depth * 0.001, "mm"
    return raw_depth, "m"


def quaternion_to_matrix(x, y, z, w):
    quaternion = np.array([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        raise RuntimeError("四元数模长接近 0")

    x, y, z, w = quaternion / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w),     2 * (x*z + y*w)],
        [2 * (x*y + z*w),     1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w),     2 * (y*z + x*w),     1 - 2 * (x*x + y*y)],
    ], dtype=np.float64)


def transform_to_matrix(transform):
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_to_matrix(
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    )
    matrix[:3, 3] = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    return matrix


def transform_point(matrix, xyz):
    point_h = np.array([xyz[0], xyz[1], xyz[2], 1.0], dtype=np.float64)
    return (matrix @ point_h)[:3]


def build_camera_to_base_matrix(tf_api):
    base_to_head = tf_api.get_tf_from_base_link(HEAD_FRAME)
    camera_to_head = tf_api.get_tf_from_sensor(
        agibot_gdk.SensorExtrinsicType.kHeadRGBDToHeadLink3
    )

    t_base_head = transform_to_matrix(base_to_head)
    t_head_camera = transform_to_matrix(camera_to_head)
    t_base_camera = t_base_head @ t_head_camera

    print("相机到 base_link 的 TF 链已建立:")
    print("  T_base_camera = T_base_head @ T_head_camera")
    return t_base_camera


def read_right_end_pose(tf_api):
    transform = tf_api.get_tf_from_base_link(RIGHT_END_FRAME)
    position = np.array([
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ], dtype=np.float64)
    orientation = np.array([
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    ], dtype=np.float64)
    return position, orientation


def safe_get_image(camera, camera_type):
    try:
        return camera.get_latest_image(camera_type, CAMERA_TIMEOUT_MS)
    except Exception:
        return None


def vector3_to_array(vector):
    return np.array([vector.x, vector.y, vector.z], dtype=np.float64)


def find_right_arm_wrench(status):
    frame_names = list(status.frame_names)
    wrenches = list(status.wrenches)

    if RIGHT_END_FRAME not in frame_names:
        return None, None

    index = frame_names.index(RIGHT_END_FRAME)
    if index >= len(wrenches):
        return None, index

    wrench = wrenches[index]
    force = vector3_to_array(wrench.force)
    torque = vector3_to_array(wrench.torque)
    return (force, torque), index


def collision_pairs(status):
    return list(zip(status.collision_pairs_1, status.collision_pairs_2))


def xyz_list(values):
    return [round(float(value), 6) for value in values]


def make_snapshot(
    rgb_pixel,
    depth_pixel,
    raw_depth,
    camera_xyz,
    base_xyz,
    hand_xyz,
    hand_quaternion,
    pregrasp_xyz,
    delta_xyz,
    distance,
    status,
    right_force,
    right_torque,
    baseline_force,
    baseline_torque,
    delta_force,
    delta_torque,
    timestamp_delta_ms,
):
    return {
        "read_only": True,
        "coordinate_frame": "base_link",
        "rgb_pixel": list(rgb_pixel),
        "depth_pixel": list(depth_pixel),
        "raw_depth": round(float(raw_depth), 3),
        "camera_xyz_m": xyz_list(camera_xyz),
        "target_base_xyz_m": xyz_list(base_xyz),
        "right_hand_base_xyz_m": xyz_list(hand_xyz),
        "right_hand_orientation_xyzw": [
            round(float(value), 8) for value in hand_quaternion
        ],
        "pregrasp_base_xyz_m": xyz_list(pregrasp_xyz),
        "hand_to_pregrasp_delta_xyz_m": xyz_list(delta_xyz),
        "hand_to_pregrasp_distance_m": round(float(distance), 6),
        "motion_status": {
            "mode": int(status.mode),
            "error_code": int(status.error_code),
            "error_msg": str(status.error_msg),
            "collision_pairs": [list(pair) for pair in collision_pairs(status)],
        },
        "right_arm_wrench": {
            "force": xyz_list(right_force),
            "torque": xyz_list(right_torque),
            "baseline_force": xyz_list(baseline_force),
            "baseline_torque": xyz_list(baseline_torque),
            "delta_force": xyz_list(delta_force),
            "delta_torque": xyz_list(delta_torque),
        },
        "rgb_depth_timestamp_delta_ms": round(float(timestamp_delta_ms), 3),
    }


def draw_line(frame, text, row, color, scale=0.48):
    y = 28 + row * 27
    cv2.putText(
        frame,
        text,
        (12, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        1,
        cv2.LINE_AA,
    )


def main():
    global latest_snapshot

    print("23_pose_debugger_live.py")
    print("READ ONLY MODE")
    print("不会控制机械臂和夹爪")
    print("R: 重新采集 Wrench 基线")
    print("S: 保存当前快照")
    print("ESC: 退出")

    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK 初始化失败")
        return

    camera = agibot_gdk.Camera()
    tf_api = agibot_gdk.TF()
    robot = agibot_gdk.Robot()
    time.sleep(3.0)

    baseline_force_samples = []
    baseline_torque_samples = []
    baseline_force = None
    baseline_torque = None
    last_console_print = 0.0

    try:
        fx, fy, cx, cy = read_depth_intrinsics(camera)
        t_base_camera = build_camera_to_base_matrix(tf_api)

        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, mouse_callback)

        while True:
            color_image = safe_get_image(
                camera,
                agibot_gdk.CameraType.kHeadColor,
            )
            depth_image = safe_get_image(
                camera,
                agibot_gdk.CameraType.kHeadDepth,
            )

            try:
                status = robot.get_motion_control_status()
            except Exception as exc:
                print(f"读取 Motion Status 失败: {exc}")
                time.sleep(RETRY_SLEEP_S)
                continue

            wrench_result, right_wrench_index = find_right_arm_wrench(status)
            if wrench_result is None:
                print("未找到 arm_r_end_link 对应 Wrench")
                time.sleep(RETRY_SLEEP_S)
                continue

            right_force, right_torque = wrench_result

            if baseline_force is None:
                baseline_force_samples.append(right_force.copy())
                baseline_torque_samples.append(right_torque.copy())

                if len(baseline_force_samples) >= BASELINE_SAMPLE_COUNT:
                    baseline_force = np.mean(
                        np.asarray(baseline_force_samples), axis=0
                    )
                    baseline_torque = np.mean(
                        np.asarray(baseline_torque_samples), axis=0
                    )
                    print()
                    print("右臂 Wrench 基线采集完成")
                    print(f"  baseline force = {xyz_list(baseline_force)}")
                    print(f"  baseline torque = {xyz_list(baseline_torque)}")

            if color_image is None or depth_image is None:
                time.sleep(RETRY_SLEEP_S)
                continue

            frame = decode_color(color_image)
            depth_map = decode_depth(depth_image)
            if frame is None or depth_map is None:
                time.sleep(RETRY_SLEEP_S)
                continue

            hand_xyz, hand_quaternion = read_right_end_pose(tf_api)
            pairs = collision_pairs(status)

            if baseline_force is None:
                delta_force = np.zeros(3, dtype=np.float64)
                delta_torque = np.zeros(3, dtype=np.float64)
                baseline_text = (
                    f"Baseline {len(baseline_force_samples)}/{BASELINE_SAMPLE_COUNT}"
                )
            else:
                delta_force = right_force - baseline_force
                delta_torque = right_torque - baseline_torque
                baseline_text = "Baseline READY"

            draw_line(
                frame,
                f"Hand Base XYZ: {hand_xyz[0]:.3f}, {hand_xyz[1]:.3f}, {hand_xyz[2]:.3f}",
                0,
                (255, 255, 0),
            )
            draw_line(
                frame,
                f"Mode={status.mode} Error={status.error_code} Collision={len(pairs)}",
                1,
                (0, 255, 0) if status.error_code == 0 and not pairs else (0, 0, 255),
            )
            draw_line(
                frame,
                f"Right F: {right_force[0]:.2f}, {right_force[1]:.2f}, {right_force[2]:.2f}",
                2,
                (200, 200, 255),
            )
            draw_line(
                frame,
                f"Right T: {right_torque[0]:.3f}, {right_torque[1]:.3f}, {right_torque[2]:.3f}",
                3,
                (200, 200, 255),
            )
            draw_line(
                frame,
                f"Delta F: {delta_force[0]:.2f}, {delta_force[1]:.2f}, {delta_force[2]:.2f}",
                4,
                (0, 200, 255),
            )
            draw_line(
                frame,
                f"Delta T: {delta_torque[0]:.3f}, {delta_torque[1]:.3f}, {delta_torque[2]:.3f}",
                5,
                (0, 200, 255),
            )
            draw_line(frame, baseline_text, 6, (255, 180, 0))

            if clicked_x >= 0 and clicked_y >= 0:
                color_h, color_w = frame.shape[:2]
                depth_h, depth_w = depth_map.shape[:2]

                depth_u = int(round(clicked_x * depth_w / color_w))
                depth_v = int(round(clicked_y * depth_h / color_h))
                depth_u = max(0, min(depth_w - 1, depth_u))
                depth_v = max(0, min(depth_h - 1, depth_v))

                raw_depth, valid_count = median_depth(
                    depth_map,
                    depth_u,
                    depth_v,
                )

                cv2.circle(
                    frame,
                    (clicked_x, clicked_y),
                    6,
                    (0, 0, 255),
                    -1,
                )

                if raw_depth is not None:
                    z_m, raw_unit = depth_to_meters(raw_depth)
                    camera_xyz = np.array([
                        (depth_u - cx) * z_m / fx,
                        (depth_v - cy) * z_m / fy,
                        z_m,
                    ], dtype=np.float64)
                    base_xyz = transform_point(t_base_camera, camera_xyz)
                    pregrasp_xyz = base_xyz.copy()
                    pregrasp_xyz[2] += PREGRASP_OFFSET_Z_M
                    delta_xyz = pregrasp_xyz - hand_xyz
                    distance = float(np.linalg.norm(delta_xyz))

                    timestamp_delta_ms = abs(
                        int(color_image.timestamp_ns)
                        - int(depth_image.timestamp_ns)
                    ) / 1_000_000.0

                    table_delta = abs(float(base_xyz[2]) - TABLE_HEIGHT_M)
                    table_state = (
                        "PASS"
                        if table_delta <= TABLE_Z_TOLERANCE_M
                        else "WARNING"
                    )

                    draw_line(
                        frame,
                        f"Camera XYZ: {camera_xyz[0]:.3f}, {camera_xyz[1]:.3f}, {camera_xyz[2]:.3f}",
                        8,
                        (0, 255, 0),
                    )
                    draw_line(
                        frame,
                        f"Target Base XYZ: {base_xyz[0]:.3f}, {base_xyz[1]:.3f}, {base_xyz[2]:.3f}",
                        9,
                        (0, 255, 0),
                    )
                    draw_line(
                        frame,
                        f"PreGrasp XYZ: {pregrasp_xyz[0]:.3f}, {pregrasp_xyz[1]:.3f}, {pregrasp_xyz[2]:.3f}",
                        10,
                        (255, 255, 0),
                    )
                    draw_line(
                        frame,
                        f"Delta XYZ: {delta_xyz[0]:.3f}, {delta_xyz[1]:.3f}, {delta_xyz[2]:.3f}",
                        11,
                        (0, 200, 255),
                    )
                    draw_line(
                        frame,
                        f"Distance={distance:.3f}m TableZ={table_state} dt={timestamp_delta_ms:.1f}ms",
                        12,
                        (0, 255, 0) if table_state == "PASS" else (0, 0, 255),
                    )
                    draw_line(
                        frame,
                        f"Depth raw={raw_depth:.1f}{raw_unit} samples={valid_count}",
                        13,
                        (220, 220, 220),
                    )

                    if baseline_force is not None:
                        latest_snapshot = make_snapshot(
                            (clicked_x, clicked_y),
                            (depth_u, depth_v),
                            raw_depth,
                            camera_xyz,
                            base_xyz,
                            hand_xyz,
                            hand_quaternion,
                            pregrasp_xyz,
                            delta_xyz,
                            distance,
                            status,
                            right_force,
                            right_torque,
                            baseline_force,
                            baseline_torque,
                            delta_force,
                            delta_torque,
                            timestamp_delta_ms,
                        )

                    now = time.time()
                    if now - last_console_print >= STATUS_PRINT_INTERVAL_S:
                        print()
                        print("实时位姿摘要")
                        print(f"  Camera XYZ = {xyz_list(camera_xyz)}")
                        print(f"  Target Base XYZ = {xyz_list(base_xyz)}")
                        print(f"  Right Hand XYZ = {xyz_list(hand_xyz)}")
                        print(f"  PreGrasp XYZ = {xyz_list(pregrasp_xyz)}")
                        print(f"  Delta XYZ = {xyz_list(delta_xyz)}")
                        print(f"  Distance = {distance:.6f} m")
                        print(f"  Motion mode = {status.mode}")
                        print(f"  Error code = {status.error_code}")
                        print(f"  Collision count = {len(pairs)}")
                        print(f"  Right force = {xyz_list(right_force)}")
                        print(f"  Right torque = {xyz_list(right_torque)}")
                        print(f"  Delta force = {xyz_list(delta_force)}")
                        print(f"  Delta torque = {xyz_list(delta_torque)}")
                        last_console_print = now

            draw_line(
                frame,
                "READ ONLY | R baseline | S save | ESC exit",
                15,
                (255, 180, 0),
            )

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                break

            if key in (ord("r"), ord("R")):
                baseline_force_samples.clear()
                baseline_torque_samples.clear()
                baseline_force = None
                baseline_torque = None
                latest_snapshot = None
                print()
                print("正在重新采集右臂 Wrench 基线")

            if key in (ord("s"), ord("S")):
                if latest_snapshot is None:
                    print()
                    print("当前无完整快照，请等待基线完成并点击目标")
                else:
                    SNAPSHOT_PATH.write_text(
                        json.dumps(latest_snapshot, ensure_ascii=False, indent=2),
                        encoding="utf-8",
                    )
                    print()
                    print(f"快照已保存: {SNAPSHOT_PATH}")

    except KeyboardInterrupt:
        print()
        print("用户中断")
    except Exception as exc:
        print(f"程序异常: {exc}")
        raise
    finally:
        cv2.destroyAllWindows()
        try:
            camera.close_camera()
        except Exception:
            pass
        try:
            agibot_gdk.gdk_release()
        except Exception:
            pass


if __name__ == "__main__":
    main()
