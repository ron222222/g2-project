#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
41_gripper_tcp_pose_debugger.py

READ ONLY，不发送任何机器人运动或夹爪命令。

功能：
1. 读取 arm_r_end_link 实时位姿。
2. 优先读取 URDF 已定义的 gripper_r_center_link 实时位姿。
3. 同时根据 URDF 固定偏移 [0, 0, 0.14308] m 计算 TCP 位姿。
4. 对比 TF 直接值与计算值，验证 TCP 偏移。
5. 读取 product_xyz_v3.json / product_xyz_v2.json 中的产品 base_xyz。
6. 显示 TCP 到产品的 XYZ 差值和三维距离。
7. 按 S 保存快照，按 Q/ESC 退出。

URDF 依据：
arm_r_end_link -> gripper_r_center_link
xyz="0.0 0.0 0.14308", rpy="0 0 0"
"""

import json
import math
import time
from pathlib import Path
from typing import Optional, Tuple

import agibot_gdk

PROGRAM_NAME = "41_gripper_tcp_pose_debugger.py"
END_FRAME = "arm_r_end_link"
TCP_FRAME = "gripper_r_center_link"

TCP_OFFSET_END_M = [0.0, 0.0, 0.14308]
PRODUCT_FILES = ["product_xyz_v3.json", "product_xyz_v2.json"]
SNAPSHOT_FILE = "gripper_tcp_pose_snapshot.json"

UPDATE_HZ = 5.0
UPDATE_DT = 1.0 / UPDATE_HZ
TF_TIMEOUT_S = 10.0


def normalize_quaternion(q):
    norm = math.sqrt(sum(float(value) ** 2 for value in q))
    if norm < 1e-12:
        raise RuntimeError("四元数模长接近 0")
    return [float(value) / norm for value in q]


def quaternion_to_rotation_matrix(q):
    x, y, z, w = normalize_quaternion(q)
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def rotate_vector(rotation, vector):
    return [
        sum(rotation[row][column] * vector[column] for column in range(3))
        for row in range(3)
    ]


def vector_add(a, b):
    return [float(a[index]) + float(b[index]) for index in range(3)]


def vector_subtract(a, b):
    return [float(a[index]) - float(b[index]) for index in range(3)]


def vector_norm(vector):
    return math.sqrt(sum(float(value) ** 2 for value in vector))


def read_pose(tf_api, frame):
    transform = tf_api.get_tf_from_base_link(frame)
    position = [
        float(transform.translation.x),
        float(transform.translation.y),
        float(transform.translation.z),
    ]
    quaternion = normalize_quaternion([
        float(transform.rotation.x),
        float(transform.rotation.y),
        float(transform.rotation.z),
        float(transform.rotation.w),
    ])
    return position, quaternion


def wait_for_pose(tf_api, frame, timeout_s=TF_TIMEOUT_S):
    deadline = time.time() + timeout_s
    last_error = None
    while time.time() < deadline:
        try:
            if tf_api.can_transform("base_link", frame):
                return read_pose(tf_api, frame)
        except Exception as exc:
            last_error = exc
        time.sleep(0.2)
    raise RuntimeError(
        f"等待 TF 超时: base_link <- {frame}; last_error={last_error}"
    )


def try_read_pose(tf_api, frame) -> Optional[Tuple[list, list]]:
    try:
        if not tf_api.can_transform("base_link", frame):
            return None
        return read_pose(tf_api, frame)
    except Exception:
        return None


def calculate_tcp_from_end(end_position, end_quaternion):
    rotation = quaternion_to_rotation_matrix(end_quaternion)
    offset_in_base = rotate_vector(rotation, TCP_OFFSET_END_M)
    tcp_position = vector_add(end_position, offset_in_base)
    tcp_quaternion = end_quaternion.copy()  # URDF rpy=0 0 0
    return tcp_position, tcp_quaternion, offset_in_base


def load_product_xyz():
    for file_name in PRODUCT_FILES:
        path = Path(file_name)
        if not path.exists():
            continue
        data = json.loads(path.read_text(encoding="utf-8"))
        if "base_xyz" not in data:
            continue
        xyz = [float(value) for value in data["base_xyz"]]
        return file_name, xyz, data
    return None, None, None


def fmt_vector(vector, digits=5):
    if vector is None:
        return "N/A"
    return "[" + ", ".join(f"{value:.{digits}f}" for value in vector) + "]"


def make_snapshot(
    end_position,
    end_quaternion,
    calculated_tcp_position,
    calculated_tcp_quaternion,
    offset_in_base,
    tf_tcp_pose,
    product_file,
    product_xyz,
):
    snapshot = {
        "program": PROGRAM_NAME,
        "read_only": True,
        "urdf_tcp": {
            "parent_frame": END_FRAME,
            "child_frame": TCP_FRAME,
            "offset_xyz_m": TCP_OFFSET_END_M,
            "offset_rpy_rad": [0.0, 0.0, 0.0],
        },
        "arm_r_end_link": {
            "position": end_position,
            "quaternion_xyzw": end_quaternion,
        },
        "calculated_gripper_tcp": {
            "position": calculated_tcp_position,
            "quaternion_xyzw": calculated_tcp_quaternion,
            "offset_vector_in_base": offset_in_base,
        },
        "product_source_file": product_file,
        "product_base_xyz": product_xyz,
    }

    if tf_tcp_pose is not None:
        tf_tcp_position, tf_tcp_quaternion = tf_tcp_pose
        tf_difference = vector_subtract(tf_tcp_position, calculated_tcp_position)
        snapshot["tf_gripper_r_center_link"] = {
            "position": tf_tcp_position,
            "quaternion_xyzw": tf_tcp_quaternion,
        }
        snapshot["tcp_tf_vs_calculated"] = {
            "delta_xyz_m": tf_difference,
            "distance_m": vector_norm(tf_difference),
        }

    if product_xyz is not None:
        tcp_to_product = vector_subtract(product_xyz, calculated_tcp_position)
        snapshot["tcp_to_product"] = {
            "delta_xyz_m": tcp_to_product,
            "distance_m": vector_norm(tcp_to_product),
        }

    return snapshot


def print_status(snapshot):
    end_data = snapshot["arm_r_end_link"]
    tcp_data = snapshot["calculated_gripper_tcp"]

    print("\033[2J\033[H", end="")
    print("=" * 78)
    print(PROGRAM_NAME)
    print("READ ONLY MODE，不控制机械臂和夹爪")
    print("URDF: arm_r_end_link -> gripper_r_center_link")
    print(f"TCP 局部偏移: {fmt_vector(TCP_OFFSET_END_M)} m")
    print("=" * 78)
    print(f"arm_r_end_link XYZ       : {fmt_vector(end_data['position'])} m")
    print(f"arm_r_end_link Quaternion: {fmt_vector(end_data['quaternion_xyzw'], 6)}")
    print(f"TCP 偏移在 base_link 中 : {fmt_vector(tcp_data['offset_vector_in_base'])} m")
    print(f"计算 TCP XYZ             : {fmt_vector(tcp_data['position'])} m")
    print(f"计算 TCP Quaternion      : {fmt_vector(tcp_data['quaternion_xyzw'], 6)}")

    if "tf_gripper_r_center_link" in snapshot:
        tf_data = snapshot["tf_gripper_r_center_link"]
        comparison = snapshot["tcp_tf_vs_calculated"]
        print("-" * 78)
        print(f"TF TCP XYZ               : {fmt_vector(tf_data['position'])} m")
        print(f"TF TCP Quaternion        : {fmt_vector(tf_data['quaternion_xyzw'], 6)}")
        print(f"TF - 计算 TCP 差值       : {fmt_vector(comparison['delta_xyz_m'])} m")
        print(f"TF 与计算 TCP 距离       : {comparison['distance_m']:.6f} m")
    else:
        print("-" * 78)
        print(f"TF 中暂未读取到 {TCP_FRAME}，当前使用 URDF 偏移计算 TCP。")

    print("-" * 78)
    if snapshot.get("product_base_xyz") is not None:
        tcp_product = snapshot["tcp_to_product"]
        print(f"产品来源文件             : {snapshot['product_source_file']}")
        print(f"产品 Base XYZ            : {fmt_vector(snapshot['product_base_xyz'])} m")
        print(f"产品 - TCP 差值          : {fmt_vector(tcp_product['delta_xyz_m'])} m")
        print(f"TCP 到产品三维距离       : {tcp_product['distance_m']:.5f} m")
    else:
        print("未找到 product_xyz_v3.json 或 product_xyz_v2.json。")

    print("=" * 78)
    print("按 S + Enter 保存快照，按 Q + Enter 退出，直接 Enter 刷新。")


def main():
    initialized = False
    try:
        result = agibot_gdk.gdk_init()
        if result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError(f"GDK 初始化失败: {result}")
        initialized = True

        tf_api = agibot_gdk.TF()
        time.sleep(2.0)
        wait_for_pose(tf_api, END_FRAME)

        last_snapshot = None
        while True:
            end_position, end_quaternion = wait_for_pose(tf_api, END_FRAME)
            calculated_tcp_position, calculated_tcp_quaternion, offset_in_base = (
                calculate_tcp_from_end(end_position, end_quaternion)
            )
            tf_tcp_pose = try_read_pose(tf_api, TCP_FRAME)
            product_file, product_xyz, _ = load_product_xyz()

            last_snapshot = make_snapshot(
                end_position=end_position,
                end_quaternion=end_quaternion,
                calculated_tcp_position=calculated_tcp_position,
                calculated_tcp_quaternion=calculated_tcp_quaternion,
                offset_in_base=offset_in_base,
                tf_tcp_pose=tf_tcp_pose,
                product_file=product_file,
                product_xyz=product_xyz,
            )
            print_status(last_snapshot)

            command = input("> ").strip().lower()
            if command in ("q", "quit", "exit"):
                break
            if command == "s":
                Path(SNAPSHOT_FILE).write_text(
                    json.dumps(last_snapshot, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
                print(f"已保存: {SNAPSHOT_FILE}")
                time.sleep(1.0)
            else:
                time.sleep(UPDATE_DT)

    except KeyboardInterrupt:
        print("\n用户中断，程序退出。")
    finally:
        if initialized:
            agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
