#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
42_tcp_pregrasp_pose_planner.py

READ ONLY，不发送任何机器人运动或夹爪命令。

基于：
- 产品坐标 product_xyz_v3.json / product_xyz_v2.json
- URDF 固定变换 arm_r_end_link -> gripper_r_center_link
  xyz = [0, 0, 0.14308] m, rpy = [0, 0, 0]

规划：
1. 将 gripper_r_center_link 作为 TCP。
2. 生成产品上方的 TCP PreGrasp 目标。
3. 令 TCP 局部 +Z 轴朝向 base_link -Z，夹爪朝下。
4. 反算 arm_r_end_link 的目标位姿：
   T_base_end = T_base_tcp * inverse(T_end_tcp)
5. 保存 tcp_pregrasp_plan.json。

默认采用朝下姿态：局部 +Z -> base -Z，局部 +X -> base +X。
对应旋转矩阵 diag(1, -1, -1)，四元数 xyzw = [1, 0, 0, 0]。
"""

import json
import math
from pathlib import Path

PROGRAM_NAME = "42_tcp_pregrasp_pose_planner.py"
OUTPUT_FILE = "tcp_pregrasp_plan.json"
PRODUCT_FILES = ["product_xyz_v3.json", "product_xyz_v2.json"]

# URDF: arm_r_end_link -> gripper_r_center_link
TCP_OFFSET_END_M = [0.0, 0.0, 0.14308]
TCP_OFFSET_RPY_RAD = [0.0, 0.0, 0.0]

# TCP 抓取中心位于产品上方的安全高度
TCP_PREGRASP_HEIGHT_M = 0.150

# 朝下姿态：TCP local +Z 指向 base_link -Z。
# 这里选择绕 base_link/TCP X 轴旋转 180°。
TCP_DOWN_QUATERNION_XYZW = [1.0, 0.0, 0.0, 0.0]

# 保守的软件工作空间，仅作规划检查
END_WORKSPACE_X = (0.20, 1.20)
END_WORKSPACE_Y = (-0.80, 0.80)
END_WORKSPACE_Z = (0.20, 1.50)
TCP_WORKSPACE_X = (0.20, 1.20)
TCP_WORKSPACE_Y = (-0.80, 0.80)
TCP_WORKSPACE_Z = (0.20, 1.50)

MIN_PRODUCT_CONFIDENCE = 0.50


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
        sum(rotation[row][column] * float(vector[column]) for column in range(3))
        for row in range(3)
    ]


def vector_add(a, b):
    return [float(a[index]) + float(b[index]) for index in range(3)]


def vector_subtract(a, b):
    return [float(a[index]) - float(b[index]) for index in range(3)]


def vector_norm(vector):
    return math.sqrt(sum(float(value) ** 2 for value in vector))


def workspace_ok(xyz, limits):
    return (
        limits[0][0] < xyz[0] < limits[0][1]
        and limits[1][0] < xyz[1] < limits[1][1]
        and limits[2][0] < xyz[2] < limits[2][1]
    )


def load_product():
    for file_name in PRODUCT_FILES:
        path = Path(file_name)
        if not path.exists():
            continue
        data = json.loads(path.read_text(encoding="utf-8"))
        if "base_xyz" not in data:
            continue
        product_xyz = [float(value) for value in data["base_xyz"]]
        confidence = data.get("confidence")
        confidence = float(confidence) if confidence is not None else None
        return file_name, product_xyz, confidence, data
    raise RuntimeError(
        "未找到包含 base_xyz 的 product_xyz_v3.json 或 product_xyz_v2.json"
    )


def calculate_end_target_from_tcp(tcp_position, tcp_quaternion):
    # URDF 中 T_end_tcp 只有平移且无旋转。
    # p_tcp = p_end + R_base_end * offset_end_tcp
    # 因为 R_base_end = R_base_tcp，所以：
    # p_end = p_tcp - R_base_tcp * offset_end_tcp
    rotation_base_tcp = quaternion_to_rotation_matrix(tcp_quaternion)
    offset_in_base = rotate_vector(rotation_base_tcp, TCP_OFFSET_END_M)
    end_position = vector_subtract(tcp_position, offset_in_base)
    end_quaternion = normalize_quaternion(tcp_quaternion)
    return end_position, end_quaternion, offset_in_base


def main():
    product_file, product_xyz, confidence, product_data = load_product()

    tcp_target_position = [
        product_xyz[0],
        product_xyz[1],
        product_xyz[2] + TCP_PREGRASP_HEIGHT_M,
    ]
    tcp_target_quaternion = normalize_quaternion(TCP_DOWN_QUATERNION_XYZW)

    end_target_position, end_target_quaternion, offset_in_base = (
        calculate_end_target_from_tcp(
            tcp_position=tcp_target_position,
            tcp_quaternion=tcp_target_quaternion,
        )
    )

    reconstructed_tcp = vector_add(end_target_position, offset_in_base)
    reconstruction_error = vector_norm(
        vector_subtract(reconstructed_tcp, tcp_target_position)
    )

    tcp_workspace_pass = workspace_ok(
        tcp_target_position,
        (TCP_WORKSPACE_X, TCP_WORKSPACE_Y, TCP_WORKSPACE_Z),
    )
    end_workspace_pass = workspace_ok(
        end_target_position,
        (END_WORKSPACE_X, END_WORKSPACE_Y, END_WORKSPACE_Z),
    )
    confidence_pass = (
        confidence is None or confidence >= MIN_PRODUCT_CONFIDENCE
    )

    plan = {
        "program": PROGRAM_NAME,
        "read_only": True,
        "source_file": product_file,
        "product_detection": {
            "confidence": confidence,
            "minimum_confidence": MIN_PRODUCT_CONFIDENCE,
            "confidence_check": "PASS" if confidence_pass else "WARNING",
            "base_xyz": product_xyz,
        },
        "urdf_tcp": {
            "parent_frame": "arm_r_end_link",
            "child_frame": "gripper_r_center_link",
            "offset_xyz_m": TCP_OFFSET_END_M,
            "offset_rpy_rad": TCP_OFFSET_RPY_RAD,
        },
        "tcp_pregrasp_target": {
            "frame": "gripper_r_center_link",
            "position": tcp_target_position,
            "quaternion_xyzw": tcp_target_quaternion,
            "height_above_product_m": TCP_PREGRASP_HEIGHT_M,
            "orientation_description": (
                "TCP local +Z points toward base_link -Z; "
                "TCP local +X points toward base_link +X"
            ),
        },
        "arm_r_end_link_target": {
            "position": end_target_position,
            "quaternion_xyzw": end_target_quaternion,
        },
        "tcp_offset_in_base_at_target": offset_in_base,
        "validation": {
            "tcp_workspace_check": "PASS" if tcp_workspace_pass else "FAIL",
            "end_workspace_check": "PASS" if end_workspace_pass else "FAIL",
            "tcp_reconstruction_error_m": reconstruction_error,
            "motion_command_sent": False,
        },
        "source_product_data": product_data,
    }

    Path(OUTPUT_FILE).write_text(
        json.dumps(plan, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    print("=" * 78)
    print(PROGRAM_NAME)
    print("READ ONLY MODE，不控制机械臂和夹爪")
    print("=" * 78)
    print(f"产品来源文件             : {product_file}")
    print(f"产品置信度               : {confidence}")
    print(f"产品 Base XYZ            : {[round(v, 5) for v in product_xyz]}")
    print("-" * 78)
    print(f"TCP PreGrasp XYZ         : {[round(v, 5) for v in tcp_target_position]}")
    print(f"TCP 目标 Quaternion      : {[round(v, 6) for v in tcp_target_quaternion]}")
    print(f"TCP 偏移在 base_link 中 : {[round(v, 5) for v in offset_in_base]}")
    print("-" * 78)
    print(f"arm_r_end_link 目标 XYZ  : {[round(v, 5) for v in end_target_position]}")
    print(f"arm_r_end_link Quaternion: {[round(v, 6) for v in end_target_quaternion]}")
    print("-" * 78)
    print(f"检测置信度检查           : {'PASS' if confidence_pass else 'WARNING'}")
    print(f"TCP 工作空间检查         : {'PASS' if tcp_workspace_pass else 'FAIL'}")
    print(f"End 工作空间检查         : {'PASS' if end_workspace_pass else 'FAIL'}")
    print(f"TCP 反算重建误差         : {reconstruction_error:.9f} m")
    print(f"输出文件                 : {OUTPUT_FILE}")
    print("=" * 78)

    if not tcp_workspace_pass or not end_workspace_pass:
        raise RuntimeError("规划结果未通过工作空间检查，请勿用于运动")


if __name__ == "__main__":
    main()
