#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
47_record_grasp_ready_joint_pose.py

READ ONLY。用于记录人工/VR示教得到的“垂直抓取预备姿态”。
不发送机械臂、底盘或夹爪控制命令。

操作：
1. 使用HMI/VR把右臂调整到安全、自然、夹爪朝下的抓取预备姿态。
2. 确保夹爪TCP位于桌面上方足够安全的位置。
3. 运行本程序。
4. 输入 SAVE 保存关节姿态。

输出：grasp_ready_joint_pose.json
后续程序应先用 move_arm_joint() 进入该已验证关节姿态，
再使用小范围TCP笛卡尔修正完成视觉抓取。
"""

import json
import math
import time
from pathlib import Path

import agibot_gdk

OUTPUT_FILE = "grasp_ready_joint_pose.json"
LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"
TCP_FRAME = "gripper_r_center_link"

LEFT_JOINTS = [
    "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
    "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
]
RIGHT_JOINTS = [
    "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
    "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]

MIN_TCP_Z_M = 0.85
MAX_ABS_JOINT_VELOCITY_RAD_S = 0.03


def normalize_q(q):
    norm = math.sqrt(sum(float(v) ** 2 for v in q))
    if norm < 1e-12:
        raise RuntimeError("四元数模长接近0")
    return [float(v) / norm for v in q]


def read_pose(tf_api, frame):
    t = tf_api.get_tf_from_base_link(frame)
    return {
        "position": [float(t.translation.x), float(t.translation.y), float(t.translation.z)],
        "quaternion_xyzw": normalize_q([
            t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
        ]),
    }


def wait_pose(tf_api, frame, timeout=10.0):
    deadline = time.time() + timeout
    last_error = None
    while time.time() < deadline:
        try:
            if tf_api.can_transform("base_link", frame):
                return read_pose(tf_api, frame)
        except Exception as exc:
            last_error = exc
        time.sleep(0.2)
    raise RuntimeError(f"等待TF超时: {frame}; {last_error}")


def map_joint_states(robot):
    raw = robot.get_joint_states()
    return {state["name"]: state for state in raw["states"]}


def get_arm_values(state_map, names):
    missing = [name for name in names if name not in state_map]
    if missing:
        raise RuntimeError(f"缺少关节状态: {missing}")

    values = []
    for name in names:
        state = state_map[name]
        values.append({
            "name": name,
            "motor_position": float(state["motor_position"]),
            "motor_velocity": float(state["motor_velocity"]),
            "effort": float(state["effort"]),
            "error_code": int(state["error_code"]),
        })
    return values


def main():
    initialized = False
    try:
        result = agibot_gdk.gdk_init()
        if result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError(f"GDK初始化失败: {result}")
        initialized = True

        robot = agibot_gdk.Robot()
        tf_api = agibot_gdk.TF()
        time.sleep(2.0)

        state_map = map_joint_states(robot)
        left = get_arm_values(state_map, LEFT_JOINTS)
        right = get_arm_values(state_map, RIGHT_JOINTS)
        left_pose = wait_pose(tf_api, LEFT_FRAME)
        right_pose = wait_pose(tf_api, RIGHT_FRAME)
        tcp_pose = wait_pose(tf_api, TCP_FRAME)

        max_velocity = max(
            abs(item["motor_velocity"]) for item in left + right
        )
        error_joints = [
            item["name"] for item in left + right if item["error_code"] != 0
        ]

        print("=" * 78)
        print("47_record_grasp_ready_joint_pose.py")
        print("READ ONLY，不控制机器人")
        print(f"右臂 End XYZ : {[round(v, 5) for v in right_pose['position']]}")
        print(f"右臂 TCP XYZ : {[round(v, 5) for v in tcp_pose['position']]}")
        print(f"TCP Quaternion: {[round(v, 6) for v in tcp_pose['quaternion_xyzw']]}")
        print(f"最大关节速度: {max_velocity:.5f} rad/s")
        print(f"关节错误码异常: {error_joints if error_joints else '无'}")
        print("右臂 motor_position:")
        for item in right:
            print(f"  {item['name']}: {item['motor_position']:.8f}")
        print("=" * 78)

        if tcp_pose["position"][2] < MIN_TCP_Z_M:
            raise RuntimeError(
                f"TCP高度{tcp_pose['position'][2]:.4f}m低于记录门限{MIN_TCP_Z_M:.4f}m"
            )
        if max_velocity > MAX_ABS_JOINT_VELOCITY_RAD_S:
            raise RuntimeError("机器人尚未静止，拒绝记录")
        if error_joints:
            raise RuntimeError("存在关节错误码，拒绝记录")

        if input("确认当前是安全的垂直抓取预备姿态，输入 SAVE：").strip() != "SAVE":
            print("未输入SAVE，不保存。")
            return

        data = {
            "program": "47_record_grasp_ready_joint_pose.py",
            "read_only": True,
            "joint_value_source": "motor_position",
            "joint_order": {
                "left": LEFT_JOINTS,
                "right": RIGHT_JOINTS,
                "both": LEFT_JOINTS + RIGHT_JOINTS,
            },
            "left_arm_motor_positions": [item["motor_position"] for item in left],
            "right_arm_motor_positions": [item["motor_position"] for item in right],
            "both_arm_motor_positions": [item["motor_position"] for item in left + right],
            "left_arm_states": left,
            "right_arm_states": right,
            "left_end_pose": left_pose,
            "right_end_pose": right_pose,
            "right_tcp_pose": tcp_pose,
            "checks": {
                "tcp_height_check": "PASS",
                "stationary_check": "PASS",
                "joint_error_check": "PASS",
                "motion_command_sent": False,
            },
        }
        Path(OUTPUT_FILE).write_text(
            json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(f"已保存: {OUTPUT_FILE}")

    finally:
        if initialized:
            agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
