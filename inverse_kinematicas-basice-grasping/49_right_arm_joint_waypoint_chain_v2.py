#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
49_right_arm_joint_waypoint_chain_v2.py

修复 move_arm_joint() 的右臂参数长度：
- control_group=1 时只发送右臂7个 positions 和7个 velocities。
- 不再发送“左臂7个 + 右臂7个”的14元素列表。

本程序只验证VR示教的右臂关节路径，不识别产品、不下探、不控制夹爪。
默认 ENABLE_REAL_MOTION=False。
默认 STOP_AFTER_STAGE=1，仅执行WAYPOINT_1后停止；验证通过后再逐步改为2、3、4。
"""

import json
import math
import time
from pathlib import Path
from typing import Dict, List

import agibot_gdk

ENABLE_REAL_MOTION = True
STOP_AFTER_STAGE = 2  # 1=只到WP1；2=到WP2；3=到WP3；4=到PREGRASP
REPORT_FILE = "right_arm_joint_waypoint_chain_v2_report.json"

RIGHT_FRAME = "arm_r_end_link"
TCP_FRAME = "gripper_r_center_link"
RIGHT_JOINTS = [
    "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
    "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]

HOME = [-1.57079643, -1.57079608, 1.57079585, -1.57079645,
        0.00000024, 0.00000000, 0.00000036]
WAYPOINT_1 = [-1.91304851, 0.20393955, 1.83169374, -1.84651811,
              0.29203423, -0.28742294, 0.91643847]
WAYPOINT_2 = [-2.34999969, 0.99999998, 2.39123433, -1.79048179,
              -1.50846242, -0.95681261, -0.11304288]
WAYPOINT_3 = [-2.35000014, 0.99999983, 2.27358603, -1.75903363,
              0.97554659, 0.01765348, 0.59585902]
PREGRASP = [-0.26450533, -1.08659669, 0.16999971, -1.47974414,
            -0.31172467, 0.44504713, 1.37577128]

POSE_CHAIN = [
    ("waypoint_1", WAYPOINT_1),
    ("waypoint_2", WAYPOINT_2),
    ("waypoint_3", WAYPOINT_3),
    ("pregrasp", PREGRASP),
]

RECORDED_TCP = {
    "waypoint_1": [0.58639, -0.34289, 0.93608],
    "waypoint_2": [0.65699, -0.16556, 0.98419],
    "waypoint_3": [0.63205, -0.07514, 0.97981],
    "pregrasp": [0.66409, -0.09406, 0.82812],
}

ARM_SPEED_RAD_S = 0.08
JOINT_TOLERANCE_RAD = 0.06
TCP_TOLERANCE_M = 0.050
BASELINE_SAMPLES = 30
TORQUE_DELTA_LIMIT_NM = 30.0
SETTLE_SECONDS = 1.0

JOINT_LIMITS = [
    (-3.071796, 3.071796),
    (-2.059505, 2.059505),
    (-3.071796, 3.071796),
    (-2.495838, 1.012308),
    (-3.071796, 3.071796),
    (-1.012308, 1.012308),
    (-1.535907, 1.535907),
]


def save(report):
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def distance(a, b):
    return math.sqrt(sum((float(b[i]) - float(a[i])) ** 2 for i in range(3)))


def read_tcp(tf_api):
    t = tf_api.get_tf_from_base_link(TCP_FRAME)
    return {
        "position": [float(t.translation.x), float(t.translation.y), float(t.translation.z)],
        "orientation": [float(t.rotation.x), float(t.rotation.y),
                        float(t.rotation.z), float(t.rotation.w)],
    }


def joint_state_map(robot):
    return {state["name"]: state for state in robot.get_joint_states()["states"]}


def right_positions(robot):
    states = joint_state_map(robot)
    missing = [name for name in RIGHT_JOINTS if name not in states]
    if missing:
        raise RuntimeError(f"缺少右臂关节状态: {missing}")
    return [float(states[name]["motor_position"]) for name in RIGHT_JOINTS]


def get_torques(robot) -> Dict[str, float]:
    return {state["name"]: float(state["effort"])
            for state in robot.get_joint_states()["states"]}


def establish_baseline(robot):
    samples = []
    print("建立力矩基线，请保持机器人静止...")
    for _ in range(BASELINE_SAMPLES):
        samples.append(get_torques(robot))
        time.sleep(0.05)
    names = set().union(*(sample.keys() for sample in samples))
    return {name: sum(sample.get(name, 0.0) for sample in samples) / len(samples)
            for name in names}


def verify_torque(robot, baseline, report):
    current = get_torques(robot)
    abnormal = []
    for name, value in current.items():
        if name in baseline:
            delta = abs(value - baseline[name])
            if delta > TORQUE_DELTA_LIMIT_NM:
                abnormal.append({"joint": name, "delta": delta})
    if abnormal:
        report["status"] = "STOPPED_TORQUE"
        report["torque_abnormal"] = abnormal[:10]
        save(report)
        raise RuntimeError("力矩安全检查触发")


def validate_target(label, target):
    if len(target) != 7:
        raise RuntimeError(f"{label}目标长度不是7: {len(target)}")
    for index, (value, limits) in enumerate(zip(target, JOINT_LIMITS)):
        if not limits[0] <= value <= limits[1]:
            raise RuntimeError(
                f"{label} {RIGHT_JOINTS[index]}={value:.6f}超出限位{limits}"
            )


def move_stage(robot, tf_api, label, target, baseline, report):
    validate_target(label, target)
    verify_torque(robot, baseline, report)

    positions = list(target)                 # 右臂7个
    velocities = [ARM_SPEED_RAD_S] * 7       # 右臂7个
    control_group = 1                        # 右臂

    print("-" * 78)
    print(f"[{label}] positions长度={len(positions)}, velocities长度={len(velocities)}")
    print(f"[{label}] 目标: {[round(v, 5) for v in target]}")

    result = robot.move_arm_joint(positions, velocities, control_group)
    if result != 0:
        raise RuntimeError(f"{label} move_arm_joint返回失败: {result}")

    time.sleep(SETTLE_SECONDS)
    actual = right_positions(robot)
    errors = [abs(a - b) for a, b in zip(actual, target)]
    max_error = max(errors)
    tcp = read_tcp(tf_api)
    tcp_error = distance(tcp["position"], RECORDED_TCP[label])

    record = {
        "stage": label,
        "target_right_joints": target,
        "actual_right_joints": actual,
        "joint_errors_rad": errors,
        "max_joint_error_rad": max_error,
        "actual_tcp": tcp,
        "recorded_tcp_position": RECORDED_TCP[label],
        "tcp_position_error_m": tcp_error,
    }
    report["stages"].append(record)
    save(report)

    print(f"[{label}验证] 最大关节误差={max_error:.4f}rad")
    print(f"[{label}验证] TCP位置误差={tcp_error:.4f}m")

    if max_error > JOINT_TOLERANCE_RAD:
        report["status"] = f"STOPPED_{label.upper()}_JOINT_ERROR"
        save(report)
        raise RuntimeError(f"{label}关节未到位")
    if tcp_error > TCP_TOLERANCE_M:
        report["status"] = f"STOPPED_{label.upper()}_TCP_ERROR"
        save(report)
        raise RuntimeError(f"{label} TCP未复现")


def main():
    if STOP_AFTER_STAGE not in (1, 2, 3, 4):
        raise RuntimeError("STOP_AFTER_STAGE只能是1、2、3或4")

    report = {
        "program": "49_right_arm_joint_waypoint_chain_v2.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "stop_after_stage": STOP_AFTER_STAGE,
        "api_contract": {
            "control_group": 1,
            "positions_length": 7,
            "velocities_length": 7,
        },
        "stages": [],
        "status": "INITIALIZED",
    }

    initialized = False
    try:
        result = agibot_gdk.gdk_init()
        if result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError(f"GDK初始化失败: {result}")
        initialized = True

        robot = agibot_gdk.Robot()
        tf_api = agibot_gdk.TF()
        time.sleep(2.0)

        report["initial_right_joints"] = right_positions(robot)
        report["initial_tcp"] = read_tcp(tf_api)
        report["planned_stages"] = [name for name, _ in POSE_CHAIN[:STOP_AFTER_STAGE]]
        save(report)

        print("=" * 78)
        print("49_right_arm_joint_waypoint_chain_v2.py")
        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print(f"STOP_AFTER_STAGE = {STOP_AFTER_STAGE}")
        print("右臂接口参数: 7个positions + 7个velocities + control_group=1")
        print(f"本次阶段: {[name for name, _ in POSE_CHAIN[:STOP_AFTER_STAGE]]}")
        print("不控制夹爪、不下探、不抓取。")
        print("=" * 78)

        if not ENABLE_REAL_MOTION:
            report["status"] = "DRY_RUN_PASS"
            save(report)
            print("DRY RUN通过，未发送运动命令。")
            return

        if input("确认急停可用、路径无障碍后输入 MOVE：").strip() != "MOVE":
            report["status"] = "CANCELLED_BY_USER"
            save(report)
            return

        baseline = establish_baseline(robot)
        for label, target in POSE_CHAIN[:STOP_AFTER_STAGE]:
            move_stage(robot, tf_api, label, target, baseline, report)

        report["status"] = f"STAGE_{STOP_AFTER_STAGE}_REACHED"
        save(report)
        print("=" * 78)
        print(f"结束状态: {report['status']}")
        print(f"报告文件: {REPORT_FILE}")
        print("=" * 78)

    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED"
        report["error"] = "KeyboardInterrupt"
        save(report)
        print("用户中断，已停止。")
    except Exception as exc:
        if report.get("status") == "INITIALIZED":
            report["status"] = "ERROR"
        report["error"] = str(exc)
        save(report)
        raise
    finally:
        if initialized:
            agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
