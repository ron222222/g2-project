#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
51_right_arm_joint_chain_segmented_v3.py

解决长距离 move_arm_joint() 在 WAYPOINT_2 -> WAYPOINT_3 阶段超时的问题。

核心变化：
1. control_group=1，始终只发送右臂7个位置和7个速度。
2. 每个示教关键点之间按关节最大变化自动拆成多个短目标。
3. 每个短目标最大关节变化默认不超过0.30 rad。
4. 每个短目标完成后读取 motor_position 验证。
5. 仅在示教关键点结束时验证TCP。
6. 默认只执行到WAYPOINT_3，不执行PREGRASP、不控制夹爪、不抓取。

注意：自动短目标是两个VR示教关节姿态之间的线性关节插值点。
第一次真实测试必须有人持急停并观察整个扫掠路径。
"""

import json
import math
import time
from pathlib import Path
from typing import Dict

import agibot_gdk

ENABLE_REAL_MOTION = True
STOP_AFTER_STAGE = 2  # 1=WP1, 2=WP2, 3=WP3, 4=PREGRASP
REPORT_FILE = "right_arm_joint_chain_segmented_v3_report.json"

RIGHT_JOINTS = [
    "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
    "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]
TCP_FRAME = "gripper_r_center_link"

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

ARM_SPEED_RAD_S = 0.12
MAX_DELTA_PER_SUBTARGET_RAD = 0.30
SUBTARGET_JOINT_TOLERANCE_RAD = 0.07
KEYPOINT_TCP_TOLERANCE_M = 0.050
SETTLE_SUBTARGET_S = 0.20
SETTLE_KEYPOINT_S = 0.80
BASELINE_SAMPLES = 30
TORQUE_DELTA_LIMIT_NM = 30.0
REQUIRE_CONFIRM_BEFORE_KEYPOINT = True

JOINT_LIMITS = [
    (-3.071796, 3.071796), (-2.059505, 2.059505),
    (-3.071796, 3.071796), (-2.495838, 1.012308),
    (-3.071796, 3.071796), (-1.012308, 1.012308),
    (-1.535907, 1.535907),
]


def save(report):
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def distance(a, b):
    return math.sqrt(sum((float(b[i]) - float(a[i])) ** 2 for i in range(3)))


def joint_map(robot):
    return {state["name"]: state for state in robot.get_joint_states()["states"]}


def right_positions(robot):
    states = joint_map(robot)
    return [float(states[name]["motor_position"]) for name in RIGHT_JOINTS]


def read_tcp(tf_api):
    t = tf_api.get_tf_from_base_link(TCP_FRAME)
    return [float(t.translation.x), float(t.translation.y), float(t.translation.z)]


def get_torques(robot) -> Dict[str, float]:
    return {state["name"]: float(state["effort"])
            for state in robot.get_joint_states()["states"]}


def baseline(robot):
    samples = []
    print("建立力矩基线，请保持机器人静止...")
    for _ in range(BASELINE_SAMPLES):
        samples.append(get_torques(robot))
        time.sleep(0.05)
    names = set().union(*(sample.keys() for sample in samples))
    return {name: sum(sample.get(name, 0.0) for sample in samples) / len(samples)
            for name in names}


def verify_torque(robot, torque_base, report):
    current = get_torques(robot)
    bad = []
    for name, value in current.items():
        if name in torque_base:
            delta = abs(value - torque_base[name])
            if delta > TORQUE_DELTA_LIMIT_NM:
                bad.append({"joint": name, "delta": delta})
    if bad:
        report["status"] = "STOPPED_TORQUE"
        report["torque_abnormal"] = bad[:10]
        save(report)
        raise RuntimeError("力矩安全检查触发")


def validate_target(label, target):
    if len(target) != 7:
        raise RuntimeError(f"{label}目标不是7个关节")
    for i, (value, limits) in enumerate(zip(target, JOINT_LIMITS)):
        if not limits[0] <= value <= limits[1]:
            raise RuntimeError(
                f"{label}: {RIGHT_JOINTS[i]}={value:.6f}超出限位{limits}"
            )


def interpolate(start, target, alpha):
    return [(1.0-alpha)*a + alpha*b for a, b in zip(start, target)]


def execute_segmented_keypoint(robot, tf_api, label, target, torque_base, report):
    validate_target(label, target)
    start = right_positions(robot)
    deltas = [abs(b-a) for a, b in zip(start, target)]
    max_delta = max(deltas)
    segment_count = max(1, int(math.ceil(max_delta / MAX_DELTA_PER_SUBTARGET_RAD)))

    print("-" * 78)
    print(f"[{label}] 最大关节变化={max_delta:.4f}rad, 自动短目标数={segment_count}")

    if REQUIRE_CONFIRM_BEFORE_KEYPOINT:
        command = input(
            f"准备执行 {label}，共{segment_count}个短目标；确认路径无障碍后输入 NEXT："
        ).strip()
        if command != "NEXT":
            raise RuntimeError(f"用户取消{label}")

    stage_record = {
        "stage": label,
        "start": start,
        "target": target,
        "max_joint_delta_rad": max_delta,
        "segment_count": segment_count,
        "subtargets": [],
    }

    for index in range(1, segment_count + 1):
        verify_torque(robot, torque_base, report)
        alpha = index / segment_count
        subtarget = interpolate(start, target, alpha)
        velocities = [ARM_SPEED_RAD_S] * 7

        print(f"  [{label}] 短目标 {index}/{segment_count}, alpha={alpha:.3f}")
        try:
            result = robot.move_arm_joint(subtarget, velocities, 1)
        except Exception as exc:
            actual = right_positions(robot)
            stage_record["subtargets"].append({
                "index": index,
                "target": subtarget,
                "actual_after_error": actual,
                "error": str(exc),
            })
            report["stages"].append(stage_record)
            report["status"] = f"STOPPED_{label.upper()}_SUBTARGET_{index}"
            save(report)
            raise

        if result != 0:
            raise RuntimeError(f"{label}短目标{index}返回失败: {result}")

        time.sleep(SETTLE_SUBTARGET_S)
        actual = right_positions(robot)
        errors = [abs(a-b) for a, b in zip(actual, subtarget)]
        max_error = max(errors)
        stage_record["subtargets"].append({
            "index": index,
            "alpha": alpha,
            "target": subtarget,
            "actual": actual,
            "max_joint_error_rad": max_error,
        })
        save(report)

        print(f"    最大关节误差={max_error:.5f}rad")
        if max_error > SUBTARGET_JOINT_TOLERANCE_RAD:
            report["stages"].append(stage_record)
            report["status"] = f"STOPPED_{label.upper()}_SUBTARGET_ERROR"
            save(report)
            raise RuntimeError(f"{label}短目标{index}未到位")

    time.sleep(SETTLE_KEYPOINT_S)
    actual_tcp = read_tcp(tf_api)
    tcp_error = distance(actual_tcp, RECORDED_TCP[label])
    stage_record["actual_tcp"] = actual_tcp
    stage_record["recorded_tcp"] = RECORDED_TCP[label]
    stage_record["tcp_error_m"] = tcp_error
    report["stages"].append(stage_record)
    save(report)

    print(f"[{label}关键点验证] TCP误差={tcp_error:.5f}m")
    if tcp_error > KEYPOINT_TCP_TOLERANCE_M:
        report["status"] = f"STOPPED_{label.upper()}_TCP_ERROR"
        save(report)
        raise RuntimeError(f"{label} TCP未复现")


def main():
    if STOP_AFTER_STAGE not in (1, 2, 3, 4):
        raise RuntimeError("STOP_AFTER_STAGE只能为1、2、3或4")

    report = {
        "program": "51_right_arm_joint_chain_segmented_v3.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "stop_after_stage": STOP_AFTER_STAGE,
        "max_delta_per_subtarget_rad": MAX_DELTA_PER_SUBTARGET_RAD,
        "arm_speed_rad_s": ARM_SPEED_RAD_S,
        "stages": [],
        "status": "INITIALIZED",
    }
    initialized = False

    try:
        if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("GDK初始化失败")
        initialized = True
        robot = agibot_gdk.Robot()
        tf_api = agibot_gdk.TF()
        time.sleep(2.0)

        report["initial_right_joints"] = right_positions(robot)
        report["initial_tcp"] = read_tcp(tf_api)
        report["planned_stages"] = [name for name, _ in POSE_CHAIN[:STOP_AFTER_STAGE]]
        save(report)

        print("=" * 78)
        print("51_right_arm_joint_chain_segmented_v3.py")
        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print(f"STOP_AFTER_STAGE = {STOP_AFTER_STAGE}")
        print(f"单个短目标最大关节变化 = {MAX_DELTA_PER_SUBTARGET_RAD:.2f}rad")
        print(f"右臂速度 = {ARM_SPEED_RAD_S:.2f}rad/s")
        print(f"本次关键点 = {[name for name, _ in POSE_CHAIN[:STOP_AFTER_STAGE]]}")
        print("不控制夹爪、不下探、不抓取。")
        print("=" * 78)

        if not ENABLE_REAL_MOTION:
            report["status"] = "DRY_RUN_PASS"
            save(report)
            print("DRY RUN通过，未发送运动命令。")
            return

        if input("确认急停可用、扫掠区域无障碍后输入 MOVE：").strip() != "MOVE":
            report["status"] = "CANCELLED_BY_USER"
            save(report)
            return

        torque_base = baseline(robot)
        for label, target in POSE_CHAIN[:STOP_AFTER_STAGE]:
            execute_segmented_keypoint(robot, tf_api, label, target, torque_base, report)

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
