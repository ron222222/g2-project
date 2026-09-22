#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
53_waypoint2_yolo_oriented_pick_fast_direct_v2.py

基于原始53的最小改动：
1. 程序开始时头部不动作。
2. HOME -> waypoint_1 一次 move_arm_joint 到位，不分段、不再要求NEXT。
3. waypoint_1验证通过后，头部 idx13 移动到实测正确的 +20度并闭环验证。
4. waypoint_1 -> waypoint_2 一次 move_arm_joint 到位，不分段、不再要求NEXT。
5. 原53后续YOLO、角度规划、角度对齐、XY、下降、夹取、抬升逻辑保持不变。
6. 原53后续高位角度对齐等危险动作仍保留NEXT确认。

依赖同目录：
- 52_waypoint2_yolo_pick_lift_safe.py
- 53_waypoint2_yolo_oriented_pick_fast.py
"""

import importlib.util
import math
import time
from pathlib import Path

BASE53 = Path(__file__).with_name("53_waypoint2_yolo_oriented_pick_fast.py")
if not BASE53.exists():
    raise RuntimeError(f"缺少原始53程序: {BASE53}")

spec = importlib.util.spec_from_file_location("original53", BASE53)
original53 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(original53)
pick52 = original53.pick52

HEAD_TARGET_DEG = 20.0
HEAD_SPEED_RAD_S = 0.20
HEAD_TOLERANCE_DEG = 1.5
HEAD_SETTLE_S = 1.0
DIRECT_ARM_SPEED_RAD_S = 0.20
DIRECT_JOINT_TOLERANCE_RAD = 0.06
DIRECT_TCP_TOLERANCE_M = 0.050

HEAD_JOINTS = [
    "idx11_head_joint1",
    "idx12_head_joint2",
    "idx13_head_joint3",
]

original53.HEAD_TARGET_DEG = HEAD_TARGET_DEG
original53.HEAD_SPEED_RAD_S = HEAD_SPEED_RAD_S
_head_completed = False


def get_head_positions(robot):
    states = {s["name"]: s for s in robot.get_joint_states()["states"]}
    missing = [name for name in HEAD_JOINTS if name not in states]
    if missing:
        raise RuntimeError(f"缺少头部关节状态: {missing}")
    return [float(states[name]["motor_position"]) for name in HEAD_JOINTS]


def head_after_waypoint1(robot, report):
    global _head_completed
    before = get_head_positions(robot)
    target = [before[0], before[1], math.radians(HEAD_TARGET_DEG)]

    print("-" * 78)
    print("[waypoint_1已验证] 执行头部 idx13 = +20度")
    print(f"[头部] 当前(deg)={[round(math.degrees(v), 3) for v in before]}")
    print(f"[头部] 目标(deg)={[round(math.degrees(v), 3) for v in target]}")

    result = robot.move_head_joint(target, [HEAD_SPEED_RAD_S] * 3)
    if result != 0:
        raise RuntimeError(f"move_head_joint失败: {result}")
    time.sleep(HEAD_SETTLE_S)

    after = get_head_positions(robot)
    error_deg = abs(math.degrees(after[2]) - HEAD_TARGET_DEG)
    report["head_after_waypoint_1"] = {
        "before_rad": before,
        "before_deg": [math.degrees(v) for v in before],
        "target_rad": target,
        "target_deg": [math.degrees(v) for v in target],
        "after_rad": after,
        "after_deg": [math.degrees(v) for v in after],
        "idx13_error_deg": error_deg,
    }
    original53.save(report)

    print(f"[头部验证] 实际(deg)={[round(math.degrees(v), 3) for v in after]}")
    print(f"[头部验证] idx13误差={error_deg:.3f}deg")
    if error_deg > HEAD_TOLERANCE_DEG:
        report["status"] = "STOPPED_HEAD_AFTER_WP1_NOT_REACHED"
        original53.save(report)
        raise RuntimeError("头部+20度未到位，禁止继续waypoint_2和YOLO")

    _head_completed = True
    print("[头部验证] +20度低头通过，继续waypoint_2。")
    print("-" * 78)


def direct_waypoint_move(robot, tf_api, label, target, baseline, report):
    """只替换固定Waypoint移动：一次到点且不读取第二次NEXT。"""
    global _head_completed

    pick52.validate_joint_target(label, target)
    pick52.verify_torque(robot, baseline, report)

    # 若程序从waypoint_1附近启动，进入waypoint_2前仍先完成头部动作。
    if label == "waypoint_2" and not _head_completed:
        head_after_waypoint1(robot, report)

    print(f"[{label}] 一次到点，速度={DIRECT_ARM_SPEED_RAD_S:.2f}rad/s")
    result = robot.move_arm_joint(
        list(target),
        [DIRECT_ARM_SPEED_RAD_S] * 7,
        1,
    )
    if result != 0:
        raise RuntimeError(f"{label} move_arm_joint失败: {result}")

    time.sleep(pick52.SETTLE_S)
    actual = pick52.positions(robot, pick52.RIGHT_JOINTS)
    max_error = pick52.max_joint_error(actual, target)
    tcp = pick52.wait_pose(tf_api, pick52.TCP_FRAME)
    tcp_error = pick52.distance(tcp.position, pick52.RECORDED_TCP[label])

    report["joint_stages"].append({
        "stage": label,
        "mode": "single_move_arm_joint_no_stage_prompt",
        "target": list(target),
        "actual": actual,
        "max_joint_error_rad": max_error,
        "actual_tcp": tcp.__dict__,
        "tcp_error_m": tcp_error,
    })
    original53.save(report)

    print(f"[{label}验证] 最大关节误差={max_error:.5f}rad, TCP误差={tcp_error:.5f}m")
    if max_error > DIRECT_JOINT_TOLERANCE_RAD:
        raise RuntimeError(f"{label}关节未到位")
    if tcp_error > DIRECT_TCP_TOLERANCE_M:
        raise RuntimeError(f"{label} TCP未复现")

    if label == "waypoint_1" and not _head_completed:
        head_after_waypoint1(robot, report)


def skip_early_head(robot):
    current = get_head_positions(robot)
    print("[头部] 起始阶段不动作；将在waypoint_1验证通过后低头+20度。")
    return current, current


# 只覆盖原53的两个接入点。
original53.move_head_down = skip_early_head
pick52.segmented_joint_move = direct_waypoint_move


if __name__ == "__main__":
    original53.main()
