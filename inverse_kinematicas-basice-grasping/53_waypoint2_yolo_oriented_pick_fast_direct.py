#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
53_waypoint2_yolo_oriented_pick_fast_direct.py

对原始53程序做最小定点修改：
1. 不在程序开始时移动头部。
2. WAYPOINT_1 一次 move_arm_joint() 到位，不再拆成4步。
3. WAYPOINT_1 到位并通过TCP验证后，idx13_head_joint3 移动到实测正确的 +20度。
4. 头部通过 motor_position 验证后，才允许继续 WAYPOINT_2。
5. WAYPOINT_2 同样一次 move_arm_joint() 到位，不再拆步。
6. 原53的YOLO位置/角度识别、角度对齐、XY、下降、夹取、抬升逻辑保持原样。

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

# ==================== 仅修改这些参数 ====================
HEAD_TARGET_DEG = 20.0
HEAD_SPEED_RAD_S = 0.20
HEAD_TOLERANCE_DEG = 1.5
HEAD_SETTLE_S = 1.0

# 一次到点的速度。此前0.22 rad/s已验证分段可到位；单次路径先用0.20更稳妥。
DIRECT_ARM_SPEED_RAD_S = 0.20
DIRECT_JOINT_TOLERANCE_RAD = 0.06
DIRECT_TCP_TOLERANCE_M = 0.050

HEAD_JOINTS = [
    "idx11_head_joint1",
    "idx12_head_joint2",
    "idx13_head_joint3",
]

# 同步原53终端显示。
original53.HEAD_TARGET_DEG = HEAD_TARGET_DEG
original53.HEAD_SPEED_RAD_S = HEAD_SPEED_RAD_S

# 保存原53工具。
pick52 = original53.pick52
_head_completed = False


def get_head_positions(robot):
    states = {s["name"]: s for s in robot.get_joint_states()["states"]}
    missing = [name for name in HEAD_JOINTS if name not in states]
    if missing:
        raise RuntimeError(f"缺少头部关节状态: {missing}")
    return [float(states[name]["motor_position"]) for name in HEAD_JOINTS]


def head_positive20_after_wp1(robot, report):
    global _head_completed
    before = get_head_positions(robot)
    target = [before[0], before[1], math.radians(HEAD_TARGET_DEG)]

    print("-" * 78)
    print("[waypoint_1已到位] 现在执行头部 idx13 = +20度")
    print(f"[头部] 当前(deg)={[round(math.degrees(v),3) for v in before]}")
    print(f"[头部] 目标(deg)={[round(math.degrees(v),3) for v in target]}")

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

    print(f"[头部验证] 实际(deg)={[round(math.degrees(v),3) for v in after]}")
    print(f"[头部验证] idx13误差={error_deg:.3f}deg")
    if error_deg > HEAD_TOLERANCE_DEG:
        report["status"] = "STOPPED_HEAD_AFTER_WP1_NOT_REACHED"
        original53.save(report)
        raise RuntimeError("头部+20度未到位，禁止继续WAYPOINT_2和YOLO")

    _head_completed = True
    print("[头部验证] +20度低头通过，继续WAYPOINT_2。")
    print("-" * 78)


def direct_joint_move(robot, tf_api, label, target, baseline, report):
    """替换原53分段函数：每个Waypoint只调用一次move_arm_joint。"""
    global _head_completed

    pick52.validate_joint_target(label, target)
    pick52.verify_torque(robot, baseline, report)

    # 从waypoint_1状态直接启动程序时，在进入WP2前补做头部动作。
    if label == "waypoint_2" and not _head_completed:
        head_positive20_after_wp1(robot, report)

    if original53.REQUIRE_STAGE_CONFIRMATION:
        if input(f"准备一次到达 {label}，确认路径无障碍后输入 NEXT：").strip() != "NEXT":
            raise RuntimeError(f"用户取消{label}")

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
        "mode": "single_move_arm_joint",
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

    # 关键要求：必须是WP1到位和验证完成之后，头部才动作。
    if label == "waypoint_1" and not _head_completed:
        head_positive20_after_wp1(robot, report)


def disable_original_early_head(robot):
    """原53 main仍会调用move_head_down；这里明确改为只读，不发送命令。"""
    current = get_head_positions(robot)
    print("[头部] 程序起始阶段不动作；将在WAYPOINT_1验证通过后低头+20度。")
    return current, current


# 只替换两个调用点，原53其余逻辑完全保留。
original53.move_head_down = disable_original_early_head
pick52.segmented_joint_move = direct_joint_move


if __name__ == "__main__":
    original53.main()
