#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
53_waypoint2_yolo_oriented_pick_fast_v2.py

基于已验证的53版做头部控制修正：
- 低头方向由 -20 deg 修正为 +20 deg；
- 头部运动后读取 motor_position 闭环验证；
- idx13误差超过1.5 deg时，禁止机械臂进入WAYPOINT_1；
- 其余关节路径、YOLO中心/角度识别、夹爪角度对齐和抓取逻辑沿用53版。

依赖同目录：
- 52_waypoint2_yolo_pick_lift_safe.py
- 53_waypoint2_yolo_oriented_pick_fast.py
"""

import importlib.util
import math
import time
from pathlib import Path

import agibot_gdk

BASE53 = Path(__file__).with_name("53_waypoint2_yolo_oriented_pick_fast.py")
if not BASE53.exists():
    raise RuntimeError(f"缺少依赖程序: {BASE53}")

spec = importlib.util.spec_from_file_location("pick53_base", BASE53)
pick53 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pick53)

# 已由57号独立程序验证：idx13正20度为正确低头位。
HEAD_TARGET_DEG = 20.0
HEAD_SPEED_RAD_S = 0.20
HEAD_POSITION_TOLERANCE_DEG = 1.5
HEAD_SETTLE_SECONDS = 1.0

# 将已验证参数同步给53版的显示与报告逻辑。
pick53.HEAD_TARGET_DEG = HEAD_TARGET_DEG
pick53.HEAD_SPEED_RAD_S = HEAD_SPEED_RAD_S

HEAD_JOINTS = [
    "idx11_head_joint1",
    "idx12_head_joint2",
    "idx13_head_joint3",
]


def get_head_positions(robot):
    states = {state["name"]: state for state in robot.get_joint_states()["states"]}
    missing = [name for name in HEAD_JOINTS if name not in states]
    if missing:
        raise RuntimeError(f"缺少头部关节状态: {missing}")
    return [float(states[name]["motor_position"]) for name in HEAD_JOINTS]


def move_head_down_verified(robot):
    """移动到实测正确的+20度低头位，并通过motor_position闭环验证。"""
    before = get_head_positions(robot)
    target = [
        before[0],
        before[1],
        math.radians(HEAD_TARGET_DEG),
    ]

    print(
        "[头部] 当前角度(deg)="
        f"{[round(math.degrees(value), 3) for value in before]}"
    )
    print(
        "[头部] 目标角度(deg)="
        f"{[round(math.degrees(value), 3) for value in target]}"
    )

    result = robot.move_head_joint(target, [HEAD_SPEED_RAD_S] * 3)
    if result != 0:
        raise RuntimeError(f"头部move_head_joint失败: {result}")

    time.sleep(HEAD_SETTLE_SECONDS)
    after = get_head_positions(robot)
    error_deg = abs(math.degrees(after[2]) - HEAD_TARGET_DEG)

    print(
        "[头部验证] 实际角度(deg)="
        f"{[round(math.degrees(value), 3) for value in after]}"
    )
    print(f"[头部验证] idx13误差={error_deg:.3f}deg")

    if error_deg > HEAD_POSITION_TOLERANCE_DEG:
        raise RuntimeError(
            f"idx13_head_joint3未达到+20度低头位，误差={error_deg:.3f}deg；"
            "禁止继续机械臂动作"
        )

    print("[头部验证] +20度低头位通过，允许进入WAYPOINT路径。")
    return before, target


# 替换53版原有未验证的头部函数，main()其余流程保持不变。
pick53.move_head_down = move_head_down_verified


if __name__ == "__main__":
    pick53.main()
