#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
54_head_joint3_down20_verify.py

只测试并验证头部 idx13_head_joint3 低头20度。
不控制机械臂、夹爪或底盘。
默认不运动；改 ENABLE_REAL_MOTION=True 后才发送命令。
"""

import math
import time
import json
from pathlib import Path
import agibot_gdk

ENABLE_REAL_MOTION = True
TARGET_JOINT3_DEG = -20.0
HEAD_SPEED_RAD_S = 0.15
POSITION_TOLERANCE_DEG = 1.5
REPORT_FILE = "head_joint3_down20_verify_report.json"

HEAD_JOINTS = [
    "idx11_head_joint1",
    "idx12_head_joint2",
    "idx13_head_joint3",
]


def get_head_positions(robot):
    states = {s["name"]: s for s in robot.get_joint_states()["states"]}
    missing = [name for name in HEAD_JOINTS if name not in states]
    if missing:
        raise RuntimeError(f"缺少头部关节状态: {missing}")
    return [float(states[name]["motor_position"]) for name in HEAD_JOINTS]


def save(report):
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def main():
    report = {
        "program": "54_head_joint3_down20_verify.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "target_joint3_deg": TARGET_JOINT3_DEG,
        "status": "INITIALIZED",
    }
    initialized = False
    try:
        result = agibot_gdk.gdk_init()
        if result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError(f"GDK初始化失败: {result}")
        initialized = True
        robot = agibot_gdk.Robot()
        time.sleep(2.0)

        before = get_head_positions(robot)
        target = [before[0], before[1], math.radians(TARGET_JOINT3_DEG)]
        report["before_rad"] = before
        report["before_deg"] = [math.degrees(v) for v in before]
        report["target_rad"] = target
        report["target_deg"] = [math.degrees(v) for v in target]
        save(report)

        print("=" * 72)
        print("54_head_joint3_down20_verify.py")
        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print(f"当前头部角度(deg) = {[round(math.degrees(v), 3) for v in before]}")
        print(f"目标头部角度(deg) = {[round(math.degrees(v), 3) for v in target]}")
        print("仅控制头部，不控制机械臂和夹爪。")
        print("=" * 72)

        if not ENABLE_REAL_MOTION:
            report["status"] = "DRY_RUN_PASS"
            save(report)
            print("DRY RUN通过，未发送头部运动命令。")
            return

        if input("确认头部周围无障碍，输入 HEAD：").strip() != "HEAD":
            report["status"] = "CANCELLED_BY_USER"
            save(report)
            return

        result = robot.move_head_joint(target, [HEAD_SPEED_RAD_S] * 3)
        if result != 0:
            raise RuntimeError(f"move_head_joint失败: {result}")

        time.sleep(1.0)
        after = get_head_positions(robot)
        error_deg = abs(math.degrees(after[2] - target[2]))
        report["after_rad"] = after
        report["after_deg"] = [math.degrees(v) for v in after]
        report["joint3_error_deg"] = error_deg

        print(f"运动后角度(deg) = {[round(math.degrees(v), 3) for v in after]}")
        print(f"idx13误差 = {error_deg:.3f}deg")

        if error_deg > POSITION_TOLERANCE_DEG:
            report["status"] = "HEAD_TARGET_NOT_REACHED"
            save(report)
            raise RuntimeError("idx13_head_joint3未达到-20度")

        report["status"] = "HEAD_TARGET_REACHED"
        save(report)
        print("头部低头20度验证通过。")

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
