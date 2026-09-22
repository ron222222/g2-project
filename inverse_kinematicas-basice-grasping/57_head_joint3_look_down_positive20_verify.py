#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
57_head_joint3_look_down_positive20_verify.py

根据实测重新定义头部低头方向：
人工调整并能拍到产品时，idx13_head_joint3 实测为 +20.019 deg。
因此本程序目标改为 +20 deg，不再尝试 -20 deg。

只控制头部，不控制机械臂、夹爪或底盘。
"""

import json
import math
import time
from pathlib import Path

import agibot_gdk

ENABLE_REAL_MOTION = True
LOOK_DOWN_TARGET_DEG = 20.0
HEAD_SPEED_RAD_S = 0.20
POSITION_TOLERANCE_DEG = 1.5
REPORT_FILE = "head_joint3_look_down_positive20_report.json"
HEAD_JOINTS = [
    "idx11_head_joint1",
    "idx12_head_joint2",
    "idx13_head_joint3",
]


def save(report):
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def get_positions(robot):
    states = {s["name"]: s for s in robot.get_joint_states()["states"]}
    missing = [name for name in HEAD_JOINTS if name not in states]
    if missing:
        raise RuntimeError(f"缺少头部关节状态: {missing}")
    return [float(states[name]["motor_position"]) for name in HEAD_JOINTS]


def main():
    report = {
        "program": "57_head_joint3_look_down_positive20_verify.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "look_down_target_deg": LOOK_DOWN_TARGET_DEG,
        "status": "INITIALIZED",
    }
    initialized = False
    try:
        if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("GDK初始化失败")
        initialized = True
        robot = agibot_gdk.Robot()
        time.sleep(2.0)

        before = get_positions(robot)
        target = [before[0], before[1], math.radians(LOOK_DOWN_TARGET_DEG)]
        report["before_rad"] = before
        report["before_deg"] = [math.degrees(v) for v in before]
        report["target_rad"] = target
        report["target_deg"] = [math.degrees(v) for v in target]
        save(report)

        print("=" * 76)
        print("57_head_joint3_look_down_positive20_verify.py")
        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print(f"当前头部角度(deg) = {[round(math.degrees(v), 3) for v in before]}")
        print(f"低头目标角度(deg) = {[round(math.degrees(v), 3) for v in target]}")
        print("实测约定：idx13正20度为能看到产品的低头方向。")
        print("仅控制头部，不控制机械臂、夹爪或底盘。")
        print("=" * 76)

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
        after = get_positions(robot)
        error_deg = abs(math.degrees(after[2]) - LOOK_DOWN_TARGET_DEG)
        report["after_rad"] = after
        report["after_deg"] = [math.degrees(v) for v in after]
        report["joint3_error_deg"] = error_deg

        print(f"运动后头部角度(deg) = {[round(math.degrees(v), 3) for v in after]}")
        print(f"idx13误差 = {error_deg:.3f}deg")

        if error_deg > POSITION_TOLERANCE_DEG:
            report["status"] = "LOOK_DOWN_TARGET_NOT_REACHED"
            save(report)
            raise RuntimeError("idx13_head_joint3未达到正20度低头位")

        report["status"] = "LOOK_DOWN_TARGET_REACHED"
        save(report)
        print("头部正20度低头位验证通过。")

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
