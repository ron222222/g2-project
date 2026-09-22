#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
55_head_joint3_down20_segmented_verify.py

只控制头部 idx13_head_joint3 到 -20 度，并进行分段闭环验证。
不控制机械臂、夹爪或底盘。

修正54版一次移动只到约-1度的问题：
- 将当前角度到目标角度拆成最多10度/段；
- 每段使用0.30 rad/s；
- 每段完成后读取motor_position；
- 若未到位，最多补发2次该段目标；
- 最终误差必须小于1.5度。
"""

import json
import math
import time
from pathlib import Path

import agibot_gdk

ENABLE_REAL_MOTION = True
TARGET_JOINT3_DEG = -20.0
HEAD_SPEED_RAD_S = 0.30
MAX_STEP_DEG = 10.0
SEGMENT_TOLERANCE_DEG = 2.0
FINAL_TOLERANCE_DEG = 1.5
MAX_RETRIES_PER_SEGMENT = 2
SETTLE_SECONDS = 0.5
REPORT_FILE = "head_joint3_down20_segmented_verify_report.json"

HEAD_JOINTS = [
    "idx11_head_joint1",
    "idx12_head_joint2",
    "idx13_head_joint3",
]


def save(report):
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def get_head_positions(robot):
    states = {state["name"]: state for state in robot.get_joint_states()["states"]}
    missing = [name for name in HEAD_JOINTS if name not in states]
    if missing:
        raise RuntimeError(f"缺少头部关节状态: {missing}")
    return [float(states[name]["motor_position"]) for name in HEAD_JOINTS]


def deg(values):
    return [math.degrees(value) for value in values]


def main():
    report = {
        "program": "55_head_joint3_down20_segmented_verify.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "target_joint3_deg": TARGET_JOINT3_DEG,
        "segments": [],
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
        start_deg = math.degrees(before[2])
        total_delta_deg = TARGET_JOINT3_DEG - start_deg
        segment_count = max(1, int(math.ceil(abs(total_delta_deg) / MAX_STEP_DEG)))

        report.update({
            "before_rad": before,
            "before_deg": deg(before),
            "total_delta_deg": total_delta_deg,
            "segment_count": segment_count,
        })
        save(report)

        print("=" * 76)
        print("55_head_joint3_down20_segmented_verify.py")
        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print(f"当前头部角度(deg) = {[round(v, 3) for v in deg(before)]}")
        print(f"idx13目标 = {TARGET_JOINT3_DEG:.3f}deg")
        print(f"总变化 = {total_delta_deg:.3f}deg，分段数 = {segment_count}")
        print(f"每段最大变化 = {MAX_STEP_DEG:.1f}deg，速度 = {HEAD_SPEED_RAD_S:.2f}rad/s")
        print("只控制头部；不控制机械臂、夹爪和底盘。")
        print("=" * 76)

        if not ENABLE_REAL_MOTION:
            report["status"] = "DRY_RUN_PASS"
            save(report)
            print("DRY RUN通过，未发送头部命令。")
            return

        if input("确认头部周围无障碍，输入 HEAD：").strip() != "HEAD":
            report["status"] = "CANCELLED_BY_USER"
            save(report)
            return

        hold_joint1 = before[0]
        hold_joint2 = before[1]

        for segment_index in range(1, segment_count + 1):
            alpha = segment_index / segment_count
            segment_target_deg = start_deg + alpha * total_delta_deg
            segment_target = [
                hold_joint1,
                hold_joint2,
                math.radians(segment_target_deg),
            ]

            reached = False
            segment_record = {
                "segment": segment_index,
                "target_deg": [math.degrees(v) for v in segment_target],
                "attempts": [],
            }

            for attempt_index in range(1, MAX_RETRIES_PER_SEGMENT + 2):
                result = robot.move_head_joint(
                    segment_target,
                    [HEAD_SPEED_RAD_S] * 3,
                )
                if result != 0:
                    raise RuntimeError(
                        f"第{segment_index}段第{attempt_index}次move_head_joint失败: {result}"
                    )

                time.sleep(SETTLE_SECONDS)
                actual = get_head_positions(robot)
                error_deg = abs(math.degrees(actual[2]) - segment_target_deg)
                segment_record["attempts"].append({
                    "attempt": attempt_index,
                    "actual_rad": actual,
                    "actual_deg": deg(actual),
                    "joint3_error_deg": error_deg,
                })
                save(report)

                print(
                    f"[段{segment_index}/{segment_count} 尝试{attempt_index}] "
                    f"目标={segment_target_deg:.3f}deg，"
                    f"实际={math.degrees(actual[2]):.3f}deg，"
                    f"误差={error_deg:.3f}deg"
                )

                if error_deg <= SEGMENT_TOLERANCE_DEG:
                    reached = True
                    break

            segment_record["reached"] = reached
            report["segments"].append(segment_record)
            save(report)

            if not reached:
                report["status"] = f"SEGMENT_{segment_index}_NOT_REACHED"
                save(report)
                raise RuntimeError(f"头部第{segment_index}段未到位")

        after = get_head_positions(robot)
        final_error_deg = abs(math.degrees(after[2]) - TARGET_JOINT3_DEG)
        report["after_rad"] = after
        report["after_deg"] = deg(after)
        report["final_error_deg"] = final_error_deg

        print("-" * 76)
        print(f"最终头部角度(deg) = {[round(v, 3) for v in deg(after)]}")
        print(f"idx13最终误差 = {final_error_deg:.3f}deg")

        if final_error_deg > FINAL_TOLERANCE_DEG:
            report["status"] = "FINAL_TARGET_NOT_REACHED"
            save(report)
            raise RuntimeError("idx13_head_joint3最终未达到-20度")

        report["status"] = "HEAD_TARGET_REACHED"
        save(report)
        print("头部idx13分段低头20度验证通过。")

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
