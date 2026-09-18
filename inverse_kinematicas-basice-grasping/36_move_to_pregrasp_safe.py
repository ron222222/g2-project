#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
36_move_to_pregrasp_safe.py

安全移动右臂到 pregrasp 点。
基于已验证的 G2 接口：Robot.end_effector_pose_control(EndEffectorPose)。

默认 DRY_RUN=True，不会发送任何运动命令。
只有同时满足以下条件才可能运动：
1. 手动将 DRY_RUN 改为 False
2. pregrasp_plan.json 检查通过
3. 实时位姿与计划位姿偏差合理
4. 用户在终端输入 MOVE

本程序不控制夹爪、不下探、不抓取。
"""

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List

import agibot_gdk

# ==================== 安全配置 ====================
DRY_RUN = True
PLAN_FILE = "pregrasp_plan.json"
REPORT_FILE = "move_to_pregrasp_safe_report.json"

LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"
RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
LIFE_TIME = 0.02
MAX_TRANSLATION_STEP_M = 0.001
HOLD_SECONDS = 0.30

# 保守安全阈值
MAX_MOVE_DISTANCE_M = 0.70
MAX_START_POSE_DRIFT_M = 0.05
WORKSPACE_X = (0.20, 1.50)
WORKSPACE_Y = (-0.80, 0.80)
WORKSPACE_Z = (0.20, 1.50)


@dataclass
class PoseData:
    position: List[float]
    orientation: List[float]


def distance(a, b):
    return math.sqrt(sum((y - x) ** 2 for x, y in zip(a, b)))


def lerp(a, b, alpha):
    return [(1.0 - alpha) * x + alpha * y for x, y in zip(a, b)]


def normalize_quaternion(q):
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-9:
        raise ValueError("四元数模长接近0")
    return [v / n for v in q]


def read_pose(tf_api, frame):
    t = tf_api.get_tf_from_base_link(frame)
    return PoseData(
        [float(t.translation.x), float(t.translation.y), float(t.translation.z)],
        normalize_quaternion([
            float(t.rotation.x), float(t.rotation.y),
            float(t.rotation.z), float(t.rotation.w)
        ]),
    )


def workspace_check(xyz):
    return (
        WORKSPACE_X[0] < xyz[0] < WORKSPACE_X[1]
        and WORKSPACE_Y[0] < xyz[1] < WORKSPACE_Y[1]
        and WORKSPACE_Z[0] < xyz[2] < WORKSPACE_Z[1]
    )


def set_both_arm_pose(robot, left, right):
    req = agibot_gdk.EndEffectorPose()
    req.life_time = LIFE_TIME
    req.group = agibot_gdk.EndEffectorControlGroup.kBothArms

    lp = req.left_end_effector_pose.position
    lq = req.left_end_effector_pose.orientation
    rp = req.right_end_effector_pose.position
    rq = req.right_end_effector_pose.orientation

    lp.x, lp.y, lp.z = left.position
    lq.x, lq.y, lq.z, lq.w = left.orientation
    rp.x, rp.y, rp.z = right.position
    rq.x, rq.y, rq.z, rq.w = right.orientation

    result = robot.end_effector_pose_control(req)
    if result != 0:
        raise RuntimeError(f"end_effector_pose_control失败: {result}")


def save_report(report):
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def main():
    if not Path(PLAN_FILE).exists():
        raise RuntimeError(f"未找到 {PLAN_FILE}，请先运行34_yolo_pregrasp_generator.py")

    plan = json.loads(Path(PLAN_FILE).read_text(encoding="utf-8"))
    target = [float(v) for v in plan["pregrasp_xyz"]]
    planned_start = [float(v) for v in plan["arm_r_xyz"]]

    if not workspace_check(target):
        raise RuntimeError(f"目标点未通过工作空间检查: {target}")

    report = {
        "program": "36_move_to_pregrasp_safe.py",
        "dry_run": DRY_RUN,
        "target_pregrasp_xyz": target,
        "planned_start_arm_r_xyz": planned_start,
        "motion_command_sent": False,
        "status": "INITIALIZED",
    }

    init_ok = False
    try:
        result = agibot_gdk.gdk_init()
        if result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError(f"GDK初始化失败: {result}")
        init_ok = True

        robot = agibot_gdk.Robot()
        tf_api = agibot_gdk.TF()
        time.sleep(2.0)

        if not tf_api.can_transform("base_link", LEFT_FRAME):
            raise RuntimeError(f"TF不存在: base_link <- {LEFT_FRAME}")
        if not tf_api.can_transform("base_link", RIGHT_FRAME):
            raise RuntimeError(f"TF不存在: base_link <- {RIGHT_FRAME}")

        left_now = read_pose(tf_api, LEFT_FRAME)
        right_now = read_pose(tf_api, RIGHT_FRAME)
        live_distance = distance(right_now.position, target)
        start_drift = distance(right_now.position, planned_start)
        steps = max(2, int(math.ceil(live_distance / MAX_TRANSLATION_STEP_M)))

        report.update({
            "live_left_pose": {
                "position": left_now.position,
                "orientation": left_now.orientation,
            },
            "live_right_pose": {
                "position": right_now.position,
                "orientation": right_now.orientation,
            },
            "live_distance_m": round(live_distance, 6),
            "start_pose_drift_m": round(start_drift, 6),
            "trajectory_steps": steps,
            "step_m": MAX_TRANSLATION_STEP_M,
            "workspace_check": "PASS",
            "distance_check": "PASS" if live_distance <= MAX_MOVE_DISTANCE_M else "FAIL",
            "start_pose_check": "PASS" if start_drift <= MAX_START_POSE_DRIFT_M else "FAIL",
        })

        print("=" * 68)
        print("36_move_to_pregrasp_safe.py")
        print(f"DRY_RUN = {DRY_RUN}")
        print(f"当前右臂 XYZ: {[round(v, 4) for v in right_now.position]}")
        print(f"目标 PreGrasp XYZ: {[round(v, 4) for v in target]}")
        print(f"实时移动距离: {live_distance:.4f} m")
        print(f"计划起点漂移: {start_drift:.4f} m")
        print(f"插值步数: {steps}, 每步上限: {MAX_TRANSLATION_STEP_M * 1000:.1f} mm")
        print("左臂保持当前位姿，右臂保持当前四元数姿态")
        print("本程序不控制夹爪、不下探、不抓取")
        print("=" * 68)

        if live_distance > MAX_MOVE_DISTANCE_M:
            report["status"] = "BLOCKED_DISTANCE"
            raise RuntimeError("实时移动距离超过安全阈值")
        if start_drift > MAX_START_POSE_DRIFT_M:
            report["status"] = "BLOCKED_START_DRIFT"
            raise RuntimeError("实时右臂位置与计划记录偏差过大，请重新生成pregrasp_plan.json")

        if DRY_RUN:
            report["status"] = "DRY_RUN_PASS"
            print("DRY_RUN通过：未发送任何运动命令。")
            save_report(report)
            return

        confirm = input("真实运动已启用。确认周围安全后输入 MOVE：").strip()
        if confirm != "MOVE":
            report["status"] = "CANCELLED_BY_USER"
            print("未输入MOVE，已取消。")
            save_report(report)
            return

        # 左臂保持当前位姿；右臂位置插值，姿态保持当前四元数。
        for i in range(1, steps + 1):
            alpha = i / steps
            right_cmd = PoseData(
                lerp(right_now.position, target, alpha),
                right_now.orientation.copy(),
            )
            set_both_arm_pose(robot, left_now, right_cmd)
            time.sleep(DT)

        cycles = max(1, int(HOLD_SECONDS * RATE_HZ))
        final_right = PoseData(target, right_now.orientation.copy())
        for _ in range(cycles):
            set_both_arm_pose(robot, left_now, final_right)
            time.sleep(DT)

        report["motion_command_sent"] = True
        report["status"] = "MOVE_COMPLETED"
        print("右臂已完成PreGrasp移动。")
        save_report(report)

    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED"
        report["error"] = "KeyboardInterrupt"
        save_report(report)
        print("用户中断，已停止继续发送命令。")
    except Exception as exc:
        report["status"] = report.get("status", "ERROR")
        report["error"] = str(exc)
        save_report(report)
        raise
    finally:
        if init_ok:
            agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
