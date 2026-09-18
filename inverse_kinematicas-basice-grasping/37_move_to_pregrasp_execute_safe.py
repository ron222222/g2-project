#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
37_move_to_pregrasp_execute_safe.py

右臂安全移动到 PreGrasp，左臂保持初始位姿。
基于已验证接口 Robot.end_effector_pose_control(EndEffectorPose)。

安全机制：
1. 默认 ENABLE_REAL_MOTION=False，不运动。
2. 必须存在 pregrasp_plan.json 和 move_to_pregrasp_safe_report.json。
3. 上一阶段报告必须为 DRY_RUN_PASS，且未发送运动命令。
4. 检查实时起点漂移、目标工作空间、最大移动距离。
5. 建立关节力矩基线，运动中检查力矩突变。
6. 1 mm 插值、50 Hz 发送，仅移动右臂位置，保持双臂当前姿态。
7. 真实运动前必须输入 MOVE。
8. 不控制夹爪、不下探、不抓取。
"""

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import agibot_gdk

# 首次下载后保持 False。完成现场安全检查后，才可手动改为 True。
ENABLE_REAL_MOTION = True

PLAN_FILE = "pregrasp_plan.json"
DRY_REPORT_FILE = "move_to_pregrasp_safe_report.json"
EXEC_REPORT_FILE = "move_to_pregrasp_execute_report.json"

LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"
RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
LIFE_TIME = 0.02
MAX_TRANSLATION_STEP_M = 0.001
MAX_MOVE_DISTANCE_M = 0.70
MAX_START_POSE_DRIFT_M = 0.03
HOLD_SECONDS = 0.30
BASELINE_SAMPLES = 30
TORQUE_DELTA_LIMIT_NM = 30.0

WORKSPACE_X = (0.20, 1.50)
WORKSPACE_Y = (-0.80, 0.80)
WORKSPACE_Z = (0.20, 1.50)


@dataclass
class PoseData:
    position: List[float]
    orientation: List[float]


def save_report(report):
    Path(EXEC_REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def distance(a, b):
    return math.sqrt(sum((y - x) ** 2 for x, y in zip(a, b)))


def lerp(a, b, alpha):
    return [(1.0 - alpha) * x + alpha * y for x, y in zip(a, b)]


def normalize_quaternion(q):
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-9:
        raise RuntimeError("四元数模长接近0")
    return [v / n for v in q]


def read_pose(tf_api, frame):
    t = tf_api.get_tf_from_base_link(frame)
    return PoseData(
        [float(t.translation.x), float(t.translation.y), float(t.translation.z)],
        normalize_quaternion([
            float(t.rotation.x), float(t.rotation.y),
            float(t.rotation.z), float(t.rotation.w),
        ]),
    )


def workspace_ok(xyz):
    return (
        WORKSPACE_X[0] < xyz[0] < WORKSPACE_X[1]
        and WORKSPACE_Y[0] < xyz[1] < WORKSPACE_Y[1]
        and WORKSPACE_Z[0] < xyz[2] < WORKSPACE_Z[1]
    )


def get_joint_torques(robot) -> Dict[str, float]:
    raw = robot.get_joint_states()
    return {s["name"]: float(s["effort"]) for s in raw["states"]}


def establish_torque_baseline(robot):
    samples = []
    print("正在建立力矩基线，请保持机器人静止...")
    for i in range(BASELINE_SAMPLES):
        samples.append(get_joint_torques(robot))
        print(f"  基线采样 {i + 1}/{BASELINE_SAMPLES}", end="\r")
        time.sleep(0.05)
    print()
    joints = set().union(*(s.keys() for s in samples))
    baseline = {
        name: sum(s.get(name, 0.0) for s in samples) / len(samples)
        for name in joints
    }
    return baseline


def check_torque_delta(robot, baseline):
    current = get_joint_torques(robot)
    abnormal = []
    for name, value in current.items():
        if name in baseline:
            delta = abs(value - baseline[name])
            if delta > TORQUE_DELTA_LIMIT_NM:
                abnormal.append((name, delta, value, baseline[name]))
    return abnormal


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


def main():
    if not Path(PLAN_FILE).exists():
        raise RuntimeError(f"未找到 {PLAN_FILE}")
    if not Path(DRY_REPORT_FILE).exists():
        raise RuntimeError(f"未找到 {DRY_REPORT_FILE}，请先运行36_move_to_pregrasp_safe.py")

    plan = json.loads(Path(PLAN_FILE).read_text(encoding="utf-8"))
    dry = json.loads(Path(DRY_REPORT_FILE).read_text(encoding="utf-8"))

    if dry.get("status") != "DRY_RUN_PASS":
        raise RuntimeError(f"上一阶段状态不是DRY_RUN_PASS: {dry.get('status')}")
    if dry.get("motion_command_sent") is not False:
        raise RuntimeError("上一阶段报告中的motion_command_sent不是false")

    target = [float(v) for v in plan["pregrasp_xyz"]]
    planned_start = [float(v) for v in plan["arm_r_xyz"]]

    report = {
        "program": "37_move_to_pregrasp_execute_safe.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "target_pregrasp_xyz": target,
        "motion_command_sent": False,
        "status": "INITIALIZED",
    }

    if not workspace_ok(target):
        report["status"] = "BLOCKED_WORKSPACE"
        save_report(report)
        raise RuntimeError(f"目标未通过工作空间检查: {target}")

    initialized = False
    try:
        result = agibot_gdk.gdk_init()
        if result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError(f"GDK初始化失败: {result}")
        initialized = True

        robot = agibot_gdk.Robot()
        tf_api = agibot_gdk.TF()
        time.sleep(2.0)

        if not tf_api.can_transform("base_link", LEFT_FRAME):
            raise RuntimeError(f"TF不存在: base_link <- {LEFT_FRAME}")
        if not tf_api.can_transform("base_link", RIGHT_FRAME):
            raise RuntimeError(f"TF不存在: base_link <- {RIGHT_FRAME}")

        left_start = read_pose(tf_api, LEFT_FRAME)
        right_start = read_pose(tf_api, RIGHT_FRAME)
        live_distance = distance(right_start.position, target)
        start_drift = distance(right_start.position, planned_start)
        steps = max(2, int(math.ceil(live_distance / MAX_TRANSLATION_STEP_M)))

        report.update({
            "live_left_start": left_start.position,
            "live_right_start": right_start.position,
            "live_distance_m": round(live_distance, 6),
            "start_pose_drift_m": round(start_drift, 6),
            "trajectory_steps": steps,
            "workspace_check": "PASS",
            "distance_check": "PASS" if live_distance <= MAX_MOVE_DISTANCE_M else "FAIL",
            "start_pose_check": "PASS" if start_drift <= MAX_START_POSE_DRIFT_M else "FAIL",
        })

        print("=" * 72)
        print("37_move_to_pregrasp_execute_safe.py")
        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print(f"当前右臂 XYZ: {[round(v, 4) for v in right_start.position]}")
        print(f"目标 PreGrasp XYZ: {[round(v, 4) for v in target]}")
        print(f"移动距离: {live_distance:.4f} m")
        print(f"起点漂移: {start_drift:.4f} m")
        print(f"插值步数: {steps}, 频率: {RATE_HZ:.1f} Hz")
        print("左臂保持当前位姿；右臂保持当前四元数姿态")
        print("不控制夹爪、不下探、不抓取")
        print("=" * 72)

        if live_distance > MAX_MOVE_DISTANCE_M:
            report["status"] = "BLOCKED_DISTANCE"
            raise RuntimeError("移动距离超过安全阈值")
        if start_drift > MAX_START_POSE_DRIFT_M:
            report["status"] = "BLOCKED_START_DRIFT"
            raise RuntimeError("实时位置与计划起点偏差过大，请重新执行33和34")

        if not ENABLE_REAL_MOTION:
            report["status"] = "ARMED_BUT_MOTION_DISABLED"
            save_report(report)
            print("真实运动开关为False，未发送任何命令。")
            return

        confirm = input("确认急停可用、工作区无人、路径无障碍后输入 MOVE：").strip()
        if confirm != "MOVE":
            report["status"] = "CANCELLED_BY_USER"
            save_report(report)
            print("未输入MOVE，已取消。")
            return

        baseline = establish_torque_baseline(robot)
        report["torque_baseline_joint_count"] = len(baseline)

        for i in range(1, steps + 1):
            abnormal = check_torque_delta(robot, baseline)
            if abnormal:
                report["status"] = "STOPPED_TORQUE"
                report["torque_abnormal"] = [
                    {"joint": n, "delta": d, "current": c, "baseline": b}
                    for n, d, c, b in abnormal[:10]
                ]
                raise RuntimeError(f"力矩安全检查触发，共{len(abnormal)}个关节异常")

            alpha = i / steps
            right_cmd = PoseData(
                lerp(right_start.position, target, alpha),
                right_start.orientation.copy(),
            )
            set_both_arm_pose(robot, left_start, right_cmd)
            report["motion_command_sent"] = True
            time.sleep(DT)

            if i % 50 == 0 or i == steps:
                print(f"进度: {i}/{steps}")

        final_right = PoseData(target, right_start.orientation.copy())
        for _ in range(max(1, int(HOLD_SECONDS * RATE_HZ))):
            set_both_arm_pose(robot, left_start, final_right)
            time.sleep(DT)

        final_pose = read_pose(tf_api, RIGHT_FRAME)
        final_error = distance(final_pose.position, target)
        report.update({
            "final_right_xyz": final_pose.position,
            "final_position_error_m": round(final_error, 6),
            "status": "MOVE_COMPLETED",
        })
        save_report(report)
        print(f"移动完成，末端位置误差: {final_error:.4f} m")

    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED"
        report["error"] = "KeyboardInterrupt"
        save_report(report)
        print("用户中断，已停止后续命令。")
    except Exception as exc:
        report["error"] = str(exc)
        if report.get("status") == "INITIALIZED":
            report["status"] = "ERROR"
        save_report(report)
        raise
    finally:
        if initialized:
            agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
