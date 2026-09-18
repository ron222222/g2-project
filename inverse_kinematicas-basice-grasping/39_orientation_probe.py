#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
39_orientation_probe.py

右臂末端姿态探测程序。
目的：在当前 XYZ 附近，以很小的姿态变化测试 G2 末端位姿控制的姿态跟踪能力。

默认 ENABLE_REAL_MOTION=False，只生成候选姿态和报告，不发送运动命令。
真实测试模式下：
- 左臂保持启动位姿
- 右臂位置保持启动 XYZ
- 分别测试绕 X/Y/Z 轴 +/-5 degree 的小姿态变化
- 每个测试完成后读取 arm_r_end_link，记录位置误差和姿态误差
- 每个测试后恢复初始姿态
- 不控制夹爪、不下探、不抓取

注意：本程序不是“放松姿态约束”的 IK 接口，因为当前 GDK 接口仍要求完整四元数。
本程序用于找出哪些小姿态方向可以被控制器稳定跟踪。
"""

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List

import agibot_gdk

# ==================== 安全配置 ====================
ENABLE_REAL_MOTION = True
REPORT_FILE = "orientation_probe_report.json"

LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"

RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
LIFE_TIME = 0.02
PROBE_ANGLE_DEG = 5.0
ORIENTATION_STEPS = 50
HOLD_SECONDS = 0.30
TF_SETTLE_SECONDS = 0.50

# 姿态恢复误差超过该值时停止后续测试
MAX_RECOVERY_POSITION_ERROR_M = 0.020
MAX_RECOVERY_ANGLE_ERROR_DEG = 5.0


@dataclass
class PoseData:
    position: List[float]
    orientation: List[float]


def save_report(report):
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def normalize_quaternion(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-9:
        raise RuntimeError("四元数模长接近0")
    return [v / norm for v in q]


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return normalize_quaternion([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def axis_angle_quaternion(axis, angle_deg):
    angle_rad = math.radians(angle_deg)
    half = angle_rad / 2.0
    s = math.sin(half)
    if axis == "x":
        return [s, 0.0, 0.0, math.cos(half)]
    if axis == "y":
        return [0.0, s, 0.0, math.cos(half)]
    if axis == "z":
        return [0.0, 0.0, s, math.cos(half)]
    raise ValueError(f"未知轴: {axis}")


def slerp(q0, q1, alpha):
    q0 = normalize_quaternion(q0)
    q1 = normalize_quaternion(q1)
    dot = sum(a * b for a, b in zip(q0, q1))

    if dot < 0.0:
        q1 = [-v for v in q1]
        dot = -dot

    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return normalize_quaternion([
            (1.0 - alpha) * a + alpha * b
            for a, b in zip(q0, q1)
        ])

    theta0 = math.acos(dot)
    sin_theta0 = math.sin(theta0)
    theta = theta0 * alpha
    s0 = math.cos(theta) - dot * math.sin(theta) / sin_theta0
    s1 = math.sin(theta) / sin_theta0
    return normalize_quaternion([
        s0 * a + s1 * b
        for a, b in zip(q0, q1)
    ])


def position_distance(a, b):
    return math.sqrt(sum((float(y) - float(x)) ** 2 for x, y in zip(a, b)))


def quaternion_angle_error_deg(q1, q2):
    q1 = normalize_quaternion(q1)
    q2 = normalize_quaternion(q2)
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def read_pose(tf_api, frame):
    t = tf_api.get_tf_from_base_link(frame)
    return PoseData(
        position=[
            float(t.translation.x),
            float(t.translation.y),
            float(t.translation.z),
        ],
        orientation=normalize_quaternion([
            float(t.rotation.x),
            float(t.rotation.y),
            float(t.rotation.z),
            float(t.rotation.w),
        ]),
    )


def wait_for_pose(tf_api, frame, timeout_s=10.0):
    deadline = time.time() + timeout_s
    last_error = None
    while time.time() < deadline:
        try:
            if tf_api.can_transform("base_link", frame):
                return read_pose(tf_api, frame)
        except Exception as exc:
            last_error = exc
        time.sleep(0.2)
    raise RuntimeError(
        f"等待TF超时: base_link <- {frame}; last_error={last_error}"
    )


def set_both_arm_pose(robot, left_pose, right_pose):
    request = agibot_gdk.EndEffectorPose()
    request.life_time = LIFE_TIME
    request.group = agibot_gdk.EndEffectorControlGroup.kBothArms

    lp = request.left_end_effector_pose.position
    lq = request.left_end_effector_pose.orientation
    rp = request.right_end_effector_pose.position
    rq = request.right_end_effector_pose.orientation

    lp.x, lp.y, lp.z = left_pose.position
    lq.x, lq.y, lq.z, lq.w = left_pose.orientation
    rp.x, rp.y, rp.z = right_pose.position
    rq.x, rq.y, rq.z, rq.w = right_pose.orientation

    result = robot.end_effector_pose_control(request)
    if result != 0:
        raise RuntimeError(f"end_effector_pose_control失败: {result}")


def move_orientation(robot, left_hold, right_position, q_start, q_goal):
    for step in range(1, ORIENTATION_STEPS + 1):
        alpha = step / ORIENTATION_STEPS
        command = PoseData(
            position=right_position.copy(),
            orientation=slerp(q_start, q_goal, alpha),
        )
        set_both_arm_pose(robot, left_hold, command)
        time.sleep(DT)

    final_command = PoseData(
        position=right_position.copy(),
        orientation=q_goal.copy(),
    )
    for _ in range(max(1, int(HOLD_SECONDS * RATE_HZ))):
        set_both_arm_pose(robot, left_hold, final_command)
        time.sleep(DT)


def make_candidates(q_initial):
    candidates = []
    for axis in ("x", "y", "z"):
        for sign in (-1.0, 1.0):
            angle = sign * PROBE_ANGLE_DEG
            delta_q = axis_angle_quaternion(axis, angle)
            # 右乘表示相对于当前末端局部坐标系施加小旋转。
            q_target = quaternion_multiply(q_initial, delta_q)
            candidates.append({
                "name": f"local_{axis}_{angle:+.1f}deg",
                "axis": axis,
                "angle_deg": angle,
                "target_quaternion": q_target,
            })
    return candidates


def main():
    report = {
        "program": "39_orientation_probe.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "probe_angle_deg": PROBE_ANGLE_DEG,
        "motion_command_sent": False,
        "tests": [],
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

        left_initial = wait_for_pose(tf_api, LEFT_FRAME)
        right_initial = wait_for_pose(tf_api, RIGHT_FRAME)
        candidates = make_candidates(right_initial.orientation)

        report.update({
            "initial_left_pose": {
                "position": left_initial.position,
                "orientation": left_initial.orientation,
            },
            "initial_right_pose": {
                "position": right_initial.position,
                "orientation": right_initial.orientation,
            },
            "candidates": candidates,
        })

        print("=" * 76)
        print("39_orientation_probe.py")
        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print(f"当前右臂 XYZ: {[round(v, 5) for v in right_initial.position]}")
        print(
            "当前右臂 Quaternion: "
            f"{[round(v, 6) for v in right_initial.orientation]}"
        )
        print(f"姿态探测角度: +/-{PROBE_ANGLE_DEG:.1f} degree")
        print("右臂 XYZ 保持当前值，仅测试小姿态变化")
        print("不控制夹爪、不下探、不抓取")
        print("=" * 76)

        for candidate in candidates:
            print(
                f"候选 {candidate['name']}: "
                f"{[round(v, 6) for v in candidate['target_quaternion']]}"
            )

        if not ENABLE_REAL_MOTION:
            report["status"] = "DRY_RUN_PASS"
            save_report(report)
            print("DRY RUN通过，候选四元数已写入报告。")
            print("未发送任何运动命令。")
            return

        confirmation = input(
            "确认急停可用、工作区无人、允许小角度姿态测试后输入 PROBE："
        ).strip()
        if confirmation != "PROBE":
            report["status"] = "CANCELLED_BY_USER"
            save_report(report)
            print("未输入PROBE，已取消。")
            return

        for index, candidate in enumerate(candidates, start=1):
            before = wait_for_pose(tf_api, RIGHT_FRAME)
            print("-" * 76)
            print(f"测试 {index}/{len(candidates)}: {candidate['name']}")
            print(
                "目标 Quaternion: "
                f"{[round(v, 6) for v in candidate['target_quaternion']]}"
            )

            move_orientation(
                robot=robot,
                left_hold=left_initial,
                right_position=right_initial.position,
                q_start=before.orientation,
                q_goal=candidate["target_quaternion"],
            )
            report["motion_command_sent"] = True
            time.sleep(TF_SETTLE_SECONDS)

            after = wait_for_pose(tf_api, RIGHT_FRAME)
            position_error = position_distance(
                right_initial.position, after.position
            )
            orientation_error = quaternion_angle_error_deg(
                candidate["target_quaternion"], after.orientation
            )

            test_result = {
                "name": candidate["name"],
                "command_quaternion": candidate["target_quaternion"],
                "measured_position": after.position,
                "measured_quaternion": after.orientation,
                "position_drift_m": round(position_error, 6),
                "orientation_error_deg": round(orientation_error, 4),
            }
            report["tests"].append(test_result)
            save_report(report)

            print(f"实测 Quaternion: {[round(v, 6) for v in after.orientation]}")
            print(f"位置漂移: {position_error:.4f} m")
            print(f"姿态误差: {orientation_error:.2f} degree")

            # 每个候选后恢复初始姿态。
            move_orientation(
                robot=robot,
                left_hold=left_initial,
                right_position=right_initial.position,
                q_start=after.orientation,
                q_goal=right_initial.orientation,
            )
            time.sleep(TF_SETTLE_SECONDS)

            recovered = wait_for_pose(tf_api, RIGHT_FRAME)
            recovery_position_error = position_distance(
                right_initial.position, recovered.position
            )
            recovery_orientation_error = quaternion_angle_error_deg(
                right_initial.orientation, recovered.orientation
            )
            test_result["recovery_position_error_m"] = round(
                recovery_position_error, 6
            )
            test_result["recovery_orientation_error_deg"] = round(
                recovery_orientation_error, 4
            )
            save_report(report)

            print(f"恢复位置误差: {recovery_position_error:.4f} m")
            print(f"恢复姿态误差: {recovery_orientation_error:.2f} degree")

            if (
                recovery_position_error > MAX_RECOVERY_POSITION_ERROR_M
                or recovery_orientation_error > MAX_RECOVERY_ANGLE_ERROR_DEG
            ):
                report["status"] = "STOPPED_RECOVERY_ERROR"
                save_report(report)
                raise RuntimeError("恢复初始姿态误差过大，停止后续探测")

        report["status"] = "PROBE_COMPLETED"
        save_report(report)
        print("=" * 76)
        print("姿态探测完成，已恢复初始姿态。")
        print(f"报告文件: {REPORT_FILE}")
        print("=" * 76)

    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED"
        report["error"] = "KeyboardInterrupt"
        save_report(report)
        print("用户中断，已停止后续命令。")
    except Exception as exc:
        if report.get("status") == "INITIALIZED":
            report["status"] = "ERROR"
        report["error"] = str(exc)
        save_report(report)
        raise
    finally:
        if initialized:
            agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
