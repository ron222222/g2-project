#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
38_move_to_pregrasp_incremental.py

右臂以闭环增量方式逼近 pregrasp_xyz：
- 每一大段最多前进 30 mm
- 每一大段内部以 1 mm、50 Hz 插值发送
- 每段结束重新读取 arm_r_end_link，重新计算误差
- 误差 <= 20 mm 时停止
- 连续两段改善不足时停止
- 左臂保持启动时位姿
- 右臂保持启动时四元数姿态
- 不控制夹爪、不下探、不抓取

默认 ENABLE_REAL_MOTION=False，不发送运动命令。
真实运动前还必须在终端输入 MOVE。
"""

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import agibot_gdk

# ==================== 运动与安全配置 ====================
ENABLE_REAL_MOTION = True

PLAN_FILE = "pregrasp_plan.json"
REPORT_FILE = "move_to_pregrasp_incremental_report.json"

LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"

RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
LIFE_TIME = 0.02

SEGMENT_LENGTH_M = 0.030          # 每个闭环大段最多30 mm
INNER_STEP_M = 0.001              # 每个发送步最多1 mm
TARGET_TOLERANCE_M = 0.020        # 20 mm以内视为到达
MAX_TOTAL_DISTANCE_M = 0.70
MAX_SEGMENTS = 30
HOLD_SECONDS = 0.30
TF_SETTLE_SECONDS = 0.50

BASELINE_SAMPLES = 30
TORQUE_DELTA_LIMIT_NM = 30.0
TORQUE_CHECK_EVERY_N_STEPS = 1

MIN_SEGMENT_PROGRESS_M = 0.002    # 每段至少改善2 mm
MAX_STALLED_SEGMENTS = 2

WORKSPACE_X = (0.20, 1.50)
WORKSPACE_Y = (-0.80, 0.80)
WORKSPACE_Z = (0.20, 1.50)


@dataclass
class PoseData:
    position: List[float]
    orientation: List[float]


def save_report(report):
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def distance(a, b):
    return math.sqrt(sum((float(y) - float(x)) ** 2 for x, y in zip(a, b)))


def lerp(a, b, alpha):
    return [(1.0 - alpha) * x + alpha * y for x, y in zip(a, b)]


def normalize_quaternion(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-9:
        raise RuntimeError("四元数模长接近0")
    return [v / norm for v in q]


def workspace_ok(xyz):
    return (
        WORKSPACE_X[0] < xyz[0] < WORKSPACE_X[1]
        and WORKSPACE_Y[0] < xyz[1] < WORKSPACE_Y[1]
        and WORKSPACE_Z[0] < xyz[2] < WORKSPACE_Z[1]
    )


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


def wait_for_tf(tf_api, frame, timeout_s=10.0):
    deadline = time.time() + timeout_s
    last_error = None
    while time.time() < deadline:
        try:
            if tf_api.can_transform("base_link", frame):
                return read_pose(tf_api, frame)
        except Exception as exc:
            last_error = exc
        time.sleep(0.2)
    raise RuntimeError(f"等待TF超时: base_link <- {frame}; last_error={last_error}")


def get_joint_torques(robot) -> Dict[str, float]:
    raw = robot.get_joint_states()
    return {
        state["name"]: float(state["effort"])
        for state in raw["states"]
    }


def establish_torque_baseline(robot):
    print("正在建立力矩基线，请保持机器人静止...")
    samples = []
    for index in range(BASELINE_SAMPLES):
        samples.append(get_joint_torques(robot))
        print(f"  基线采样 {index + 1}/{BASELINE_SAMPLES}", end="\r")
        time.sleep(0.05)
    print()

    names = set().union(*(sample.keys() for sample in samples))
    return {
        name: sum(sample.get(name, 0.0) for sample in samples) / len(samples)
        for name in names
    }


def check_torque_from_baseline(robot, baseline):
    current = get_joint_torques(robot)
    abnormal = []
    for name, value in current.items():
        if name not in baseline:
            continue
        delta = abs(value - baseline[name])
        if delta > TORQUE_DELTA_LIMIT_NM:
            abnormal.append({
                "joint": name,
                "delta": float(delta),
                "current": float(value),
                "baseline": float(baseline[name]),
            })
    return abnormal


def set_both_arm_pose(robot, left_pose, right_pose):
    request = agibot_gdk.EndEffectorPose()
    request.life_time = LIFE_TIME
    request.group = agibot_gdk.EndEffectorControlGroup.kBothArms

    left_position = request.left_end_effector_pose.position
    left_orientation = request.left_end_effector_pose.orientation
    right_position = request.right_end_effector_pose.position
    right_orientation = request.right_end_effector_pose.orientation

    left_position.x, left_position.y, left_position.z = left_pose.position
    left_orientation.x, left_orientation.y, left_orientation.z, left_orientation.w = (
        left_pose.orientation
    )
    right_position.x, right_position.y, right_position.z = right_pose.position
    right_orientation.x, right_orientation.y, right_orientation.z, right_orientation.w = (
        right_pose.orientation
    )

    result = robot.end_effector_pose_control(request)
    if result != 0:
        raise RuntimeError(f"end_effector_pose_control失败: {result}")


def calculate_segment_target(current_xyz, final_xyz):
    remaining = distance(current_xyz, final_xyz)
    if remaining <= SEGMENT_LENGTH_M:
        return final_xyz.copy()

    ratio = SEGMENT_LENGTH_M / remaining
    return [
        current_xyz[i] + ratio * (final_xyz[i] - current_xyz[i])
        for i in range(3)
    ]


def execute_inner_segment(
    robot,
    left_hold,
    right_start,
    segment_target,
    right_orientation,
    torque_baseline,
    report,
):
    segment_distance = distance(right_start, segment_target)
    steps = max(2, int(math.ceil(segment_distance / INNER_STEP_M)))

    for step in range(1, steps + 1):
        if step % TORQUE_CHECK_EVERY_N_STEPS == 0:
            abnormal = check_torque_from_baseline(robot, torque_baseline)
            if abnormal:
                report["status"] = "STOPPED_TORQUE"
                report["torque_abnormal"] = abnormal[:10]
                raise RuntimeError(
                    f"力矩安全检查触发，共{len(abnormal)}个关节异常"
                )

        alpha = step / steps
        right_command = PoseData(
            position=lerp(right_start, segment_target, alpha),
            orientation=right_orientation.copy(),
        )
        set_both_arm_pose(robot, left_hold, right_command)
        report["motion_command_sent"] = True
        time.sleep(DT)

    final_command = PoseData(
        position=segment_target.copy(),
        orientation=right_orientation.copy(),
    )
    for _ in range(max(1, int(HOLD_SECONDS * RATE_HZ))):
        set_both_arm_pose(robot, left_hold, final_command)
        time.sleep(DT)

    return steps


def main():
    if not Path(PLAN_FILE).exists():
        raise RuntimeError(
            f"未找到 {PLAN_FILE}，请先运行34_yolo_pregrasp_generator.py"
        )

    plan = json.loads(Path(PLAN_FILE).read_text(encoding="utf-8"))
    final_target = [float(value) for value in plan["pregrasp_xyz"]]

    report = {
        "program": "38_move_to_pregrasp_incremental.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "target_pregrasp_xyz": final_target,
        "segment_length_m": SEGMENT_LENGTH_M,
        "inner_step_m": INNER_STEP_M,
        "target_tolerance_m": TARGET_TOLERANCE_M,
        "torque_delta_limit_nm": TORQUE_DELTA_LIMIT_NM,
        "motion_command_sent": False,
        "segments": [],
        "status": "INITIALIZED",
    }

    if not workspace_ok(final_target):
        report["status"] = "BLOCKED_WORKSPACE"
        save_report(report)
        raise RuntimeError(f"目标未通过工作空间检查: {final_target}")

    initialized = False
    try:
        init_result = agibot_gdk.gdk_init()
        if init_result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError(f"GDK初始化失败: {init_result}")
        initialized = True

        robot = agibot_gdk.Robot()
        tf_api = agibot_gdk.TF()
        time.sleep(2.0)

        left_hold = wait_for_tf(tf_api, LEFT_FRAME)
        right_initial = wait_for_tf(tf_api, RIGHT_FRAME)
        initial_error = distance(right_initial.position, final_target)

        report.update({
            "initial_left_pose": {
                "position": left_hold.position,
                "orientation": left_hold.orientation,
            },
            "initial_right_pose": {
                "position": right_initial.position,
                "orientation": right_initial.orientation,
            },
            "initial_error_m": round(initial_error, 6),
        })

        print("=" * 76)
        print("38_move_to_pregrasp_incremental.py")
        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print(f"当前右臂 XYZ: {[round(v, 4) for v in right_initial.position]}")
        print(f"目标 PreGrasp XYZ: {[round(v, 4) for v in final_target]}")
        print(f"初始误差: {initial_error:.4f} m")
        print(f"每段最大位移: {SEGMENT_LENGTH_M * 1000:.1f} mm")
        print(f"每发送步上限: {INNER_STEP_M * 1000:.1f} mm")
        print(f"停止容差: {TARGET_TOLERANCE_M * 1000:.1f} mm")
        print(f"力矩增量阈值: {TORQUE_DELTA_LIMIT_NM:.1f} N·m")
        print("左臂保持启动位姿，右臂保持启动四元数")
        print("不控制夹爪、不下探、不抓取")
        print("=" * 76)

        if initial_error > MAX_TOTAL_DISTANCE_M:
            report["status"] = "BLOCKED_DISTANCE"
            raise RuntimeError("初始距离超过安全上限")

        if initial_error <= TARGET_TOLERANCE_M:
            report["status"] = "ALREADY_WITHIN_TOLERANCE"
            report["final_error_m"] = round(initial_error, 6)
            save_report(report)
            print("当前末端已在目标容差内。")
            return

        if not ENABLE_REAL_MOTION:
            estimated_segments = int(math.ceil(initial_error / SEGMENT_LENGTH_M))
            report["estimated_segments"] = estimated_segments
            report["status"] = "DRY_RUN_PASS"
            save_report(report)
            print(f"DRY RUN通过，预计最多需要 {estimated_segments} 个增量段。")
            print("未发送任何运动命令。")
            return

        confirmation = input(
            "确认急停可用、工作区无人、路径无障碍后输入 MOVE："
        ).strip()
        if confirmation != "MOVE":
            report["status"] = "CANCELLED_BY_USER"
            save_report(report)
            print("未输入MOVE，已取消。")
            return

        torque_baseline = establish_torque_baseline(robot)
        report["torque_baseline_joint_count"] = len(torque_baseline)

        fixed_right_orientation = right_initial.orientation.copy()
        previous_error = initial_error
        stalled_segments = 0

        for segment_index in range(1, MAX_SEGMENTS + 1):
            right_before = wait_for_tf(tf_api, RIGHT_FRAME)
            error_before = distance(right_before.position, final_target)

            if error_before <= TARGET_TOLERANCE_M:
                report["status"] = "TARGET_REACHED"
                break

            segment_target = calculate_segment_target(
                right_before.position, final_target
            )
            if not workspace_ok(segment_target):
                report["status"] = "BLOCKED_SEGMENT_WORKSPACE"
                raise RuntimeError(
                    f"第{segment_index}段目标未通过工作空间检查: {segment_target}"
                )

            print("-" * 76)
            print(f"增量段 {segment_index}/{MAX_SEGMENTS}")
            print(f"  当前 XYZ: {[round(v, 4) for v in right_before.position]}")
            print(f"  段目标 XYZ: {[round(v, 4) for v in segment_target]}")
            print(f"  最终目标误差(段前): {error_before:.4f} m")

            inner_steps = execute_inner_segment(
                robot=robot,
                left_hold=left_hold,
                right_start=right_before.position,
                segment_target=segment_target,
                right_orientation=fixed_right_orientation,
                torque_baseline=torque_baseline,
                report=report,
            )

            time.sleep(TF_SETTLE_SECONDS)
            right_after = wait_for_tf(tf_api, RIGHT_FRAME)
            error_after = distance(right_after.position, final_target)
            progress = error_before - error_after
            actual_motion = distance(right_before.position, right_after.position)

            segment_record = {
                "segment": segment_index,
                "command_start_xyz": right_before.position,
                "command_target_xyz": segment_target,
                "measured_end_xyz": right_after.position,
                "inner_steps": inner_steps,
                "error_before_m": round(error_before, 6),
                "error_after_m": round(error_after, 6),
                "progress_m": round(progress, 6),
                "actual_motion_m": round(actual_motion, 6),
            }
            report["segments"].append(segment_record)
            save_report(report)

            print(f"  实测结束 XYZ: {[round(v, 4) for v in right_after.position]}")
            print(f"  实测移动: {actual_motion:.4f} m")
            print(f"  误差改善: {progress:.4f} m")
            print(f"  剩余误差: {error_after:.4f} m")

            if error_after <= TARGET_TOLERANCE_M:
                report["status"] = "TARGET_REACHED"
                previous_error = error_after
                break

            if progress < MIN_SEGMENT_PROGRESS_M:
                stalled_segments += 1
                print(
                    f"  警告：本段改善不足，连续停滞={stalled_segments}/"
                    f"{MAX_STALLED_SEGMENTS}"
                )
            else:
                stalled_segments = 0

            if stalled_segments >= MAX_STALLED_SEGMENTS:
                report["status"] = "STOPPED_STALLED"
                report["stalled_segments"] = stalled_segments
                previous_error = error_after
                break

            # 若误差反而明显增大，也停止，避免继续偏离。
            if error_after > previous_error + 0.010:
                report["status"] = "STOPPED_DIVERGING"
                previous_error = error_after
                break

            previous_error = error_after
        else:
            report["status"] = "STOPPED_MAX_SEGMENTS"

        final_pose = wait_for_tf(tf_api, RIGHT_FRAME)
        final_error = distance(final_pose.position, final_target)
        report.update({
            "final_right_xyz": final_pose.position,
            "final_error_m": round(final_error, 6),
            "segment_count": len(report["segments"]),
        })

        if final_error <= TARGET_TOLERANCE_M:
            report["status"] = "TARGET_REACHED"

        save_report(report)
        print("=" * 76)
        print(f"结束状态: {report['status']}")
        print(f"最终右臂 XYZ: {[round(v, 4) for v in final_pose.position]}")
        print(f"最终误差: {final_error:.4f} m")
        print("=" * 76)

    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED"
        report["error"] = "KeyboardInterrupt"
        save_report(report)
        print("用户中断，已停止后续指令。")
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
