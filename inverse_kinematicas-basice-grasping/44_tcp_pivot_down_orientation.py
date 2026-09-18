#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
44_tcp_pivot_down_orientation.py

围绕 gripper_r_center_link（TCP）旋转右夹爪到朝下姿态。
旋转期间保持 TCP 安全点固定，并根据每一步目标四元数反算 arm_r_end_link XYZ。

默认 ENABLE_REAL_MOTION=False，不发送运动命令。
真实测试前必须手动改为 True，并在终端输入 PIVOT。

安全策略：
- 左臂保持启动位姿
- 首先把 TCP 抬到安全高度
- 每个姿态大段最多旋转 5 degree
- 大段内部每步最多 0.5 degree，50 Hz
- 每个 5 degree 大段结束后读取真实 End/TCP
- TCP 漂移、姿态改善不足、力矩异常时立即停止
- 只验证旋转朝下，不水平移动到产品，不下降，不控制夹爪
"""

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import agibot_gdk

ENABLE_REAL_MOTION = True

REPORT_FILE = "tcp_pivot_down_orientation_report.json"
LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"
TCP_FRAME = "gripper_r_center_link"

# URDF: arm_r_end_link -> gripper_r_center_link
TCP_OFFSET_END_M = [0.0, 0.0, 0.14308]
TARGET_DOWN_QUATERNION_XYZW = [1.0, 0.0, 0.0, 0.0]

RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
LIFE_TIME = 0.02

TCP_CLEARANCE_Z_M = 1.020
TRANSLATION_STEP_M = 0.001
PIVOT_SEGMENT_DEG = 5.0
INNER_ORIENTATION_STEP_DEG = 0.5
MAX_PIVOT_SEGMENTS = 24
TARGET_ORIENTATION_TOLERANCE_DEG = 5.0
TCP_POSITION_TOLERANCE_M = 0.020
MAX_TCP_DRIFT_PER_SEGMENT_M = 0.020
MIN_ORIENTATION_PROGRESS_DEG = 1.0
MAX_STALLED_SEGMENTS = 2
TF_SETTLE_SECONDS = 0.60
HOLD_SECONDS = 0.25

BASELINE_SAMPLES = 30
TORQUE_DELTA_LIMIT_NM = 30.0

END_WORKSPACE_X = (0.20, 1.20)
END_WORKSPACE_Y = (-0.80, 0.80)
END_WORKSPACE_Z = (0.20, 1.50)
TCP_WORKSPACE_X = (0.20, 1.20)
TCP_WORKSPACE_Y = (-0.80, 0.80)
TCP_WORKSPACE_Z = (0.20, 1.50)


@dataclass
class PoseData:
    position: List[float]
    orientation: List[float]


def save_report(report):
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def normalize_quaternion(q):
    norm = math.sqrt(sum(float(v) ** 2 for v in q))
    if norm < 1e-12:
        raise RuntimeError("四元数模长接近0")
    return [float(v) / norm for v in q]


def quaternion_angle_error_deg(q1, q2):
    q1 = normalize_quaternion(q1)
    q2 = normalize_quaternion(q2)
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def slerp(q0, q1, alpha):
    q0 = normalize_quaternion(q0)
    q1 = normalize_quaternion(q1)
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = [-v for v in q1]
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return normalize_quaternion(
            [(1.0 - alpha) * a + alpha * b for a, b in zip(q0, q1)]
        )
    theta0 = math.acos(dot)
    sin_theta0 = math.sin(theta0)
    theta = theta0 * alpha
    s0 = math.cos(theta) - dot * math.sin(theta) / sin_theta0
    s1 = math.sin(theta) / sin_theta0
    return normalize_quaternion([s0 * a + s1 * b for a, b in zip(q0, q1)])


def quaternion_to_rotation_matrix(q):
    x, y, z, w = normalize_quaternion(q)
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def rotate_vector(rotation, vector):
    return [
        sum(rotation[row][column] * vector[column] for column in range(3))
        for row in range(3)
    ]


def vector_add(a, b):
    return [float(a[i]) + float(b[i]) for i in range(3)]


def vector_subtract(a, b):
    return [float(a[i]) - float(b[i]) for i in range(3)]


def distance(a, b):
    return math.sqrt(sum((float(b[i]) - float(a[i])) ** 2 for i in range(3)))


def lerp(a, b, alpha):
    return [(1.0 - alpha) * a[i] + alpha * b[i] for i in range(3)]


def workspace_ok(xyz, xlim, ylim, zlim):
    return (
        xlim[0] < xyz[0] < xlim[1]
        and ylim[0] < xyz[1] < ylim[1]
        and zlim[0] < xyz[2] < zlim[1]
    )


def end_from_tcp(tcp_position, end_quaternion):
    rotation = quaternion_to_rotation_matrix(end_quaternion)
    offset_base = rotate_vector(rotation, TCP_OFFSET_END_M)
    end_position = vector_subtract(tcp_position, offset_base)
    return end_position, offset_base


def read_pose(tf_api, frame):
    t = tf_api.get_tf_from_base_link(frame)
    return PoseData(
        [float(t.translation.x), float(t.translation.y), float(t.translation.z)],
        normalize_quaternion([
            float(t.rotation.x), float(t.rotation.y),
            float(t.rotation.z), float(t.rotation.w),
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
    raise RuntimeError(f"等待TF超时: {frame}; last_error={last_error}")


def get_joint_torques(robot) -> Dict[str, float]:
    raw = robot.get_joint_states()
    return {s["name"]: float(s["effort"]) for s in raw["states"]}


def establish_torque_baseline(robot):
    print("正在建立力矩基线，请保持机器人静止...")
    samples = []
    for index in range(BASELINE_SAMPLES):
        samples.append(get_joint_torques(robot))
        print(f"  基线采样 {index + 1}/{BASELINE_SAMPLES}", end="\r")
        time.sleep(0.05)
    print()
    names = set().union(*(s.keys() for s in samples))
    return {
        name: sum(s.get(name, 0.0) for s in samples) / len(samples)
        for name in names
    }


def verify_torque(robot, baseline, report):
    current = get_joint_torques(robot)
    abnormal = []
    for name, value in current.items():
        if name not in baseline:
            continue
        delta = abs(value - baseline[name])
        if delta > TORQUE_DELTA_LIMIT_NM:
            abnormal.append({
                "joint": name,
                "delta": delta,
                "current": value,
                "baseline": baseline[name],
            })
    if abnormal:
        report["status"] = "STOPPED_TORQUE"
        report["torque_abnormal"] = abnormal[:10]
        save_report(report)
        raise RuntimeError(f"力矩安全检查触发，共{len(abnormal)}个关节异常")


def set_both_arm_pose(robot, left_pose, right_pose):
    req = agibot_gdk.EndEffectorPose()
    req.life_time = LIFE_TIME
    req.group = agibot_gdk.EndEffectorControlGroup.kBothArms

    lp = req.left_end_effector_pose.position
    lq = req.left_end_effector_pose.orientation
    rp = req.right_end_effector_pose.position
    rq = req.right_end_effector_pose.orientation

    lp.x, lp.y, lp.z = left_pose.position
    lq.x, lq.y, lq.z, lq.w = left_pose.orientation
    rp.x, rp.y, rp.z = right_pose.position
    rq.x, rq.y, rq.z, rq.w = right_pose.orientation

    result = robot.end_effector_pose_control(req)
    if result != 0:
        raise RuntimeError(f"end_effector_pose_control失败: {result}")


def move_tcp_translation(robot, left_hold, start_end, target_tcp, baseline, report):
    target_end_position, _ = end_from_tcp(target_tcp, start_end.orientation)
    move_distance = distance(start_end.position, target_end_position)
    steps = max(2, int(math.ceil(move_distance / TRANSLATION_STEP_M)))
    print(f"[TCP抬高] End位移={move_distance:.4f} m, 步数={steps}")

    for step in range(1, steps + 1):
        verify_torque(robot, baseline, report)
        alpha = step / steps
        command = PoseData(
            lerp(start_end.position, target_end_position, alpha),
            start_end.orientation.copy(),
        )
        set_both_arm_pose(robot, left_hold, command)
        report["motion_command_sent"] = True
        time.sleep(DT)

    return target_end_position


def command_pivot_segment(
    robot, left_hold, fixed_tcp_position,
    q_start, q_segment_target, baseline, report,
):
    segment_angle = quaternion_angle_error_deg(q_start, q_segment_target)
    steps = max(2, int(math.ceil(segment_angle / INNER_ORIENTATION_STEP_DEG)))

    for step in range(1, steps + 1):
        verify_torque(robot, baseline, report)
        alpha = step / steps
        q_command = slerp(q_start, q_segment_target, alpha)
        end_position, _ = end_from_tcp(fixed_tcp_position, q_command)

        if not workspace_ok(
            end_position, END_WORKSPACE_X, END_WORKSPACE_Y, END_WORKSPACE_Z
        ):
            report["status"] = "STOPPED_END_WORKSPACE"
            save_report(report)
            raise RuntimeError(f"旋转过程End目标超出工作空间: {end_position}")

        set_both_arm_pose(
            robot,
            left_hold,
            PoseData(end_position, q_command),
        )
        report["motion_command_sent"] = True
        time.sleep(DT)

    for _ in range(max(1, int(HOLD_SECONDS * RATE_HZ))):
        verify_torque(robot, baseline, report)
        end_position, _ = end_from_tcp(fixed_tcp_position, q_segment_target)
        set_both_arm_pose(
            robot, left_hold, PoseData(end_position, q_segment_target.copy())
        )
        time.sleep(DT)

    return steps


def main():
    target_down = normalize_quaternion(TARGET_DOWN_QUATERNION_XYZW)
    report = {
        "program": "44_tcp_pivot_down_orientation.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "tcp_offset_end_m": TCP_OFFSET_END_M,
        "target_down_quaternion": target_down,
        "tcp_clearance_z_m": TCP_CLEARANCE_Z_M,
        "pivot_segment_deg": PIVOT_SEGMENT_DEG,
        "motion_command_sent": False,
        "segments": [],
        "status": "INITIALIZED",
    }

    initialized = False
    try:
        init_result = agibot_gdk.gdk_init()
        if init_result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError(f"GDK初始化失败: {init_result}")
        initialized = True

        robot = agibot_gdk.Robot()
        tf_api = agibot_gdk.TF()
        time.sleep(2.0)

        left_initial = wait_for_pose(tf_api, LEFT_FRAME)
        end_initial = wait_for_pose(tf_api, RIGHT_FRAME)
        tcp_initial = wait_for_pose(tf_api, TCP_FRAME)

        fixed_tcp_safe = [
            tcp_initial.position[0],
            tcp_initial.position[1],
            max(tcp_initial.position[2], TCP_CLEARANCE_Z_M),
        ]
        initial_angle_error = quaternion_angle_error_deg(
            end_initial.orientation, target_down
        )
        estimated_segments = int(
            math.ceil(initial_angle_error / PIVOT_SEGMENT_DEG)
        )

        report.update({
            "initial_left_pose": {
                "position": left_initial.position,
                "orientation": left_initial.orientation,
            },
            "initial_end_pose": {
                "position": end_initial.position,
                "orientation": end_initial.orientation,
            },
            "initial_tcp_pose": {
                "position": tcp_initial.position,
                "orientation": tcp_initial.orientation,
            },
            "fixed_tcp_safe_position": fixed_tcp_safe,
            "initial_orientation_error_deg": round(initial_angle_error, 4),
            "estimated_segments": estimated_segments,
        })

        print("=" * 78)
        print("44_tcp_pivot_down_orientation.py")
        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print(f"当前 End XYZ        : {[round(v, 5) for v in end_initial.position]}")
        print(f"当前 TCP XYZ        : {[round(v, 5) for v in tcp_initial.position]}")
        print(f"固定安全 TCP XYZ   : {[round(v, 5) for v in fixed_tcp_safe]}")
        print(f"当前 Quaternion     : {[round(v, 6) for v in end_initial.orientation]}")
        print(f"朝下 Quaternion     : {[round(v, 6) for v in target_down]}")
        print(f"初始姿态误差        : {initial_angle_error:.2f} degree")
        print(f"预计5度大段数       : {estimated_segments}")
        print("旋转时同时反算End XYZ，使TCP保持在安全点")
        print("不水平移动到产品、不下降、不控制夹爪")
        print("=" * 78)

        if estimated_segments > MAX_PIVOT_SEGMENTS:
            report["status"] = "BLOCKED_TOO_MANY_SEGMENTS"
            raise RuntimeError("需要的旋转段数超过安全上限")

        if not workspace_ok(
            fixed_tcp_safe, TCP_WORKSPACE_X, TCP_WORKSPACE_Y, TCP_WORKSPACE_Z
        ):
            report["status"] = "BLOCKED_TCP_WORKSPACE"
            raise RuntimeError("安全TCP点超出工作空间")

        if not ENABLE_REAL_MOTION:
            report["status"] = "DRY_RUN_PASS"
            save_report(report)
            print("DRY RUN通过，未发送任何运动命令。")
            return

        confirmation = input(
            "确认急停可用、TCP旋转扫掠区域无障碍后输入 PIVOT："
        ).strip()
        if confirmation != "PIVOT":
            report["status"] = "CANCELLED_BY_USER"
            save_report(report)
            print("未输入PIVOT，已取消。")
            return

        baseline = establish_torque_baseline(robot)
        report["torque_baseline_joint_count"] = len(baseline)

        # 先把TCP抬高到安全点，姿态保持不变。
        end_before_raise = wait_for_pose(tf_api, RIGHT_FRAME)
        move_tcp_translation(
            robot, left_initial, end_before_raise,
            fixed_tcp_safe, baseline, report,
        )
        time.sleep(TF_SETTLE_SECONDS)

        tcp_after_raise = wait_for_pose(tf_api, TCP_FRAME)
        raise_error = distance(tcp_after_raise.position, fixed_tcp_safe)
        report["raise_validation"] = {
            "measured_tcp_position": tcp_after_raise.position,
            "tcp_position_error_m": round(raise_error, 6),
        }
        save_report(report)
        print(f"[TCP抬高验证] TCP误差={raise_error:.4f} m")
        if raise_error > TCP_POSITION_TOLERANCE_M:
            report["status"] = "STOPPED_RAISE_NOT_REACHED"
            save_report(report)
            raise RuntimeError("TCP安全点未到位，停止旋转")

        previous_angle_error = quaternion_angle_error_deg(
            wait_for_pose(tf_api, RIGHT_FRAME).orientation, target_down
        )
        stalled_segments = 0

        for segment_index in range(1, MAX_PIVOT_SEGMENTS + 1):
            end_before = wait_for_pose(tf_api, RIGHT_FRAME)
            tcp_before = wait_for_pose(tf_api, TCP_FRAME)
            angle_before = quaternion_angle_error_deg(
                end_before.orientation, target_down
            )

            if angle_before <= TARGET_ORIENTATION_TOLERANCE_DEG:
                report["status"] = "DOWN_ORIENTATION_REACHED"
                break

            alpha = min(1.0, PIVOT_SEGMENT_DEG / angle_before)
            q_segment_target = slerp(
                end_before.orientation, target_down, alpha
            )

            print("-" * 78)
            print(f"旋转段 {segment_index}/{MAX_PIVOT_SEGMENTS}")
            print(f"  段前姿态误差: {angle_before:.2f} degree")
            print(f"  TCP段前位置  : {[round(v, 5) for v in tcp_before.position]}")

            inner_steps = command_pivot_segment(
                robot, left_initial, fixed_tcp_safe,
                end_before.orientation, q_segment_target,
                baseline, report,
            )
            time.sleep(TF_SETTLE_SECONDS)

            end_after = wait_for_pose(tf_api, RIGHT_FRAME)
            tcp_after = wait_for_pose(tf_api, TCP_FRAME)
            angle_after = quaternion_angle_error_deg(
                end_after.orientation, target_down
            )
            orientation_progress = angle_before - angle_after
            tcp_drift = distance(tcp_after.position, fixed_tcp_safe)

            record = {
                "segment": segment_index,
                "inner_steps": inner_steps,
                "command_target_quaternion": q_segment_target,
                "measured_end_position": end_after.position,
                "measured_end_quaternion": end_after.orientation,
                "measured_tcp_position": tcp_after.position,
                "angle_before_deg": round(angle_before, 4),
                "angle_after_deg": round(angle_after, 4),
                "orientation_progress_deg": round(orientation_progress, 4),
                "tcp_drift_m": round(tcp_drift, 6),
            }
            report["segments"].append(record)
            save_report(report)

            print(f"  段后姿态误差: {angle_after:.2f} degree")
            print(f"  本段姿态改善: {orientation_progress:.2f} degree")
            print(f"  TCP固定点漂移: {tcp_drift:.4f} m")

            if tcp_drift > MAX_TCP_DRIFT_PER_SEGMENT_M:
                report["status"] = "STOPPED_TCP_DRIFT"
                save_report(report)
                raise RuntimeError("TCP漂移超过安全阈值")

            if orientation_progress < MIN_ORIENTATION_PROGRESS_DEG:
                stalled_segments += 1
                print(
                    f"  警告：姿态改善不足，连续停滞 "
                    f"{stalled_segments}/{MAX_STALLED_SEGMENTS}"
                )
            else:
                stalled_segments = 0

            if stalled_segments >= MAX_STALLED_SEGMENTS:
                report["status"] = "STOPPED_ORIENTATION_STALLED"
                break

            if angle_after > previous_angle_error + 2.0:
                report["status"] = "STOPPED_ORIENTATION_DIVERGING"
                break

            previous_angle_error = angle_after
        else:
            report["status"] = "STOPPED_MAX_SEGMENTS"

        final_end = wait_for_pose(tf_api, RIGHT_FRAME)
        final_tcp = wait_for_pose(tf_api, TCP_FRAME)
        final_angle_error = quaternion_angle_error_deg(
            final_end.orientation, target_down
        )
        final_tcp_error = distance(final_tcp.position, fixed_tcp_safe)

        if (
            final_angle_error <= TARGET_ORIENTATION_TOLERANCE_DEG
            and final_tcp_error <= TCP_POSITION_TOLERANCE_M
        ):
            report["status"] = "DOWN_ORIENTATION_REACHED"

        report.update({
            "final_end_pose": {
                "position": final_end.position,
                "orientation": final_end.orientation,
            },
            "final_tcp_pose": {
                "position": final_tcp.position,
                "orientation": final_tcp.orientation,
            },
            "final_orientation_error_deg": round(final_angle_error, 4),
            "final_tcp_position_error_m": round(final_tcp_error, 6),
        })
        save_report(report)

        print("=" * 78)
        print(f"结束状态          : {report['status']}")
        print(f"最终 End XYZ      : {[round(v, 5) for v in final_end.position]}")
        print(f"最终 TCP XYZ      : {[round(v, 5) for v in final_tcp.position]}")
        print(f"最终姿态误差      : {final_angle_error:.2f} degree")
        print(f"最终 TCP 固定误差 : {final_tcp_error:.4f} m")
        print(f"报告文件          : {REPORT_FILE}")
        print("=" * 78)

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
