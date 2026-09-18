#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
43_tcp_pregrasp_move_safe.py

安全移动右臂，使 gripper_r_center_link 到达 TCP PreGrasp 目标。
输入：tcp_pregrasp_plan.json

分阶段策略：
1. 抬高 arm_r_end_link 到旋转安全高度，保持当前姿态。
2. 在安全高度将右夹爪姿态渐变为朝下姿态。
3. 保持朝下姿态，平移 arm_r_end_link 到规划目标。
4. 读取 arm_r_end_link 与 gripper_r_center_link，校验最终误差。

默认 ENABLE_REAL_MOTION=False，不发送任何运动命令。
不控制夹爪、不下探、不抓取。
真实运动前必须手动改为 True，并在终端输入 MOVE。
"""

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import agibot_gdk

# ==================== 安全配置 ====================
ENABLE_REAL_MOTION = True

PLAN_FILE = "tcp_pregrasp_plan.json"
REPORT_FILE = "tcp_pregrasp_move_report.json"

LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"
TCP_FRAME = "gripper_r_center_link"

RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
LIFE_TIME = 0.02

TRANSLATION_STEP_M = 0.001
ORIENTATION_STEP_DEG = 1.0
HOLD_SECONDS = 0.30
TF_SETTLE_SECONDS = 0.60

ROTATION_CLEARANCE_EXTRA_M = 0.030
MAX_TOTAL_TRANSLATION_M = 0.60
MAX_ORIENTATION_CHANGE_DEG = 120.0
MAX_START_XY_DISTANCE_M = 0.50

END_TARGET_TOLERANCE_M = 0.015
TCP_TARGET_TOLERANCE_M = 0.020
ORIENTATION_TOLERANCE_DEG = 5.0

BASELINE_SAMPLES = 30
TORQUE_DELTA_LIMIT_NM = 30.0

END_WORKSPACE_X = (0.20, 1.20)
END_WORKSPACE_Y = (-0.80, 0.80)
END_WORKSPACE_Z = (0.20, 1.50)


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
    norm = math.sqrt(sum(float(value) ** 2 for value in q))
    if norm < 1e-12:
        raise RuntimeError("四元数模长接近 0")
    return [float(value) / norm for value in q]


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
        q1 = [-value for value in q1]
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
        s0 * a + s1 * b for a, b in zip(q0, q1)
    ])


def distance(a, b):
    return math.sqrt(sum((float(y) - float(x)) ** 2 for x, y in zip(a, b)))


def xy_distance(a, b):
    return math.sqrt(
        (float(b[0]) - float(a[0])) ** 2
        + (float(b[1]) - float(a[1])) ** 2
    )


def lerp(a, b, alpha):
    return [(1.0 - alpha) * x + alpha * y for x, y in zip(a, b)]


def workspace_ok(xyz):
    return (
        END_WORKSPACE_X[0] < xyz[0] < END_WORKSPACE_X[1]
        and END_WORKSPACE_Y[0] < xyz[1] < END_WORKSPACE_Y[1]
        and END_WORKSPACE_Z[0] < xyz[2] < END_WORKSPACE_Z[1]
    )


def read_pose(tf_api, frame):
    transform = tf_api.get_tf_from_base_link(frame)
    return PoseData(
        position=[
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
        ],
        orientation=normalize_quaternion([
            float(transform.rotation.x),
            float(transform.rotation.y),
            float(transform.rotation.z),
            float(transform.rotation.w),
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
        f"等待 TF 超时: base_link <- {frame}; last_error={last_error}"
    )


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


def check_torque_delta(robot, baseline):
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
        raise RuntimeError(f"end_effector_pose_control 失败: {result}")


def verify_torque(robot, baseline, report):
    abnormal = check_torque_delta(robot, baseline)
    if abnormal:
        report["status"] = "STOPPED_TORQUE"
        report["torque_abnormal"] = abnormal[:10]
        raise RuntimeError(
            f"力矩安全检查触发，共 {len(abnormal)} 个关节异常"
        )


def move_translation(
    robot,
    left_hold,
    start_pose,
    target_position,
    fixed_orientation,
    baseline,
    report,
    stage_name,
):
    move_distance = distance(start_pose.position, target_position)
    steps = max(2, int(math.ceil(move_distance / TRANSLATION_STEP_M)))

    print(f"[{stage_name}] 距离={move_distance:.4f} m, 步数={steps}")
    for step in range(1, steps + 1):
        verify_torque(robot, baseline, report)
        alpha = step / steps
        command = PoseData(
            position=lerp(start_pose.position, target_position, alpha),
            orientation=fixed_orientation.copy(),
        )
        set_both_arm_pose(robot, left_hold, command)
        report["motion_command_sent"] = True
        time.sleep(DT)

    final_command = PoseData(
        position=target_position.copy(),
        orientation=fixed_orientation.copy(),
    )
    for _ in range(max(1, int(HOLD_SECONDS * RATE_HZ))):
        verify_torque(robot, baseline, report)
        set_both_arm_pose(robot, left_hold, final_command)
        time.sleep(DT)

    return steps


def move_orientation(
    robot,
    left_hold,
    fixed_position,
    start_quaternion,
    target_quaternion,
    baseline,
    report,
):
    angle = quaternion_angle_error_deg(start_quaternion, target_quaternion)
    steps = max(2, int(math.ceil(angle / ORIENTATION_STEP_DEG)))

    print(f"[安全高度旋转] 角度={angle:.2f} degree, 步数={steps}")
    for step in range(1, steps + 1):
        verify_torque(robot, baseline, report)
        alpha = step / steps
        command = PoseData(
            position=fixed_position.copy(),
            orientation=slerp(start_quaternion, target_quaternion, alpha),
        )
        set_both_arm_pose(robot, left_hold, command)
        report["motion_command_sent"] = True
        time.sleep(DT)

    final_command = PoseData(
        position=fixed_position.copy(),
        orientation=target_quaternion.copy(),
    )
    for _ in range(max(1, int(HOLD_SECONDS * RATE_HZ))):
        verify_torque(robot, baseline, report)
        set_both_arm_pose(robot, left_hold, final_command)
        time.sleep(DT)

    return steps


def load_plan():
    if not Path(PLAN_FILE).exists():
        raise RuntimeError(
            f"未找到 {PLAN_FILE}，请先运行 42_tcp_pregrasp_pose_planner.py"
        )

    plan = json.loads(Path(PLAN_FILE).read_text(encoding="utf-8"))
    validation = plan.get("validation", {})
    if validation.get("tcp_workspace_check") != "PASS":
        raise RuntimeError("TCP 工作空间检查未通过")
    if validation.get("end_workspace_check") != "PASS":
        raise RuntimeError("arm_r_end_link 工作空间检查未通过")
    if validation.get("motion_command_sent") is not False:
        raise RuntimeError("规划文件中的 motion_command_sent 不是 false")

    target_end = plan["arm_r_end_link_target"]
    target_tcp = plan["tcp_pregrasp_target"]

    return (
        plan,
        [float(v) for v in target_end["position"]],
        normalize_quaternion(target_end["quaternion_xyzw"]),
        [float(v) for v in target_tcp["position"]],
        normalize_quaternion(target_tcp["quaternion_xyzw"]),
    )


def main():
    (
        plan,
        end_target_position,
        end_target_quaternion,
        tcp_target_position,
        tcp_target_quaternion,
    ) = load_plan()

    report = {
        "program": "43_tcp_pregrasp_move_safe.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "source_plan": PLAN_FILE,
        "target_end_position": end_target_position,
        "target_end_quaternion": end_target_quaternion,
        "target_tcp_position": tcp_target_position,
        "target_tcp_quaternion": tcp_target_quaternion,
        "motion_command_sent": False,
        "stages": [],
        "status": "INITIALIZED",
    }

    if not workspace_ok(end_target_position):
        report["status"] = "BLOCKED_TARGET_WORKSPACE"
        save_report(report)
        raise RuntimeError("arm_r_end_link 目标超出软件工作空间")

    initialized = False
    try:
        result = agibot_gdk.gdk_init()
        if result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError(f"GDK 初始化失败: {result}")
        initialized = True

        robot = agibot_gdk.Robot()
        tf_api = agibot_gdk.TF()
        time.sleep(2.0)

        left_initial = wait_for_pose(tf_api, LEFT_FRAME)
        end_initial = wait_for_pose(tf_api, RIGHT_FRAME)
        tcp_initial = wait_for_pose(tf_api, TCP_FRAME)

        orientation_change = quaternion_angle_error_deg(
            end_initial.orientation,
            end_target_quaternion,
        )
        total_translation = distance(
            end_initial.position,
            end_target_position,
        )
        start_xy_distance = xy_distance(
            end_initial.position,
            end_target_position,
        )

        clearance_z = max(
            end_initial.position[2],
            end_target_position[2],
        ) + ROTATION_CLEARANCE_EXTRA_M

        clearance_start_position = [
            end_initial.position[0],
            end_initial.position[1],
            clearance_z,
        ]
        clearance_target_position = [
            end_target_position[0],
            end_target_position[1],
            clearance_z,
        ]

        for candidate in (
            clearance_start_position,
            clearance_target_position,
            end_target_position,
        ):
            if not workspace_ok(candidate):
                report["status"] = "BLOCKED_CLEARANCE_WORKSPACE"
                save_report(report)
                raise RuntimeError(f"中间目标超出软件工作空间: {candidate}")

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
            "orientation_change_deg": round(orientation_change, 6),
            "total_translation_m": round(total_translation, 6),
            "start_xy_distance_m": round(start_xy_distance, 6),
            "clearance_z_m": clearance_z,
            "clearance_start_position": clearance_start_position,
            "clearance_target_position": clearance_target_position,
        })

        print("=" * 78)
        print("43_tcp_pregrasp_move_safe.py")
        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print(f"当前 End XYZ       : {[round(v, 5) for v in end_initial.position]}")
        print(f"当前 TCP XYZ       : {[round(v, 5) for v in tcp_initial.position]}")
        print(f"目标 End XYZ       : {[round(v, 5) for v in end_target_position]}")
        print(f"目标 TCP XYZ       : {[round(v, 5) for v in tcp_target_position]}")
        print(f"目标 Quaternion    : {[round(v, 6) for v in end_target_quaternion]}")
        print(f"姿态变化           : {orientation_change:.2f} degree")
        print(f"End 总位移         : {total_translation:.4f} m")
        print(f"旋转安全高度       : {clearance_z:.4f} m")
        print("阶段: 抬高 -> 安全高度旋转 -> 水平移动 -> 下降至目标")
        print("不控制夹爪、不下探至产品、不执行抓取")
        print("=" * 78)

        if orientation_change > MAX_ORIENTATION_CHANGE_DEG:
            report["status"] = "BLOCKED_ORIENTATION_CHANGE"
            raise RuntimeError("目标姿态变化超过安全上限")
        if total_translation > MAX_TOTAL_TRANSLATION_M:
            report["status"] = "BLOCKED_TRANSLATION_DISTANCE"
            raise RuntimeError("End 总位移超过安全上限")
        if start_xy_distance > MAX_START_XY_DISTANCE_M:
            report["status"] = "BLOCKED_XY_DISTANCE"
            raise RuntimeError("End 水平位移超过安全上限")

        if not ENABLE_REAL_MOTION:
            report["status"] = "DRY_RUN_PASS"
            save_report(report)
            print("DRY RUN 通过，未发送任何运动命令。")
            print(f"报告文件: {REPORT_FILE}")
            return

        confirmation = input(
            "确认急停可用、工作区无人、夹爪旋转扫掠区域无障碍后输入 MOVE："
        ).strip()
        if confirmation != "MOVE":
            report["status"] = "CANCELLED_BY_USER"
            save_report(report)
            print("未输入 MOVE，已取消。")
            return

        baseline = establish_torque_baseline(robot)
        report["torque_baseline_joint_count"] = len(baseline)

        # Stage 1: 抬高 End，保持当前姿态。
        stage_start = wait_for_pose(tf_api, RIGHT_FRAME)
        steps = move_translation(
            robot,
            left_initial,
            stage_start,
            clearance_start_position,
            end_initial.orientation,
            baseline,
            report,
            "阶段1 抬高",
        )
        report["stages"].append({
            "name": "raise_to_clearance",
            "steps": steps,
            "target_position": clearance_start_position,
        })
        save_report(report)
        time.sleep(TF_SETTLE_SECONDS)

        # Stage 2: 安全高度旋转到朝下姿态。
        stage_start = wait_for_pose(tf_api, RIGHT_FRAME)
        steps = move_orientation(
            robot,
            left_initial,
            clearance_start_position,
            stage_start.orientation,
            end_target_quaternion,
            baseline,
            report,
        )
        report["stages"].append({
            "name": "rotate_at_clearance",
            "steps": steps,
            "target_quaternion": end_target_quaternion,
        })
        save_report(report)
        time.sleep(TF_SETTLE_SECONDS)

        # Stage 3: 保持朝下姿态，水平移到目标 XY 上方。
        stage_start = wait_for_pose(tf_api, RIGHT_FRAME)
        steps = move_translation(
            robot,
            left_initial,
            stage_start,
            clearance_target_position,
            end_target_quaternion,
            baseline,
            report,
            "阶段3 水平移动",
        )
        report["stages"].append({
            "name": "translate_at_clearance",
            "steps": steps,
            "target_position": clearance_target_position,
        })
        save_report(report)
        time.sleep(TF_SETTLE_SECONDS)

        # Stage 4: 保持朝下姿态，下降到 End 目标。
        stage_start = wait_for_pose(tf_api, RIGHT_FRAME)
        steps = move_translation(
            robot,
            left_initial,
            stage_start,
            end_target_position,
            end_target_quaternion,
            baseline,
            report,
            "阶段4 下降至 TCP PreGrasp",
        )
        report["stages"].append({
            "name": "descend_to_pregrasp",
            "steps": steps,
            "target_position": end_target_position,
        })
        save_report(report)
        time.sleep(TF_SETTLE_SECONDS)

        final_end = wait_for_pose(tf_api, RIGHT_FRAME)
        final_tcp = wait_for_pose(tf_api, TCP_FRAME)

        end_position_error = distance(
            final_end.position,
            end_target_position,
        )
        tcp_position_error = distance(
            final_tcp.position,
            tcp_target_position,
        )
        end_orientation_error = quaternion_angle_error_deg(
            final_end.orientation,
            end_target_quaternion,
        )
        tcp_orientation_error = quaternion_angle_error_deg(
            final_tcp.orientation,
            tcp_target_quaternion,
        )

        success = (
            end_position_error <= END_TARGET_TOLERANCE_M
            and tcp_position_error <= TCP_TARGET_TOLERANCE_M
            and end_orientation_error <= ORIENTATION_TOLERANCE_DEG
            and tcp_orientation_error <= ORIENTATION_TOLERANCE_DEG
        )

        report.update({
            "final_end_pose": {
                "position": final_end.position,
                "orientation": final_end.orientation,
            },
            "final_tcp_pose": {
                "position": final_tcp.position,
                "orientation": final_tcp.orientation,
            },
            "final_errors": {
                "end_position_error_m": round(end_position_error, 6),
                "tcp_position_error_m": round(tcp_position_error, 6),
                "end_orientation_error_deg": round(end_orientation_error, 4),
                "tcp_orientation_error_deg": round(tcp_orientation_error, 4),
            },
            "status": "TARGET_REACHED" if success else "TARGET_NOT_REACHED",
        })
        save_report(report)

        print("=" * 78)
        print(f"结束状态             : {report['status']}")
        print(f"最终 End XYZ         : {[round(v, 5) for v in final_end.position]}")
        print(f"最终 TCP XYZ         : {[round(v, 5) for v in final_tcp.position]}")
        print(f"End 位置误差         : {end_position_error:.4f} m")
        print(f"TCP 位置误差         : {tcp_position_error:.4f} m")
        print(f"End 姿态误差         : {end_orientation_error:.2f} degree")
        print(f"TCP 姿态误差         : {tcp_orientation_error:.2f} degree")
        print(f"报告文件             : {REPORT_FILE}")
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
