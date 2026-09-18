#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
40_workspace_boundary_probe.py

右臂 +X 方向工作空间边界探测。

默认 ENABLE_REAL_MOTION=False，不发送运动命令。
真实探测时：
- 左臂保持启动位姿
- 右臂保持启动 Y、Z 和四元数姿态
- 每轮仅将 X 目标增加 10 mm
- 每轮内部使用 1 mm、50 Hz 插值
- 每轮结束重新读取 arm_r_end_link
- 实际 X 增量连续两轮不足 2 mm 时停止
- 力矩相对启动基线变化超过阈值时停止
- 最多探测 12 轮，总命令增量不超过 120 mm
- 不控制夹爪、不下探、不抓取

真实运动前必须：
1. 手动设置 ENABLE_REAL_MOTION=True
2. 确认急停可用、路径无障碍、工作区无人
3. 在终端输入 PROBE
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
REPORT_FILE = "workspace_boundary_probe_report.json"

LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"

RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
LIFE_TIME = 0.02

PROBE_STEP_X_M = 0.010             # 每轮 +X 10 mm
INNER_STEP_M = 0.001               # 每个发送步最多 1 mm
MAX_PROBE_STEPS = 12               # 最多 120 mm
HOLD_SECONDS = 0.30
TF_SETTLE_SECONDS = 0.50

MIN_ACTUAL_X_PROGRESS_M = 0.002    # 每轮实测 X 至少增加 2 mm
MAX_STALLED_STEPS = 2
MAX_POSITION_DRIFT_Y_M = 0.030
MAX_POSITION_DRIFT_Z_M = 0.030

BASELINE_SAMPLES = 30
TORQUE_DELTA_LIMIT_NM = 30.0

# 保守的绝对工作空间门限，仅作软件拦截
WORKSPACE_X = (0.20, 1.20)
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


def normalize_quaternion(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-9:
        raise RuntimeError("四元数模长接近 0")
    return [v / norm for v in q]


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


def workspace_ok(xyz):
    return (
        WORKSPACE_X[0] < xyz[0] < WORKSPACE_X[1]
        and WORKSPACE_Y[0] < xyz[1] < WORKSPACE_Y[1]
        and WORKSPACE_Z[0] < xyz[2] < WORKSPACE_Z[1]
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
    for name, current_value in current.items():
        if name not in baseline:
            continue
        delta = abs(current_value - baseline[name])
        if delta > TORQUE_DELTA_LIMIT_NM:
            abnormal.append({
                "joint": name,
                "delta": float(delta),
                "current": float(current_value),
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


def lerp(a, b, alpha):
    return [(1.0 - alpha) * x + alpha * y for x, y in zip(a, b)]


def execute_probe_step(
    robot,
    left_hold,
    right_start,
    right_target,
    torque_baseline,
    report,
):
    distance = abs(right_target.position[0] - right_start.position[0])
    inner_steps = max(2, int(math.ceil(distance / INNER_STEP_M)))

    for inner_index in range(1, inner_steps + 1):
        abnormal = check_torque_delta(robot, torque_baseline)
        if abnormal:
            report["status"] = "STOPPED_TORQUE"
            report["torque_abnormal"] = abnormal[:10]
            raise RuntimeError(
                f"力矩安全检查触发，共 {len(abnormal)} 个关节异常"
            )

        alpha = inner_index / inner_steps
        command = PoseData(
            position=lerp(
                right_start.position,
                right_target.position,
                alpha,
            ),
            orientation=right_start.orientation.copy(),
        )
        set_both_arm_pose(robot, left_hold, command)
        report["motion_command_sent"] = True
        time.sleep(DT)

    for _ in range(max(1, int(HOLD_SECONDS * RATE_HZ))):
        set_both_arm_pose(robot, left_hold, right_target)
        time.sleep(DT)

    return inner_steps


def main():
    report = {
        "program": "40_workspace_boundary_probe.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "probe_axis": "+X",
        "probe_step_x_m": PROBE_STEP_X_M,
        "inner_step_m": INNER_STEP_M,
        "max_probe_steps": MAX_PROBE_STEPS,
        "torque_delta_limit_nm": TORQUE_DELTA_LIMIT_NM,
        "motion_command_sent": False,
        "steps": [],
        "status": "INITIALIZED",
    }

    initialized = False
    try:
        init_result = agibot_gdk.gdk_init()
        if init_result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError(f"GDK 初始化失败: {init_result}")
        initialized = True

        robot = agibot_gdk.Robot()
        tf_api = agibot_gdk.TF()
        time.sleep(2.0)

        left_initial = wait_for_pose(tf_api, LEFT_FRAME)
        right_initial = wait_for_pose(tf_api, RIGHT_FRAME)

        report.update({
            "initial_left_pose": {
                "position": left_initial.position,
                "orientation": left_initial.orientation,
            },
            "initial_right_pose": {
                "position": right_initial.position,
                "orientation": right_initial.orientation,
            },
        })

        theoretical_final_x = (
            right_initial.position[0]
            + PROBE_STEP_X_M * MAX_PROBE_STEPS
        )

        print("=" * 76)
        print("40_workspace_boundary_probe.py")
        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print(f"当前右臂 XYZ: {[round(v, 5) for v in right_initial.position]}")
        print(f"当前右臂 Quaternion: {[round(v, 6) for v in right_initial.orientation]}")
        print(f"探测方向: +X，每轮 {PROBE_STEP_X_M * 1000:.1f} mm")
        print(f"最多轮数: {MAX_PROBE_STEPS}")
        print(f"理论最大命令 X: {theoretical_final_x:.4f} m")
        print(f"力矩增量阈值: {TORQUE_DELTA_LIMIT_NM:.1f} N·m")
        print("Y、Z 与四元数保持启动值")
        print("不控制夹爪、不下探、不抓取")
        print("=" * 76)

        if theoretical_final_x >= WORKSPACE_X[1]:
            print(
                "提示：理论命令将接近软件 X 上限，程序会在每轮前再次检查。"
            )

        if not ENABLE_REAL_MOTION:
            report.update({
                "theoretical_final_x": theoretical_final_x,
                "status": "DRY_RUN_PASS",
            })
            save_report(report)
            print("DRY RUN 通过，未发送任何运动命令。")
            return

        confirmation = input(
            "确认急停可用、工作区无人、允许 +X 边界探测后输入 PROBE："
        ).strip()
        if confirmation != "PROBE":
            report["status"] = "CANCELLED_BY_USER"
            save_report(report)
            print("未输入 PROBE，已取消。")
            return

        torque_baseline = establish_torque_baseline(robot)
        report["torque_baseline_joint_count"] = len(torque_baseline)

        stalled_steps = 0
        last_pose = right_initial

        for probe_index in range(1, MAX_PROBE_STEPS + 1):
            before = wait_for_pose(tf_api, RIGHT_FRAME)
            target_xyz = [
                before.position[0] + PROBE_STEP_X_M,
                right_initial.position[1],
                right_initial.position[2],
            ]

            if not workspace_ok(target_xyz):
                report["status"] = "STOPPED_WORKSPACE_LIMIT"
                print(f"第 {probe_index} 轮目标超过软件工作空间，停止。")
                break

            target_pose = PoseData(
                position=target_xyz,
                orientation=right_initial.orientation.copy(),
            )

            print("-" * 76)
            print(f"探测轮 {probe_index}/{MAX_PROBE_STEPS}")
            print(f"  当前 XYZ: {[round(v, 5) for v in before.position]}")
            print(f"  命令目标 XYZ: {[round(v, 5) for v in target_xyz]}")

            inner_steps = execute_probe_step(
                robot=robot,
                left_hold=left_initial,
                right_start=before,
                right_target=target_pose,
                torque_baseline=torque_baseline,
                report=report,
            )

            time.sleep(TF_SETTLE_SECONDS)
            after = wait_for_pose(tf_api, RIGHT_FRAME)

            actual_dx = after.position[0] - before.position[0]
            dy_drift = after.position[1] - right_initial.position[1]
            dz_drift = after.position[2] - right_initial.position[2]

            record = {
                "probe_step": probe_index,
                "command_start_xyz": before.position,
                "command_target_xyz": target_xyz,
                "measured_end_xyz": after.position,
                "inner_steps": inner_steps,
                "actual_dx_m": round(actual_dx, 6),
                "y_drift_from_initial_m": round(dy_drift, 6),
                "z_drift_from_initial_m": round(dz_drift, 6),
            }
            report["steps"].append(record)
            save_report(report)

            print(f"  实测结束 XYZ: {[round(v, 5) for v in after.position]}")
            print(f"  实际 X 增量: {actual_dx * 1000:.2f} mm")
            print(f"  Y 漂移: {dy_drift * 1000:.2f} mm")
            print(f"  Z 漂移: {dz_drift * 1000:.2f} mm")

            if abs(dy_drift) > MAX_POSITION_DRIFT_Y_M:
                report["status"] = "STOPPED_Y_DRIFT"
                print("Y 漂移超过安全阈值，停止。")
                last_pose = after
                break

            if abs(dz_drift) > MAX_POSITION_DRIFT_Z_M:
                report["status"] = "STOPPED_Z_DRIFT"
                print("Z 漂移超过安全阈值，停止。")
                last_pose = after
                break

            if actual_dx < MIN_ACTUAL_X_PROGRESS_M:
                stalled_steps += 1
                print(
                    f"  警告：X 实际进展不足，连续停滞 "
                    f"{stalled_steps}/{MAX_STALLED_STEPS}"
                )
            else:
                stalled_steps = 0

            if stalled_steps >= MAX_STALLED_STEPS:
                report["status"] = "BOUNDARY_DETECTED"
                report["boundary_x_m"] = round(after.position[0], 6)
                print("连续两轮 X 进展不足，判定接近当前姿态下边界。")
                last_pose = after
                break

            last_pose = after
        else:
            report["status"] = "MAX_STEPS_COMPLETED"

        final_pose = wait_for_pose(tf_api, RIGHT_FRAME)
        report.update({
            "final_right_pose": {
                "position": final_pose.position,
                "orientation": final_pose.orientation,
            },
            "completed_probe_steps": len(report["steps"]),
        })

        if report["status"] == "INITIALIZED":
            report["status"] = "COMPLETED"

        save_report(report)
        print("=" * 76)
        print(f"结束状态: {report['status']}")
        print(f"最终右臂 XYZ: {[round(v, 5) for v in final_pose.position]}")
        if "boundary_x_m" in report:
            print(f"估计当前姿态下 +X 边界: {report['boundary_x_m']:.4f} m")
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
