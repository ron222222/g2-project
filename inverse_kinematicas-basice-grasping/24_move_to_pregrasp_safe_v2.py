#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
24_move_to_pregrasp_safe_v2.py
G2 预抓取轨迹实时安全监控预演版

重要安全限制：
- 本版本不会调用 end_effector_pose_control()。
- 本版本不会驱动机械臂或夹爪。
- 本版本以50Hz生成连续、无阶跃的预抓取轨迹，并逐点执行安全门禁预演。
- 当前尚未确认可靠的软件立即停止接口，因此 SAFETY_LOCK 固定为 True。

输入：
- pose_debug_snapshot.json
- 24_pregrasp_preflight_report.json

输出：
- 24_pregrasp_trajectory_preview.json
"""

import json
import math
import sys
import time
from pathlib import Path

import agibot_gdk

SNAPSHOT_PATH = Path(__file__).with_name("pose_debug_snapshot.json")
PREFLIGHT_PATH = Path(__file__).with_name("24_pregrasp_preflight_report.json")
OUTPUT_PATH = Path(__file__).with_name("24_pregrasp_trajectory_preview.json")

SAFETY_LOCK = True
MOVEMENT_ENABLED = False
RIGHT_END_FRAME = "arm_r_end_link"

RATE_HZ = 50.0
DT_S = 1.0 / RATE_HZ
MAX_LINEAR_STEP_M = 0.001
MAX_PATH_DISTANCE_M = 0.45

MIN_X_M = 0.25
MAX_X_M = 0.85
MIN_Y_M = -0.55
MAX_Y_M = 0.10
MIN_Z_M = 0.72
MAX_Z_M = 1.30

BASELINE_SAMPLE_COUNT = 30
BASELINE_SAMPLE_INTERVAL_S = 0.02

# 仅用于监控预演的保守变化量报警阈值。
# 尚未用于真机停止，因为软件立即停止接口未确认。
DELTA_FORCE_NORM_WARNING = 8.0
DELTA_TORQUE_NORM_WARNING = 1.5


def load_json(path, description):
    if not path.exists():
        raise FileNotFoundError(f"未找到{description}: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def vector3_list(vector):
    return [float(vector.x), float(vector.y), float(vector.z)]


def vector_subtract(a, b):
    return [float(a[i] - b[i]) for i in range(3)]


def vector_add(a, b):
    return [float(a[i] + b[i]) for i in range(3)]


def vector_scale(values, scale):
    return [float(value * scale) for value in values]


def vector_norm(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


def mean_vector(samples):
    count = len(samples)
    return [sum(sample[i] for sample in samples) / count for i in range(3)]


def within_workspace(xyz):
    return (
        MIN_X_M <= xyz[0] <= MAX_X_M
        and MIN_Y_M <= xyz[1] <= MAX_Y_M
        and MIN_Z_M <= xyz[2] <= MAX_Z_M
    )


def collision_pairs(status):
    return list(zip(status.collision_pairs_1, status.collision_pairs_2))


def find_right_wrench(status):
    names = list(status.frame_names)
    if RIGHT_END_FRAME not in names:
        raise RuntimeError("Motion Status 中未找到 arm_r_end_link")
    index = names.index(RIGHT_END_FRAME)
    if index >= len(status.wrenches):
        raise RuntimeError("arm_r_end_link 没有对应 Wrench")
    wrench = status.wrenches[index]
    return vector3_list(wrench.force), vector3_list(wrench.torque)


def get_right_hand_pose(tf_api):
    transform = tf_api.get_tf_from_base_link(RIGHT_END_FRAME)
    xyz = [
        float(transform.translation.x),
        float(transform.translation.y),
        float(transform.translation.z),
    ]
    quaternion = [
        float(transform.rotation.x),
        float(transform.rotation.y),
        float(transform.rotation.z),
        float(transform.rotation.w),
    ]
    return xyz, quaternion


def collect_baseline(robot):
    forces = []
    torques = []

    for _ in range(BASELINE_SAMPLE_COUNT):
        status = robot.get_motion_control_status()
        if int(status.error_code) != 0:
            raise RuntimeError(
                f"基线采集时 error_code={status.error_code}, "
                f"error_msg={status.error_msg}"
            )
        if collision_pairs(status):
            raise RuntimeError("基线采集时检测到 collision_pairs")
        force, torque = find_right_wrench(status)
        forces.append(force)
        torques.append(torque)
        time.sleep(BASELINE_SAMPLE_INTERVAL_S)

    return mean_vector(forces), mean_vector(torques)


def build_linear_trajectory(start_xyz, target_xyz):
    delta = vector_subtract(target_xyz, start_xyz)
    distance = vector_norm(delta)

    if distance > MAX_PATH_DISTANCE_M:
        raise RuntimeError(
            f"路径长度 {distance:.6f}m 超过限制 {MAX_PATH_DISTANCE_M}m"
        )

    steps = max(1, int(math.ceil(distance / MAX_LINEAR_STEP_M)))
    trajectory = []

    for index in range(steps + 1):
        ratio = index / steps
        point = vector_add(start_xyz, vector_scale(delta, ratio))
        if not within_workspace(point):
            raise RuntimeError(
                f"轨迹点 {index}/{steps} 超出工作空间: {point}"
            )
        trajectory.append(point)

    return trajectory, distance


def live_safety_check(robot, baseline_force, baseline_torque):
    status = robot.get_motion_control_status()
    force, torque = find_right_wrench(status)
    delta_force = vector_subtract(force, baseline_force)
    delta_torque = vector_subtract(torque, baseline_torque)
    pairs = collision_pairs(status)

    reasons = []
    if int(status.error_code) != 0:
        reasons.append(
            f"error_code={status.error_code}, error_msg={status.error_msg}"
        )
    if pairs:
        reasons.append(f"collision_pairs={pairs}")
    if vector_norm(delta_force) > DELTA_FORCE_NORM_WARNING:
        reasons.append(
            f"DeltaForceNorm={vector_norm(delta_force):.6f}"
        )
    if vector_norm(delta_torque) > DELTA_TORQUE_NORM_WARNING:
        reasons.append(
            f"DeltaTorqueNorm={vector_norm(delta_torque):.6f}"
        )

    return {
        "safe": len(reasons) == 0,
        "mode": int(status.mode),
        "error_code": int(status.error_code),
        "error_msg": str(status.error_msg),
        "collision_pairs": [list(pair) for pair in pairs],
        "force": force,
        "torque": torque,
        "delta_force": delta_force,
        "delta_torque": delta_torque,
        "delta_force_norm": vector_norm(delta_force),
        "delta_torque_norm": vector_norm(delta_torque),
        "reasons": reasons,
    }


def main():
    print("24_move_to_pregrasp_safe_v2.py")
    print("G2 预抓取轨迹安全监控预演")
    print(f"SAFETY_LOCK = {SAFETY_LOCK}")
    print(f"MOVEMENT_ENABLED = {MOVEMENT_ENABLED}")
    print("本程序不会发送机械臂运动命令")

    snapshot = load_json(SNAPSHOT_PATH, "pose_debug_snapshot.json")
    preflight = load_json(PREFLIGHT_PATH, "24_pregrasp_preflight_report.json")

    if preflight.get("all_preflight_checks_passed") is not True:
        raise RuntimeError("预检报告未全部通过，禁止生成轨迹预演")

    if preflight.get("movement_command_sent") is not False:
        raise RuntimeError("预检报告中的 movement_command_sent 状态异常")

    target_xyz = [float(value) for value in snapshot["pregrasp_base_xyz_m"]]

    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK 初始化失败")
        return 1

    robot = agibot_gdk.Robot()
    tf_api = agibot_gdk.TF()
    time.sleep(2.0)

    try:
        start_xyz, orientation_xyzw = get_right_hand_pose(tf_api)
        trajectory, path_distance = build_linear_trajectory(
            start_xyz,
            target_xyz,
        )

        print()
        print("运动目标")
        print(f"  Start XYZ = {start_xyz}")
        print(f"  Target PreGrasp XYZ = {target_xyz}")
        print(f"  Orientation XYZW = {orientation_xyzw}")
        print(f"  Path Distance = {path_distance:.6f} m")
        print(f"  Trajectory Points = {len(trajectory)}")
        print(f"  Rate = {RATE_HZ:.1f} Hz")
        print(f"  Max Step = {MAX_LINEAR_STEP_M:.6f} m")

        print()
        print("采集右臂 Wrench 静止基线")
        baseline_force, baseline_torque = collect_baseline(robot)
        print(f"  Baseline Force = {baseline_force}")
        print(f"  Baseline Torque = {baseline_torque}")

        safety_log = []
        preview_start = time.monotonic()

        for index, point in enumerate(trajectory):
            safety = live_safety_check(
                robot,
                baseline_force,
                baseline_torque,
            )
            safety["trajectory_index"] = index
            safety["trajectory_point"] = point
            safety_log.append(safety)

            if not safety["safe"]:
                print()
                print("安全监控预演触发报警")
                print(f"  index = {index}")
                print(f"  point = {point}")
                print(f"  reasons = {safety['reasons']}")
                break

            if index == 0 or index == len(trajectory) - 1 or index % 50 == 0:
                print(
                    f"  Preview {index}/{len(trajectory)-1}: "
                    f"point={point}, "
                    f"dF={safety['delta_force_norm']:.3f}, "
                    f"dT={safety['delta_torque_norm']:.3f}"
                )

            target_time = preview_start + (index + 1) * DT_S
            remaining = target_time - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)

        preview_passed = (
            len(safety_log) == len(trajectory)
            and all(item["safe"] for item in safety_log)
        )

        report = {
            "program": "24_move_to_pregrasp_safe_v2.py",
            "safety_lock": SAFETY_LOCK,
            "movement_enabled": MOVEMENT_ENABLED,
            "movement_command_sent": False,
            "preview_passed": preview_passed,
            "start_xyz_m": start_xyz,
            "target_pregrasp_xyz_m": target_xyz,
            "orientation_xyzw_reference": orientation_xyzw,
            "path_distance_m": path_distance,
            "rate_hz": RATE_HZ,
            "max_linear_step_m": MAX_LINEAR_STEP_M,
            "trajectory_point_count": len(trajectory),
            "baseline_force": baseline_force,
            "baseline_torque": baseline_torque,
            "force_norm_warning": DELTA_FORCE_NORM_WARNING,
            "torque_norm_warning": DELTA_TORQUE_NORM_WARNING,
            "safety_log": safety_log,
            "blocking_reason": (
                "尚未确认可靠的软件立即停止接口；"
                "end_effector_pose_control 接口本身无碰撞检测。"
            ),
        }

        OUTPUT_PATH.write_text(
            json.dumps(report, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        print()
        print(f"轨迹安全监控预演通过: {preview_passed}")
        print("movement_command_sent: False")
        print("机械臂未运动")
        print(f"报告已保存: {OUTPUT_PATH}")

        return 0 if preview_passed else 2

    finally:
        try:
            agibot_gdk.gdk_release()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print()
        print(f"程序异常: {exc}")
        sys.exit(1)
