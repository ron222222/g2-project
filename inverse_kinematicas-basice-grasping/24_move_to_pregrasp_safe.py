#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
24_move_to_pregrasp_safe.py
G2 预抓取运动安全门禁测试版

重要：当前版本不会驱动机械臂。
原因：GDK文档说明 end_effector_pose_control 需要50Hz连续发送、禁止阶跃，
并且该接口本身没有碰撞检测。现阶段尚未确认可靠的软件立即停止接口，
因此本程序只完成运动前安全检查、目标检查和轨迹预演。

输入：23_pose_debugger_live.py 按 S 生成的 pose_debug_snapshot.json
输出：24_pregrasp_preflight_report.json
"""

import json
import math
import sys
import time
from pathlib import Path

import agibot_gdk

SNAPSHOT_PATH = Path(__file__).with_name("pose_debug_snapshot.json")
REPORT_PATH = Path(__file__).with_name("24_pregrasp_preflight_report.json")

SAFETY_LOCK = True
RIGHT_END_FRAME = "arm_r_end_link"

# 运动目标与工作区门禁
PREGRASP_OFFSET_Z_M = 0.150
MIN_X_M = 0.25
MAX_X_M = 0.85
MIN_Y_M = -0.55
MAX_Y_M = 0.10
MIN_Z_M = 0.72
MAX_Z_M = 1.30
MAX_MOVE_DISTANCE_M = 0.45

# 启动前状态稳定性检查
STATUS_SAMPLE_COUNT = 30
STATUS_SAMPLE_INTERVAL_S = 0.02
MAX_FORCE_STD = 1.0
MAX_TORQUE_STD = 0.15


def vector3_list(vector):
    return [float(vector.x), float(vector.y), float(vector.z)]


def vector_subtract(a, b):
    return [float(a[i] - b[i]) for i in range(3)]


def vector_norm(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


def mean_vector(samples):
    count = len(samples)
    return [sum(sample[i] for sample in samples) / count for i in range(3)]


def std_vector(samples, mean):
    count = len(samples)
    return [
        math.sqrt(sum((sample[i] - mean[i]) ** 2 for sample in samples) / count)
        for i in range(3)
    ]


def collision_pairs(status):
    return [
        [str(first), str(second)]
        for first, second in zip(
            status.collision_pairs_1,
            status.collision_pairs_2,
        )
    ]


def find_right_wrench(status):
    frame_names = list(status.frame_names)
    if RIGHT_END_FRAME not in frame_names:
        raise RuntimeError("Motion Status 中未找到 arm_r_end_link")

    index = frame_names.index(RIGHT_END_FRAME)
    if index >= len(status.wrenches):
        raise RuntimeError("arm_r_end_link 没有对应的 Wrench 数据")

    wrench = status.wrenches[index]
    return vector3_list(wrench.force), vector3_list(wrench.torque)


def read_right_hand_xyz(tf_api):
    transform = tf_api.get_tf_from_base_link(RIGHT_END_FRAME)
    return [
        float(transform.translation.x),
        float(transform.translation.y),
        float(transform.translation.z),
    ]


def in_range(value, minimum, maximum):
    return minimum <= value <= maximum


def add_check(checks, name, passed, detail):
    checks.append({
        "name": name,
        "passed": bool(passed),
        "detail": detail,
    })


def sample_safety_status(robot):
    force_samples = []
    torque_samples = []
    modes = []
    errors = []
    all_collisions = []

    for _ in range(STATUS_SAMPLE_COUNT):
        status = robot.get_motion_control_status()
        force, torque = find_right_wrench(status)
        force_samples.append(force)
        torque_samples.append(torque)
        modes.append(int(status.mode))
        errors.append({
            "code": int(status.error_code),
            "message": str(status.error_msg),
        })
        all_collisions.extend(collision_pairs(status))
        time.sleep(STATUS_SAMPLE_INTERVAL_S)

    mean_force = mean_vector(force_samples)
    mean_torque = mean_vector(torque_samples)
    std_force = std_vector(force_samples, mean_force)
    std_torque = std_vector(torque_samples, mean_torque)

    return {
        "mean_force": mean_force,
        "mean_torque": mean_torque,
        "std_force": std_force,
        "std_torque": std_torque,
        "modes": modes,
        "errors": errors,
        "collision_pairs": all_collisions,
    }


def load_snapshot():
    if not SNAPSHOT_PATH.exists():
        raise FileNotFoundError(
            "未找到 pose_debug_snapshot.json。"
            "请先运行 23_pose_debugger_live.py，等待 Baseline READY，"
            "点击产品中心后按 S 保存快照。"
        )

    snapshot = json.loads(SNAPSHOT_PATH.read_text(encoding="utf-8"))
    required = [
        "target_base_xyz_m",
        "pregrasp_base_xyz_m",
        "right_hand_base_xyz_m",
        "motion_status",
        "right_arm_wrench",
    ]
    missing = [name for name in required if name not in snapshot]
    if missing:
        raise RuntimeError(f"快照缺少字段: {missing}")
    return snapshot


def main():
    print("24_move_to_pregrasp_safe.py")
    print("G2 预抓取运动安全门禁测试")
    print("SAFETY_LOCK = True")
    print("当前版本不会发送任何机械臂运动命令")

    snapshot = load_snapshot()

    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK 初始化失败")
        return 1

    robot = agibot_gdk.Robot()
    tf_api = agibot_gdk.TF()
    time.sleep(2.0)

    try:
        checks = []

        target = [float(value) for value in snapshot["target_base_xyz_m"]]
        pregrasp = [float(value) for value in snapshot["pregrasp_base_xyz_m"]]
        hand = read_right_hand_xyz(tf_api)
        delta = vector_subtract(pregrasp, hand)
        move_distance = vector_norm(delta)

        live_status = sample_safety_status(robot)

        add_check(
            checks,
            "snapshot_read_only",
            snapshot.get("read_only") is True,
            f"snapshot.read_only={snapshot.get('read_only')}",
        )
        add_check(
            checks,
            "target_x_workspace",
            in_range(target[0], MIN_X_M, MAX_X_M),
            f"target_x={target[0]:.6f}, range=[{MIN_X_M}, {MAX_X_M}]",
        )
        add_check(
            checks,
            "target_y_workspace",
            in_range(target[1], MIN_Y_M, MAX_Y_M),
            f"target_y={target[1]:.6f}, range=[{MIN_Y_M}, {MAX_Y_M}]",
        )
        add_check(
            checks,
            "pregrasp_z_workspace",
            in_range(pregrasp[2], MIN_Z_M, MAX_Z_M),
            f"pregrasp_z={pregrasp[2]:.6f}, range=[{MIN_Z_M}, {MAX_Z_M}]",
        )
        add_check(
            checks,
            "pregrasp_above_target",
            pregrasp[2] >= target[2] + PREGRASP_OFFSET_Z_M - 0.005,
            f"target_z={target[2]:.6f}, pregrasp_z={pregrasp[2]:.6f}",
        )
        add_check(
            checks,
            "move_distance",
            move_distance <= MAX_MOVE_DISTANCE_M,
            f"distance={move_distance:.6f}, max={MAX_MOVE_DISTANCE_M}",
        )
        add_check(
            checks,
            "motion_error_code",
            all(item["code"] == 0 for item in live_status["errors"]),
            f"errors={live_status['errors']}",
        )
        add_check(
            checks,
            "collision_pairs",
            len(live_status["collision_pairs"]) == 0,
            f"collision_pairs={live_status['collision_pairs']}",
        )
        add_check(
            checks,
            "force_baseline_stable",
            max(live_status["std_force"]) <= MAX_FORCE_STD,
            f"std_force={live_status['std_force']}, max={MAX_FORCE_STD}",
        )
        add_check(
            checks,
            "torque_baseline_stable",
            max(live_status["std_torque"]) <= MAX_TORQUE_STD,
            f"std_torque={live_status['std_torque']}, max={MAX_TORQUE_STD}",
        )

        all_checks_passed = all(item["passed"] for item in checks)

        report = {
            "program": "24_move_to_pregrasp_safe.py",
            "safety_lock": SAFETY_LOCK,
            "movement_command_sent": False,
            "all_preflight_checks_passed": all_checks_passed,
            "target_base_xyz_m": target,
            "pregrasp_base_xyz_m": pregrasp,
            "right_hand_live_base_xyz_m": hand,
            "hand_to_pregrasp_delta_xyz_m": delta,
            "hand_to_pregrasp_distance_m": move_distance,
            "live_safety_status": live_status,
            "checks": checks,
            "blocking_reason": (
                "GDK end_effector_pose_control 文档说明接口本身无碰撞检测，"
                "且当前尚未确认可靠的软件立即停止接口。"
            ),
        }

        REPORT_PATH.write_text(
            json.dumps(report, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        print()
        print("目标位置")
        print(f"  Target Base XYZ = {target}")
        print(f"  PreGrasp XYZ = {pregrasp}")
        print(f"  Right Hand Live XYZ = {hand}")
        print(f"  Delta XYZ = {delta}")
        print(f"  Distance = {move_distance:.6f} m")

        print()
        print("实时 Wrench 基线稳定性")
        print(f"  Mean Force = {live_status['mean_force']}")
        print(f"  Mean Torque = {live_status['mean_torque']}")
        print(f"  Std Force = {live_status['std_force']}")
        print(f"  Std Torque = {live_status['std_torque']}")

        print()
        print("安全门禁检查")
        for item in checks:
            state = "PASS" if item["passed"] else "FAIL"
            print(f"  [{state}] {item['name']}: {item['detail']}")

        print()
        print(f"全部门禁通过: {all_checks_passed}")
        print("movement_command_sent: False")
        print("机械臂未运动")
        print(f"报告已保存: {REPORT_PATH}")

        if all_checks_passed:
            print()
            print("预抓取目标在当前几何和状态门禁下通过。")
            print("但 SAFETY_LOCK 仍保持启用，因为尚未确认立即停止接口。")
        else:
            print()
            print("存在安全门禁失败，不允许进入运动阶段。")

        return 0

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
