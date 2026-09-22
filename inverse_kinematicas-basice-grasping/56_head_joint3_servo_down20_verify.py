#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
56_head_joint3_servo_down20_verify.py

使用 joint_servo_control() 以100 Hz单独控制 idx13_head_joint3 到-20度。
不再使用多次 move_head_joint()。
不控制机械臂、夹爪、底盘，也不改变 idx11/idx12。

默认 ENABLE_REAL_MOTION=False。
"""

import json
import math
import time
from pathlib import Path

import agibot_gdk

ENABLE_REAL_MOTION = True
TARGET_DEG = -20.0
DURATION_S = 2.5
RATE_HZ = 100.0
CONTROL_PERIOD_S = 0.012
FINAL_TOLERANCE_DEG = 1.5
MAX_START_ERROR_DEG = 45.0
REPORT_FILE = "head_joint3_servo_down20_verify_report.json"
JOINT_NAME = "idx13_head_joint3"


def save(report):
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def get_state(robot):
    states = {s["name"]: s for s in robot.get_joint_states()["states"]}
    if JOINT_NAME not in states:
        raise RuntimeError(f"缺少关节状态: {JOINT_NAME}")
    state = states[JOINT_NAME]
    return {
        "position_rad": float(state["motor_position"]),
        "position_deg": math.degrees(float(state["motor_position"])),
        "velocity_rad_s": float(state["motor_velocity"]),
        "effort": float(state["effort"]),
        "error_code": int(state["error_code"]),
    }


def smoothstep(t):
    return t*t*(3.0-2.0*t)


def send_servo(robot, position_rad):
    req = agibot_gdk.JointServoControlReq()
    req.control_period = CONTROL_PERIOD_S
    req.joint_names = [JOINT_NAME]
    req.joint_positions = [position_rad]
    req.joint_velocities = []
    result = robot.joint_servo_control(req)
    if result != 0:
        raise RuntimeError(f"joint_servo_control失败: {result}")


def main():
    report = {
        "program": "56_head_joint3_servo_down20_verify.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "target_deg": TARGET_DEG,
        "duration_s": DURATION_S,
        "rate_hz": RATE_HZ,
        "status": "INITIALIZED",
    }
    initialized = False
    try:
        if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("GDK初始化失败")
        initialized = True
        robot = agibot_gdk.Robot()
        time.sleep(2.0)

        before = get_state(robot)
        target_rad = math.radians(TARGET_DEG)
        start_rad = before["position_rad"]
        total_error_deg = abs(before["position_deg"] - TARGET_DEG)
        report["before"] = before
        save(report)

        print("=" * 76)
        print("56_head_joint3_servo_down20_verify.py")
        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print(f"当前 idx13 = {before['position_deg']:.3f}deg")
        print(f"目标 idx13 = {TARGET_DEG:.3f}deg")
        print(f"规划时长 = {DURATION_S:.2f}s, 发送频率 = {RATE_HZ:.1f}Hz")
        print("仅对 idx13_head_joint3 发送正常模式关节伺服命令。")
        print("=" * 76)

        if before["error_code"] != 0:
            raise RuntimeError(f"idx13存在错误码: {before['error_code']}")
        if total_error_deg > MAX_START_ERROR_DEG:
            raise RuntimeError(f"起始角度距离目标过大: {total_error_deg:.2f}deg")

        if not ENABLE_REAL_MOTION:
            report["status"] = "DRY_RUN_PASS"
            save(report)
            print("DRY RUN通过，未发送运动命令。")
            return

        if input("确认头部周围无障碍，输入 SERVO：").strip() != "SERVO":
            report["status"] = "CANCELLED_BY_USER"
            save(report)
            return

        steps = max(2, int(round(DURATION_S * RATE_HZ)))
        dt = 1.0 / RATE_HZ
        start_time = time.monotonic()

        for i in range(steps):
            linear_t = (i + 1) / steps
            alpha = smoothstep(linear_t)
            command = start_rad + alpha * (target_rad - start_rad)
            send_servo(robot, command)

            expected = start_time + (i + 1) * dt
            sleep_s = expected - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)

        # 在最终目标保持0.5秒，正常模式，不启用低延时。
        hold_steps = int(0.5 * RATE_HZ)
        for _ in range(hold_steps):
            send_servo(robot, target_rad)
            time.sleep(dt)

        time.sleep(0.3)
        after = get_state(robot)
        error_deg = abs(after["position_deg"] - TARGET_DEG)
        report["after"] = after
        report["final_error_deg"] = error_deg

        print(f"运动后 idx13 = {after['position_deg']:.3f}deg")
        print(f"最终误差 = {error_deg:.3f}deg")

        if after["error_code"] != 0:
            report["status"] = "HEAD_JOINT_ERROR"
            save(report)
            raise RuntimeError(f"运动后idx13错误码: {after['error_code']}")
        if error_deg > FINAL_TOLERANCE_DEG:
            report["status"] = "TARGET_NOT_REACHED"
            save(report)
            raise RuntimeError("idx13关节伺服未达到-20度")

        report["status"] = "HEAD_TARGET_REACHED"
        save(report)
        print("idx13_head_joint3低头20度验证通过。")

    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED"
        report["error"] = "KeyboardInterrupt"
        save(report)
        print("用户中断。")
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
