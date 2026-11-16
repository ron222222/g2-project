#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
103_g2_o10_dual_arm_tcp_probe_v1.py

G2 + OmniHand O10 双臂 TCP 只读 Probe v1

功能：
1. 自动加载 ~/.cache/agibot/app/env.sh，并在官方 GDK 环境下重新启动自身。
2. 初始化 GDK 4.1.5。
3. 只读调用 Robot.get_tcp_transform()。
4. 尝试只读调用 Robot.get_end_pose_control_frame()。
5. 输出左右臂当前 TCP 相对各自 arm_*_end_link 的 position 和 euler_zyx。
6. 保存 JSON 报告。

安全保证：
- 不调用 set_tcp_transform()。
- 不调用 switch_end_pose_control_frame()。
- 不调用 set_control_mode()。
- 不调用任何 move_* 或 end_effector_pose_control()。
- 不控制双臂、O10、腰部或底盘。
"""
from __future__ import annotations

import json
import math
import os
import shlex
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

READ_ONLY_PROBE = True
ENABLE_REAL_MOTION = False
PROGRAM_VERSION = "103_v1_auto_env_reexec"
WAIT_SEC = 2.0
RETRIES = 10
RETRY_SEC = 0.5
ENV_SH = Path.home() / ".cache/agibot/app/env.sh"
REPORT_DIR = Path(__file__).resolve().parent / "reports"


def ensure_gdk_environment_before_import() -> None:
    """在导入 native GDK 模块前，用官方 env.sh 重新启动当前程序。"""
    if os.environ.get("G2_GDK_ENV_READY") == "1":
        return
    if not ENV_SH.is_file():
        raise FileNotFoundError("GDK environment script not found: %s" % ENV_SH)

    script_path = str(Path(__file__).resolve())
    arguments = [script_path] + sys.argv[1:]
    quoted_arguments = " ".join(shlex.quote(item) for item in arguments)
    command = (
        "source %s && "
        "export G2_GDK_ENV_READY=1 && "
        "exec /usr/bin/python3.10 %s"
    ) % (shlex.quote(str(ENV_SH)), quoted_arguments)

    print("[ENV] Restarting current probe with official GDK environment ...", flush=True)
    os.execv("/bin/bash", ["bash", "-lc", command])


def field(obj: Any, name: str, default: Any = None) -> Any:
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def serializable(value: Any, depth: int = 0) -> Any:
    if depth > 6:
        return repr(value)
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(k): serializable(v, depth + 1) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [serializable(v, depth + 1) for v in value]
    if hasattr(value, "__dict__"):
        try:
            return {
                str(k): serializable(v, depth + 1)
                for k, v in vars(value).items()
                if not str(k).startswith("_")
            }
        except Exception:
            pass
    return repr(value)


def vector3(value: Any, label: str) -> list[float]:
    """兼容 list/tuple/numpy-like 或带 x/y/z 属性的三维向量。"""
    if value is None:
        raise ValueError("%s is missing" % label)
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        result = [float(value[0]), float(value[1]), float(value[2])]
    elif all(hasattr(value, axis) for axis in ("x", "y", "z")):
        result = [float(value.x), float(value.y), float(value.z)]
    else:
        try:
            result = [float(value[0]), float(value[1]), float(value[2])]
        except Exception as exc:
            raise TypeError("Unable to parse %s: %r" % (label, value)) from exc
    if not all(math.isfinite(item) for item in result):
        raise ValueError("%s contains NaN/Inf: %r" % (label, result))
    return result


def parse_arm_tcp(arm_obj: Any, arm_name: str) -> Dict[str, Any]:
    position = vector3(field(arm_obj, "position"), arm_name + ".position")
    euler_zyx = vector3(field(arm_obj, "euler_zyx"), arm_name + ".euler_zyx")
    distance = math.sqrt(sum(item * item for item in position))
    return {
        "position_m": position,
        "position_norm_m": distance,
        "euler_zyx_rad": euler_zyx,
        "euler_zyx_deg": [math.degrees(item) for item in euler_zyx],
        "finite": True,
        "raw": serializable(arm_obj),
    }


def read_tcp_with_retry(robot: Any) -> tuple[Any, int]:
    last_error: Optional[Exception] = None
    for attempt in range(1, RETRIES + 1):
        try:
            result = robot.get_tcp_transform()
            if result is not None:
                return result, attempt
        except Exception as exc:
            last_error = exc
        if attempt < RETRIES:
            time.sleep(RETRY_SEC)
    raise RuntimeError(
        "get_tcp_transform() failed after %d attempts; last error: %r"
        % (RETRIES, last_error)
    )


def main() -> int:
    ensure_gdk_environment_before_import()

    print("=" * 92)
    print("G2 O10 Dual Arm TCP Read-Only Probe")
    print("Version             :", PROGRAM_VERSION)
    print("Python executable   :", sys.executable)
    print("ROS_DISTRO          :", os.getenv("ROS_DISTRO", "<not set>"))
    print("READ_ONLY_PROBE     :", READ_ONLY_PROBE)
    print("ENABLE_REAL_MOTION  :", ENABLE_REAL_MOTION)
    print("Motion API          : NONE")
    print("=" * 92)

    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION:
        print("[FATAL] Safety lock invalid. Probe aborted.")
        return 90

    print("[ENV] Official GDK environment : READY")
    print("[ENV] PYTHONPATH               :", os.getenv("PYTHONPATH", "<not set>"))
    print("[ENV] LD_LIBRARY_PATH          :", os.getenv("LD_LIBRARY_PATH", "<not set>"))

    import agibot_gdk

    module_path = getattr(agibot_gdk, "__file__", "") or ""
    print("[GDK] module path       :", module_path)
    if "/.cache/agibot/app/gdk/lib/agibot_gdk/" not in module_path:
        print("[FATAL] Wrong GDK runtime. Probe stopped.")
        return 92

    initialized = False
    try:
        init_result = agibot_gdk.gdk_init()
        print("[GDK] gdk_init result   :", init_result)
        if init_result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("gdk_init failed: %s" % init_result)
        initialized = True

        robot = agibot_gdk.Robot()
        print("[GDK] Robot created [read only]")
        print("[DDS] Waiting %.1f seconds" % WAIT_SEC)
        time.sleep(WAIT_SEC)

        raw_tcp, attempt = read_tcp_with_retry(robot)
        print("[TCP] get_tcp_transform(): attempt %d/%d" % (attempt, RETRIES))
        print("[TCP] raw return type    :", type(raw_tcp).__name__)

        left_obj = field(raw_tcp, "left_arm")
        right_obj = field(raw_tcp, "right_arm")
        if left_obj is None or right_obj is None:
            raise ValueError(
                "TCP result does not contain left_arm/right_arm. Raw result: %r"
                % serializable(raw_tcp)
            )

        left = parse_arm_tcp(left_obj, "left_arm")
        right = parse_arm_tcp(right_obj, "right_arm")

        control_frame = None
        control_frame_status = "API_NOT_AVAILABLE"
        if hasattr(robot, "get_end_pose_control_frame"):
            try:
                control_frame = robot.get_end_pose_control_frame()
                control_frame_status = "PASS"
            except Exception as exc:
                control_frame_status = "%s: %s" % (type(exc).__name__, exc)

        print()
        print("-" * 92)
        print("[LEFT TCP] relative to arm_l_end_link")
        print("  Position XYZ [m]     :", left["position_m"])
        print("  Offset norm [m]      : %.6f" % left["position_norm_m"])
        print("  Euler ZYX [rad]      :", left["euler_zyx_rad"])
        print("  Euler ZYX [deg]      :", [round(v, 6) for v in left["euler_zyx_deg"]])

        print()
        print("-" * 92)
        print("[RIGHT TCP] relative to arm_r_end_link")
        print("  Position XYZ [m]     :", right["position_m"])
        print("  Offset norm [m]      : %.6f" % right["position_norm_m"])
        print("  Euler ZYX [rad]      :", right["euler_zyx_rad"])
        print("  Euler ZYX [deg]      :", [round(v, 6) for v in right["euler_zyx_deg"]])

        print()
        print("-" * 92)
        print("[END POSE CONTROL FRAME]")
        print("  Read status          :", control_frame_status)
        print("  Current frame        :", serializable(control_frame))

        # 这是只读事实检查，不对 TCP 是否适合 O10 抓盆作自动结论。
        overall_ok = left["finite"] and right["finite"]
        report = {
            "program": "G2 O10 Dual Arm TCP Read-Only Probe",
            "version": PROGRAM_VERSION,
            "timestamp_local": datetime.now().astimezone().isoformat(),
            "safety": {
                "read_only_probe": True,
                "enable_real_motion": False,
                "motion_executed": False,
                "tcp_modified": False,
                "control_frame_modified": False,
            },
            "environment": {
                "python": sys.executable,
                "ros_distro": os.getenv("ROS_DISTRO"),
                "agibot_gdk_module": module_path,
            },
            "tcp_read_attempt": attempt,
            "left_tcp_relative_to_arm_l_end_link": left,
            "right_tcp_relative_to_arm_r_end_link": right,
            "end_pose_control_frame_status": control_frame_status,
            "end_pose_control_frame": serializable(control_frame),
            "raw_tcp_transform": serializable(raw_tcp),
            "overall_ok": overall_ok,
        }

        REPORT_DIR.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = REPORT_DIR / ("103_dual_arm_tcp_probe_v1_%s.json" % stamp)
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

        print()
        print("=" * 92)
        print("PROBE SUMMARY")
        print("=" * 92)
        print("Left TCP read         : PASS")
        print("Right TCP read        : PASS")
        print("TCP values finite     : PASS")
        print("Control frame read    :", control_frame_status)
        print("Overall               :", "PASS" if overall_ok else "CHECK REQUIRED")
        print("TCP modified          : NO")
        print("Control frame modified: NO")
        print("Motion executed       : NO")
        print("JSON report           :", report_path)
        print("=" * 92)
        return 0 if overall_ok else 2

    finally:
        if initialized:
            release_result = agibot_gdk.gdk_release()
            print()
            print("[GDK] gdk_release result :", release_result)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print()
        print("[STOP] User interrupted. No motion command was issued.")
        raise SystemExit(130)
    except Exception as exc:
        print()
        print("[FATAL] %s: %s" % (type(exc).__name__, exc))
        traceback.print_exc()
        print("[SAFETY] Probe stopped. No motion or TCP-setting command was issued.")
        raise SystemExit(1)
