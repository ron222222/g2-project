#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""G2 + O10 双臂 TF 只读 Probe v1-fixed。"""
from __future__ import annotations
import json
import math
import os
import sys
import time
import traceback
import subprocess
import shlex
from datetime import datetime
from pathlib import Path
import numpy as np

READ_ONLY_PROBE = True
ENABLE_REAL_MOTION = False
WAIT_SEC = 2.0
RETRIES = 10
RETRY_SEC = 0.5
FRAMES = ("arm_l_end_link", "arm_r_end_link", "hand_l_base_link", "hand_r_base_link")
REPORT_DIR = Path(__file__).resolve().parent / "reports"

ENV_SH = Path.home() / ".cache/agibot/app/env.sh"

def load_gdk_environment():
    """Load env.sh into this Python process before importing agibot_gdk."""
    if not ENV_SH.is_file():
        raise FileNotFoundError("GDK environment script not found: %s" % ENV_SH)
    command = "source %s >/dev/null 2>&1 && env -0" % shlex.quote(str(ENV_SH))
    completed = subprocess.run(["bash", "-lc", command], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
    loaded = 0
    for item in completed.stdout.split(b"\0"):
        if not item or b"=" not in item:
            continue
        key, value = item.split(b"=", 1)
        try:
            os.environ[key.decode()] = value.decode()
            loaded += 1
        except UnicodeDecodeError:
            continue
    return loaded



def xyzq(obj):
    transform = getattr(obj, "transform", obj)
    translation = getattr(transform, "translation")
    rotation = getattr(transform, "rotation")
    p = np.array([float(translation.x), float(translation.y), float(translation.z)])
    q = np.array([float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)])
    return p, q


def quaternion_to_rotation(q):
    x, y, z, w = q
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        raise ValueError("Quaternion norm is zero")
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def make_transform(p, q):
    result = np.eye(4)
    result[:3, :3] = quaternion_to_rotation(q)
    result[:3, 3] = p
    return result


def euler_zyx_deg(rotation):
    pitch = math.asin(max(-1.0, min(1.0, -float(rotation[2, 0]))))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-rotation[0, 1], rotation[1, 1])
    return np.degrees([yaw, pitch, roll])


def describe_transform(transform):
    position = transform[:3, 3]
    rotation = transform[:3, :3]
    return {
        "translation_m": position.tolist(),
        "distance_m": float(np.linalg.norm(position)),
        "rotation_matrix": rotation.tolist(),
        "euler_zyx_deg": euler_zyx_deg(rotation).tolist(),
        "matrix_4x4": transform.tolist(),
    }


def closure_error(forward, reverse):
    cycle = forward @ reverse
    translation_error = float(np.linalg.norm(cycle[:3, 3]))
    cosine = max(-1.0, min(1.0, (float(np.trace(cycle[:3, :3])) - 1.0) / 2.0))
    rotation_error = math.degrees(math.acos(cosine))
    return translation_error, rotation_error, cycle


def read_base_tf(tf_api, frame):
    last_error = None
    for attempt in range(1, RETRIES + 1):
        try:
            value = tf_api.get_tf_from_base_link(frame)
            if value is not None:
                return value, attempt
        except Exception as exc:
            last_error = exc
        if attempt < RETRIES:
            time.sleep(RETRY_SEC)
    raise RuntimeError("Unable to read base_link to %s; last error: %r" % (frame, last_error))


def ensure_gdk_environment_before_import():
    """If needed, source official env.sh in bash and re-exec this script.

    This is required for native shared libraries because changing LD_LIBRARY_PATH
    only inside an already-running Python process is not reliable for dlopen.
    Re-exec starts /usr/bin/python3.10 with the official environment already set.
    """
    if os.environ.get("G2_GDK_ENV_READY") == "1":
        return False
    if not ENV_SH.is_file():
        raise FileNotFoundError("GDK environment script not found: %s" % ENV_SH)
    script_path = str(Path(__file__).resolve())
    argv = [script_path] + sys.argv[1:]
    quoted_args = " ".join(shlex.quote(x) for x in argv)
    command = (
        "source %s && "
        "export G2_GDK_ENV_READY=1 && "
        "exec /usr/bin/python3.10 %s"
    ) % (shlex.quote(str(ENV_SH)), quoted_args)
    print("[ENV] Restarting current probe with official GDK environment ...", flush=True)
    os.execv("/bin/bash", ["bash", "-lc", command])
    return True


def main():
    ensure_gdk_environment_before_import()
    print("=" * 92)
    print("G2 O10 Dual Arm TF Read-Only Probe")
    print("Version             : 102_v3_auto_env_reexec")
    print("Python executable   :", sys.executable)
    print("ROS_DISTRO          :", os.getenv("ROS_DISTRO", "<not set>"))
    print("READ_ONLY_PROBE     :", READ_ONLY_PROBE)
    print("ENABLE_REAL_MOTION  :", ENABLE_REAL_MOTION)
    print("Motion API          : NONE")
    print("=" * 92)

    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION:
        print("[FATAL] Safety lock invalid.")
        return 90
    print("[ENV] Official GDK environment : READY")
    print("[ENV] ROS_DISTRO               :", os.getenv("ROS_DISTRO", "<not set>"))
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

        tf_api = agibot_gdk.TF()
        print("[GDK] TF object created [read only]")
        print("[DDS] Waiting %.1f seconds" % WAIT_SEC)
        time.sleep(WAIT_SEC)

        transforms = {}
        base_records = {}
        for frame in FRAMES:
            raw, attempt = read_base_tf(tf_api, frame)
            p, q = xyzq(raw)
            transform = make_transform(p, q)
            transforms[frame] = transform
            base_records[frame] = {
                "attempt": attempt,
                "translation_m": p.tolist(),
                "quaternion_xyzw": q.tolist(),
                "derived": describe_transform(transform),
            }
            print()
            print("[TF] base_link -> %s attempt=%d/%d" % (frame, attempt, RETRIES))
            print("     XYZ [m]        :", np.array2string(p, precision=6))
            print("     Quaternion xyzw:", np.array2string(q, precision=7))
            print("     Euler ZYX [deg]:", np.array2string(euler_zyx_deg(transform[:3, :3]), precision=4))

        pair_specs = (
            ("arm_l_end_link", "arm_r_end_link", "LEFT_END_TO_RIGHT_END"),
            ("hand_l_base_link", "hand_r_base_link", "LEFT_HAND_TO_RIGHT_HAND"),
        )
        relative_records = {}
        all_translation_errors = []
        all_rotation_errors = []

        for left, right, label in pair_specs:
            forward = np.linalg.inv(transforms[left]) @ transforms[right]
            reverse = np.linalg.inv(transforms[right]) @ transforms[left]
            translation_error, rotation_error, cycle = closure_error(forward, reverse)
            all_translation_errors.append(translation_error)
            all_rotation_errors.append(rotation_error)
            relative_records[label] = {
                "forward": describe_transform(forward),
                "reverse": describe_transform(reverse),
                "closure_translation_error_m": translation_error,
                "closure_rotation_error_deg": rotation_error,
                "closure_matrix": cycle.tolist(),
            }
            print()
            print("-" * 92)
            print("[RELATIVE]", label)
            print("  %s -> %s XYZ [m]:" % (left, right), np.array2string(forward[:3, 3], precision=6))
            print("  distance [m]            : %.6f" % np.linalg.norm(forward[:3, 3]))
            print("  Euler ZYX [deg]         :", np.array2string(euler_zyx_deg(forward[:3, :3]), precision=4))
            print("  closure translation [m] : %.3e" % translation_error)
            print("  closure rotation [deg]   : %.3e" % rotation_error)

        maximum_translation_error = max(all_translation_errors)
        maximum_rotation_error = max(all_rotation_errors)
        overall_ok = maximum_translation_error < 1e-6 and maximum_rotation_error < 1e-6

        report = {
            "program": "G2 O10 Dual Arm TF Read-Only Probe",
            "version": "102_v3_auto_env_reexec",
            "timestamp_local": datetime.now().astimezone().isoformat(),
            "safety": {"read_only_probe": True, "enable_real_motion": False, "motion_executed": False},
            "environment": {"python": sys.executable, "ros_distro": os.getenv("ROS_DISTRO"), "agibot_gdk_module": module_path},
            "base_transforms": base_records,
            "relative_transforms": relative_records,
            "maximum_closure_translation_error_m": maximum_translation_error,
            "maximum_closure_rotation_error_deg": maximum_rotation_error,
            "overall_ok": overall_ok,
        }
        REPORT_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = REPORT_DIR / ("102_dual_arm_tf_probe_v3_auto_env_reexec_%s.json" % timestamp)
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

        print()
        print("=" * 92)
        print("PROBE SUMMARY")
        print("=" * 92)
        print("Base TF left end      : PASS")
        print("Base TF right end     : PASS")
        print("Base TF left hand     : PASS")
        print("Base TF right hand    : PASS")
        print("Relative transforms   : PASS")
        print("Transform closure     :", "PASS" if overall_ok else "CHECK REQUIRED")
        print("Overall               :", "PASS" if overall_ok else "CHECK REQUIRED")
        print("JSON report           :", report_path)
        print("Motion executed       : NO")
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
        print("[SAFETY] Probe stopped. No motion command was issued.")
        raise SystemExit(1)
