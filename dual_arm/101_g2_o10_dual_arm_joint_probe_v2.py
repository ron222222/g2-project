#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
101_g2_o10_dual_arm_joint_probe_v2.py

G2 + OmniHand O10 双臂 Joint 只读 Probe v2

运行前必须先加载 GDK 4.1.5 官方环境：
    source ~/.cache/agibot/app/env.sh

然后运行：
    /usr/bin/python3.10 101_g2_o10_dual_arm_joint_probe_v2.py

本版本安全约束：
- 只初始化 GDK、创建 Robot、读取 Whole Body Status 和 Joint States。
- 不包含任何 move_*、end_effector_pose_control、move_ee_pos、set_control_mode 等运动 API。
- 不控制双臂、O10、腰部或底盘。
- 不修改 TCP、TF、Joint、控制模式或设备参数。
- Robot 创建后等待 DDS 建链，再读取状态。
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

# -----------------------------------------------------------------------------
# 0. 安全常量
# -----------------------------------------------------------------------------
READ_ONLY_PROBE = True
ENABLE_REAL_MOTION = False
PROGRAM_VERSION = "101_v2"
PROGRAM_NAME = "G2 O10 Dual Arm Joint Read-Only Probe"
DDS_INITIAL_WAIT_SEC = 2.0
JOINT_READ_MAX_ATTEMPTS = 10
JOINT_READ_RETRY_SEC = 1.0

LEFT_ARM_JOINTS = [
    "idx21_arm_l_joint1",
    "idx22_arm_l_joint2",
    "idx23_arm_l_joint3",
    "idx24_arm_l_joint4",
    "idx25_arm_l_joint5",
    "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
]

RIGHT_ARM_JOINTS = [
    "idx61_arm_r_joint1",
    "idx62_arm_r_joint2",
    "idx63_arm_r_joint3",
    "idx64_arm_r_joint4",
    "idx65_arm_r_joint5",
    "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]

URDF_CANDIDATES = [
    "G2_t2_crs_o10_t2.urdf",
    "G2_t2_crs_o10_t2.urdf.txt",
]

REPORT_DIR = Path(__file__).resolve().parent / "reports"


def banner() -> None:
    print("=" * 92)
    print(PROGRAM_NAME)
    print(f"Version             : {PROGRAM_VERSION}")
    print(f"Python executable   : {sys.executable}")
    print(f"Python version      : {sys.version.split()[0]}")
    print(f"ROS_DISTRO          : {os.environ.get('ROS_DISTRO', '<not set>')}")
    print(f"READ_ONLY_PROBE     : {READ_ONLY_PROBE}")
    print(f"ENABLE_REAL_MOTION  : {ENABLE_REAL_MOTION}")
    print("Motion API          : NONE")
    print("=" * 92)


def field(obj: Any, name: str, default: Any = None) -> Any:
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def first_field(obj: Any, names: Iterable[str]) -> Tuple[Optional[str], Any]:
    for name in names:
        value = field(obj, name, None)
        if value is not None:
            return name, value
    return None, None


def safe_float(value: Any) -> Optional[float]:
    try:
        if value is None:
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def finite(value: Any) -> Optional[bool]:
    x = safe_float(value)
    return None if x is None else math.isfinite(x)


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
            return {str(k): serializable(v, depth + 1)
                    for k, v in vars(value).items()
                    if not str(k).startswith("_")}
        except Exception:
            pass
    return repr(value)


def find_urdf() -> Optional[Path]:
    base = Path(__file__).resolve().parent
    for name in URDF_CANDIDATES:
        p = base / name
        if p.is_file():
            return p
    return None


def load_arm_limits(path: Path) -> Dict[str, Dict[str, Optional[float]]]:
    import xml.etree.ElementTree as ET
    root = ET.fromstring(path.read_text(encoding="utf-8"))
    wanted = set(LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS)
    limits: Dict[str, Dict[str, Optional[float]]] = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        if name not in wanted:
            continue
        lim = joint.find("limit")
        limits[name] = {
            "type": joint.attrib.get("type"),
            "lower": safe_float(lim.attrib.get("lower")) if lim is not None else None,
            "upper": safe_float(lim.attrib.get("upper")) if lim is not None else None,
            "effort": safe_float(lim.attrib.get("effort")) if lim is not None else None,
            "velocity": safe_float(lim.attrib.get("velocity")) if lim is not None else None,
        }
    return limits


def normalize_joint_states(raw: Any) -> Tuple[List[Any], Dict[str, Any]]:
    """兼容 GDK 4.1.5 文档结构 {timestamp, nums, states}，同时保留防御性兼容。"""
    metadata: Dict[str, Any] = {}
    if isinstance(raw, dict):
        metadata["timestamp"] = serializable(raw.get("timestamp"))
        metadata["nums"] = serializable(raw.get("nums"))
        states = raw.get("states")
        if isinstance(states, (list, tuple)):
            return list(states), metadata
    if isinstance(raw, (list, tuple)):
        return list(raw), metadata
    for key in ("states", "joint_states", "joints", "data"):
        candidate = field(raw, key, None)
        if isinstance(candidate, (list, tuple)):
            metadata["timestamp"] = serializable(field(raw, "timestamp", None))
            metadata["nums"] = serializable(field(raw, "nums", None))
            return list(candidate), metadata
    raise TypeError("无法识别 get_joint_states() 返回结构，请保留完整控制台输出。")


def joint_name(state: Any) -> Optional[str]:
    _, value = first_field(state, ("name", "joint_name"))
    return str(value) if value is not None else None


def extract_record(state: Any) -> Dict[str, Any]:
    # 4.1.5 优先读取 motor_position / motor_velocity，同时保留原始 position/velocity。
    _, motor_pos = first_field(state, ("motor_position",))
    _, motor_vel = first_field(state, ("motor_velocity",))
    _, position = first_field(state, ("position",))
    _, velocity = first_field(state, ("velocity",))
    effort_key, effort = first_field(state, ("effort", "motor_torque", "torque"))
    _, current = first_field(state, ("motor_current", "current"))
    _, mode = first_field(state, ("mode",))
    _, error_code = first_field(state, ("error_code", "status", "state"))

    # Joint limit 比较优先使用 motor_position；不存在时再退回 position。
    selected_position = safe_float(motor_pos)
    selected_position_source = "motor_position"
    if selected_position is None:
        selected_position = safe_float(position)
        selected_position_source = "position" if selected_position is not None else None

    return {
        "name": joint_name(state),
        "mode": serializable(mode),
        "position": safe_float(position),
        "velocity": safe_float(velocity),
        "motor_position": safe_float(motor_pos),
        "motor_velocity": safe_float(motor_vel),
        "motor_current": safe_float(current),
        "effort_field": effort_key,
        "effort": safe_float(effort),
        "error_code": serializable(error_code),
        "selected_position": selected_position,
        "selected_position_source": selected_position_source,
        "selected_position_finite": finite(selected_position),
        "motor_velocity_finite": finite(motor_vel),
        "effort_finite": finite(effort),
        "raw": serializable(state),
    }


def index_states(states: List[Any]) -> Tuple[Dict[str, Any], List[str], List[int]]:
    index: Dict[str, Any] = {}
    duplicates: List[str] = []
    unnamed: List[int] = []
    for i, state in enumerate(states):
        name = joint_name(state)
        if not name:
            unnamed.append(i)
            continue
        if name in index:
            duplicates.append(name)
        else:
            index[name] = state
    return index, sorted(set(duplicates)), unnamed


def check_arm(label: str, expected: List[str], index: Dict[str, Any],
              limits: Dict[str, Dict[str, Optional[float]]]) -> Dict[str, Any]:
    rows = []
    missing = []
    arm_ok = True
    for seq, name in enumerate(expected, start=1):
        state = index.get(name)
        if state is None:
            missing.append(name)
            rows.append({"sequence": seq, "name": name, "found": False})
            arm_ok = False
            continue
        rec = extract_record(state)
        rec.update({"sequence": seq, "found": True})
        lim = limits.get(name)
        rec["urdf_limit"] = lim
        within = None
        p = rec["selected_position"]
        if lim is not None and p is not None:
            lo, hi = lim.get("lower"), lim.get("upper")
            if lo is not None and hi is not None:
                within = lo <= p <= hi
        rec["within_urdf_limit"] = within
        if rec["selected_position_finite"] is False or within is False:
            arm_ok = False
        rows.append(rec)
    return {
        "arm": label,
        "expected_joint_count": 7,
        "found_joint_count": 7 - len(missing),
        "missing_joints": missing,
        "ok": arm_ok and not missing,
        "joints": rows,
    }


def fnum(v: Any, digits: int = 6) -> str:
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    return "N/A" if v is None else str(v)


def print_arm(result: Dict[str, Any]) -> None:
    print("\n" + "-" * 92)
    print(f"[{result['arm']}] FOUND {result['found_joint_count']}/7")
    print("-" * 92)
    for j in result["joints"]:
        if not j.get("found"):
            print(f"[MISSING] J{j['sequence']} {j['name']}")
            continue
        lim = j.get("urdf_limit") or {}
        print(f"J{j['sequence']}  {j['name']}")
        print(f"    motor_position : {fnum(j.get('motor_position'))} rad")
        print(f"    motor_velocity : {fnum(j.get('motor_velocity'))}")
        print(f"    motor_current  : {fnum(j.get('motor_current'))}")
        print(f"    effort         : {fnum(j.get('effort'))}  source={j.get('effort_field') or 'N/A'}")
        print(f"    mode           : {fnum(j.get('mode'))}")
        print(f"    error_code     : {fnum(j.get('error_code'))}")
        print(f"    selected pos   : {fnum(j.get('selected_position'))}  source={j.get('selected_position_source') or 'N/A'}")
        print(f"    URDF limit     : [{fnum(lim.get('lower'))}, {fnum(lim.get('upper'))}]  within={fnum(j.get('within_urdf_limit'))}")


def read_joint_states_with_retry(robot: Any) -> Any:
    last_error: Optional[Exception] = None
    for attempt in range(1, JOINT_READ_MAX_ATTEMPTS + 1):
        try:
            raw = robot.get_joint_states()
            print(f"[DDS] Joint states received: attempt {attempt}/{JOINT_READ_MAX_ATTEMPTS}")
            return raw
        except RuntimeError as exc:
            last_error = exc
            print(f"[DDS] attempt {attempt}/{JOINT_READ_MAX_ATTEMPTS}: {exc}")
            if attempt < JOINT_READ_MAX_ATTEMPTS:
                time.sleep(JOINT_READ_RETRY_SEC)
    raise RuntimeError(
        f"Joint States 连续 {JOINT_READ_MAX_ATTEMPTS} 次读取失败；最后错误: {last_error}. "
        "本 Probe 到此停止，不执行任何运动。"
    )


def main() -> int:
    banner()
    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION:
        print("[FATAL] Safety lock invalid. Probe aborted.")
        return 90

    # 先验证官方 env.sh 是否真正生效。最关键的是 PYTHONPATH 中应有 app/gdk/lib。
    pythonpath = os.environ.get("PYTHONPATH", "")
    expected_hint = "/.cache/agibot/app/gdk/lib"
    if expected_hint not in pythonpath:
        print("[FATAL] 未检测到 GDK 4.1.5 官方 PYTHONPATH。")
        print("        请先在当前 Terminal 执行：")
        print("        source ~/.cache/agibot/app/env.sh")
        print("        然后再使用 /usr/bin/python3.10 运行本程序。")
        return 91

    import agibot_gdk

    module_path = getattr(agibot_gdk, "__file__", "") or ""
    print(f"[GDK] module path       : {module_path}")
    if "/.cache/agibot/app/gdk/lib/agibot_gdk/" not in module_path:
        print("[FATAL] 当前加载的 agibot_gdk 不是官方 env.sh 指向的 GDK runtime。")
        print("        为避免旧版/新版 GDK 混用，本 Probe 拒绝继续。")
        return 92

    urdf_path = find_urdf()
    limits: Dict[str, Dict[str, Optional[float]]] = {}
    if urdf_path:
        try:
            limits = load_arm_limits(urdf_path)
            print(f"[URDF] loaded            : {urdf_path}")
            print(f"[URDF] arm limits found  : {len(limits)}/14")
        except Exception as exc:
            print(f"[WARN] URDF parse failed : {exc}")
    else:
        print("[INFO] 未在脚本目录找到最新版 URDF，本次 Joint 读取仍继续；limit 显示 N/A。")

    gdk_initialized = False
    report: Dict[str, Any] = {}
    exit_code = 1

    try:
        print("\n[GDK] gdk_init() ...")
        init_result = agibot_gdk.gdk_init()
        print(f"[GDK] gdk_init result    : {init_result}")
        if init_result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError(f"gdk_init failed: {init_result}")
        gdk_initialized = True

        print("[GDK] Creating Robot() [no motion] ...")
        robot = agibot_gdk.Robot()

        print(f"[DDS] Waiting {DDS_INITIAL_WAIT_SEC:.1f}s for initial discovery/data ...")
        time.sleep(DDS_INITIAL_WAIT_SEC)

        whole_body_status = None
        try:
            whole_body_status = robot.get_whole_body_status()
            print("[STATUS] get_whole_body_status(): OK")
        except Exception as exc:
            # 状态读取失败不阻断 Joint Probe，但必须完整记录。
            print(f"[WARN] get_whole_body_status() failed: {type(exc).__name__}: {exc}")

        print("[JOINT] Reading robot.get_joint_states() [READ ONLY] ...")
        raw = read_joint_states_with_retry(robot)
        print(f"[JOINT] raw return type  : {type(raw).__name__}")

        states, joint_meta = normalize_joint_states(raw)
        print(f"[JOINT] total states     : {len(states)}")
        print(f"[JOINT] reported nums    : {joint_meta.get('nums')}")

        index, duplicates, unnamed = index_states(states)
        left = check_arm("LEFT_ARM", LEFT_ARM_JOINTS, index, limits)
        right = check_arm("RIGHT_ARM", RIGHT_ARM_JOINTS, index, limits)
        print_arm(left)
        print_arm(right)

        expected_set = set(LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS)
        all_names = [joint_name(s) for s in states]
        other_names = [n for n in all_names if n and n not in expected_set]

        overall_ok = (
            left["ok"] and right["ok"] and not duplicates and
            left["found_joint_count"] == 7 and right["found_joint_count"] == 7
        )

        report = {
            "program": PROGRAM_NAME,
            "version": PROGRAM_VERSION,
            "timestamp_local": datetime.now().astimezone().isoformat(),
            "safety": {
                "read_only_probe": True,
                "enable_real_motion": False,
                "movement_commands_in_program": False,
            },
            "environment": {
                "python_executable": sys.executable,
                "python_version": sys.version,
                "ros_distro": os.environ.get("ROS_DISTRO"),
                "pythonpath": os.environ.get("PYTHONPATH"),
                "ld_library_path": os.environ.get("LD_LIBRARY_PATH"),
                "agibot_gdk_module": module_path,
                "agibot_gdk_version": getattr(agibot_gdk, "__version__", None),
            },
            "gdk": {
                "init_result": str(init_result),
                "dds_initial_wait_sec": DDS_INITIAL_WAIT_SEC,
                "whole_body_status": serializable(whole_body_status),
            },
            "urdf": {
                "path": str(urdf_path) if urdf_path else None,
                "arm_limits_loaded": len(limits),
            },
            "joint_state_metadata": joint_meta,
            "joint_state": {
                "total_returned": len(states),
                "duplicate_names": duplicates,
                "unnamed_entries": unnamed,
                "other_joint_names": other_names,
            },
            "left_arm": left,
            "right_arm": right,
            "overall_ok": overall_ok,
        }

        REPORT_DIR.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = REPORT_DIR / f"101_dual_arm_joint_probe_v2_{stamp}.json"
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

        print("\n" + "=" * 92)
        print("PROBE SUMMARY")
        print("=" * 92)
        print(f"GDK runtime           : PASS")
        print(f"Joint states          : PASS")
        print(f"Left arm              : {left['found_joint_count']}/7")
        print(f"Right arm             : {right['found_joint_count']}/7")
        print(f"Duplicate joint names : {duplicates if duplicates else 'NONE'}")
        print(f"Unnamed states        : {unnamed if unnamed else 'NONE'}")
        print(f"Overall               : {'PASS' if overall_ok else 'CHECK REQUIRED'}")
        print(f"JSON report           : {report_path}")
        print("Motion executed       : NO")
        print("=" * 92)
        exit_code = 0 if overall_ok else 2

    finally:
        if gdk_initialized:
            try:
                print("\n[GDK] gdk_release() ...")
                release_result = agibot_gdk.gdk_release()
                print(f"[GDK] gdk_release result : {release_result}")
            except Exception as exc:
                print(f"[WARN] gdk_release failed: {type(exc).__name__}: {exc}")

    return exit_code


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[STOP] User interrupted. No motion command was issued.")
        raise SystemExit(130)
    except Exception as exc:
        print(f"\n[FATAL] {type(exc).__name__}: {exc}")
        traceback.print_exc()
        print("[SAFETY] Probe stopped. No motion command was issued.")
        raise SystemExit(1)
