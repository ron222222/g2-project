#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
101_g2_o10_dual_arm_joint_probe_v1.py

G2 + OmniHand O10 双臂第一阶段只读 Joint Probe

目标：
1. 使用 agibot_gdk.Robot() 读取整机 Joint States。
2. 按最新版 G2 URDF 中确认的名称提取左臂 7 轴和右臂 7 轴。
3. 检查关节是否缺失、重复、数值是否为有限值。
4. 将当前关节位置与最新版 URDF 的 Joint Limit 做只读比较。
5. 输出控制台摘要，并保存 JSON 报告。

安全原则：
- 本程序只调用 Robot() 和 get_joint_states()。
- 不包含 move_*、end_effector_pose_control、move_ee_pos、set_control_mode 等运动/控制接口。
- 不执行机械臂、灵巧手、腰部或底盘运动。
- 不修改 TCP、TF、Joint、控制模式或设备参数。

运行环境：
    /usr/bin/python3.10
    ROS2 Humble
    agibot_gdk

建议运行：
    /usr/bin/python3.10 101_g2_o10_dual_arm_joint_probe_v1.py
"""

from __future__ import annotations

import json
import math
import os
import sys
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import agibot_gdk


# ============================================================================
# 0. 安全锁：本版本是只读 Probe
# ============================================================================
READ_ONLY_PROBE = True
ENABLE_REAL_MOTION = False
PROGRAM_VERSION = "101_v1"
PROGRAM_NAME = "G2 O10 Dual Arm Joint Read-Only Probe"

# 左右臂 Joint 名称来自本项目最新版 G2 URDF。
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

# 第一版故意不把 Joint Limit 数值硬编码到程序中。
# 原因：后续所有结构/limit 必须以用户提供的最新版 URDF 为唯一依据；
# 若运行目录中存在该 URDF，本程序将自动解析对应 joint 的 lower/upper。
URDF_CANDIDATES = [
    "G2_t2_crs_o10_t2.urdf",
    "G2_t2_crs_o10_t2.urdf.txt",
]

REPORT_DIR = Path(__file__).resolve().parent / "reports"


# ============================================================================
# 1. 通用数据读取：兼容 dict / Python 对象，不猜测新版 GDK 的固定字段结构
# ============================================================================
def field(obj: Any, name: str, default: Any = None) -> Any:
    """安全读取 dict 或对象属性。"""
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def first_field(obj: Any, names: Iterable[str]) -> Tuple[Optional[str], Any]:
    """依次尝试候选字段，并同时返回真实命中的字段名。"""
    for name in names:
        value = field(obj, name, None)
        if value is not None:
            return name, value
    return None, None


def safe_number(value: Any) -> Optional[float]:
    """将可能的数值转换为 float；转换失败返回 None。"""
    try:
        if value is None:
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def finite_or_none(value: Any) -> Optional[bool]:
    number = safe_number(value)
    if number is None:
        return None
    return math.isfinite(number)


def make_serializable(value: Any, depth: int = 0) -> Any:
    """尽量把 GDK 返回对象安全转换为 JSON 可保存的数据。"""
    if depth > 5:
        return repr(value)
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(k): make_serializable(v, depth + 1) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [make_serializable(v, depth + 1) for v in value]
    if hasattr(value, "__dict__"):
        try:
            return {
                str(k): make_serializable(v, depth + 1)
                for k, v in vars(value).items()
                if not str(k).startswith("_")
            }
        except Exception:
            pass
    return repr(value)


# ============================================================================
# 2. URDF Joint Limit：只从当前最新版 URDF 文件读取，不使用旧版本数字
# ============================================================================
def find_urdf() -> Optional[Path]:
    base = Path(__file__).resolve().parent
    for name in URDF_CANDIDATES:
        p = base / name
        if p.is_file():
            return p
    return None


def load_arm_limits_from_urdf(path: Path) -> Dict[str, Dict[str, Optional[float]]]:
    import xml.etree.ElementTree as ET

    text = path.read_text(encoding="utf-8")
    root = ET.fromstring(text)
    wanted = set(LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS)
    result: Dict[str, Dict[str, Optional[float]]] = {}

    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        if name not in wanted:
            continue
        limit = joint.find("limit")
        result[name] = {
            "type": joint.attrib.get("type"),
            "lower": safe_number(limit.attrib.get("lower")) if limit is not None else None,
            "upper": safe_number(limit.attrib.get("upper")) if limit is not None else None,
            "effort": safe_number(limit.attrib.get("effort")) if limit is not None else None,
            "velocity": safe_number(limit.attrib.get("velocity")) if limit is not None else None,
        }
    return result


# ============================================================================
# 3. Joint State 解析
# ============================================================================
def normalize_joint_states(raw: Any) -> List[Any]:
    """从常见返回结构中提取 joint state 列表；无法识别则抛出明确异常。"""
    if isinstance(raw, (list, tuple)):
        return list(raw)

    for key in ("joint_states", "states", "joints", "data"):
        candidate = field(raw, key, None)
        if isinstance(candidate, (list, tuple)):
            return list(candidate)

    raise TypeError(
        "无法识别 robot.get_joint_states() 的返回结构。"
        "程序没有继续猜测字段，请把控制台中的 RAW RETURN TYPE/RAW RETURN 内容提供给开发对话。"
    )


def joint_name(state: Any) -> Optional[str]:
    _, value = first_field(state, ("name", "joint_name"))
    return str(value) if value is not None else None


def extract_joint_record(state: Any) -> Dict[str, Any]:
    """保留真实字段名，避免把 motor_position 与 position 混为一谈。"""
    name = joint_name(state)
    pos_key, pos = first_field(state, ("motor_position", "position"))
    vel_key, vel = first_field(state, ("motor_velocity", "velocity"))
    effort_key, effort = first_field(state, ("motor_torque", "torque", "effort"))
    status_key, status = first_field(state, ("status", "state", "error_code"))

    return {
        "name": name,
        "position_field": pos_key,
        "position": safe_number(pos),
        "position_finite": finite_or_none(pos),
        "velocity_field": vel_key,
        "velocity": safe_number(vel),
        "velocity_finite": finite_or_none(vel),
        "effort_field": effort_key,
        "effort": safe_number(effort),
        "effort_finite": finite_or_none(effort),
        "status_field": status_key,
        "status": make_serializable(status),
        "raw": make_serializable(state),
    }


def index_states(states: List[Any]) -> Tuple[Dict[str, Any], List[str], List[str]]:
    index: Dict[str, Any] = {}
    duplicates: List[str] = []
    unnamed: List[str] = []

    for i, state in enumerate(states):
        name = joint_name(state)
        if not name:
            unnamed.append(f"index={i}")
            continue
        if name in index:
            duplicates.append(name)
        else:
            index[name] = state
    return index, sorted(set(duplicates)), unnamed


def check_arm(
    label: str,
    names: List[str],
    index: Dict[str, Any],
    limits: Dict[str, Dict[str, Optional[float]]],
) -> Dict[str, Any]:
    joints = []
    missing = []
    ok = True

    for seq, name in enumerate(names, start=1):
        state = index.get(name)
        if state is None:
            missing.append(name)
            joints.append({"sequence": seq, "name": name, "found": False})
            ok = False
            continue

        rec = extract_joint_record(state)
        rec["sequence"] = seq
        rec["found"] = True

        limit = limits.get(name)
        rec["urdf_limit"] = limit
        within_limit = None
        if limit and rec["position"] is not None:
            lower = limit.get("lower")
            upper = limit.get("upper")
            if lower is not None and upper is not None:
                within_limit = lower <= rec["position"] <= upper
        rec["within_urdf_limit"] = within_limit

        if rec["position_finite"] is False or within_limit is False:
            ok = False
        joints.append(rec)

    return {
        "arm": label,
        "expected_joint_count": len(names),
        "found_joint_count": len(names) - len(missing),
        "missing_joints": missing,
        "ok": ok and not missing,
        "joints": joints,
    }


# ============================================================================
# 4. 显示
# ============================================================================
def print_header() -> None:
    print("=" * 88)
    print(PROGRAM_NAME)
    print(f"Version             : {PROGRAM_VERSION}")
    print(f"Python executable   : {sys.executable}")
    print(f"Python version      : {sys.version.split()[0]}")
    print(f"ROS_DISTRO          : {os.environ.get('ROS_DISTRO', '<not set>')}")
    print(f"READ_ONLY_PROBE     : {READ_ONLY_PROBE}")
    print(f"ENABLE_REAL_MOTION  : {ENABLE_REAL_MOTION}")
    print("SAFETY              : 本程序不包含机器人运动/控制调用")
    print("=" * 88)


def fmt(v: Any, digits: int = 6) -> str:
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    if v is None:
        return "N/A"
    return str(v)


def print_arm_result(result: Dict[str, Any]) -> None:
    print()
    print("-" * 88)
    print(f"[{result['arm']}] {result['found_joint_count']}/{result['expected_joint_count']} joints")
    print("-" * 88)
    for j in result["joints"]:
        if not j.get("found"):
            print(f"[MISSING] J{j['sequence']}  {j['name']}")
            continue
        lim = j.get("urdf_limit") or {}
        print(
            f"J{j['sequence']}  {j['name']}\n"
            f"    position : {fmt(j.get('position'))} rad   source={j.get('position_field') or 'N/A'}\n"
            f"    velocity : {fmt(j.get('velocity'))}       source={j.get('velocity_field') or 'N/A'}\n"
            f"    effort   : {fmt(j.get('effort'))}       source={j.get('effort_field') or 'N/A'}\n"
            f"    limit    : [{fmt(lim.get('lower'))}, {fmt(lim.get('upper'))}]"
            f"   within={fmt(j.get('within_urdf_limit'))}\n"
            f"    finite   : position={fmt(j.get('position_finite'))}, "
            f"velocity={fmt(j.get('velocity_finite'))}, effort={fmt(j.get('effort_finite'))}"
        )


# ============================================================================
# 5. 主程序
# ============================================================================
def main() -> int:
    print_header()

    # 双保险。任何人若误改安全常量，本只读版本直接拒绝继续。
    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION:
        print("[FATAL] Safety lock invalid. 101_v1 必须保持 READ_ONLY_PROBE=True 且 ENABLE_REAL_MOTION=False")
        return 90

    urdf_path = find_urdf()
    limits: Dict[str, Dict[str, Optional[float]]] = {}
    if urdf_path:
        try:
            limits = load_arm_limits_from_urdf(urdf_path)
            print(f"[URDF] Loaded          : {urdf_path}")
            print(f"[URDF] Arm limits found: {len(limits)}/14")
        except Exception as exc:
            print(f"[WARN] URDF found but parse failed: {exc}")
    else:
        print("[INFO] 最新 URDF 未放在脚本同目录，本次仍可读取 Joint；Joint Limit 检查显示 N/A。")
        print("       如需 limit 核对，请把 G2_t2_crs_o10_t2.urdf.txt 放到脚本同目录后再运行。")

    print("\n[GDK] Creating agibot_gdk.Robot() ...")
    robot = agibot_gdk.Robot()

    print("[GDK] Calling robot.get_joint_states() [READ ONLY] ...")
    raw = robot.get_joint_states()
    print(f"[GDK] RAW RETURN TYPE : {type(raw).__name__}")

    try:
        states = normalize_joint_states(raw)
    except Exception:
        print("\n[FATAL] 无法解析 get_joint_states()。以下是原始返回：")
        print(repr(raw))
        raise

    print(f"[GDK] TOTAL STATES    : {len(states)}")

    index, duplicates, unnamed = index_states(states)
    left = check_arm("LEFT_ARM", LEFT_ARM_JOINTS, index, limits)
    right = check_arm("RIGHT_ARM", RIGHT_ARM_JOINTS, index, limits)

    print_arm_result(left)
    print_arm_result(right)

    expected = set(LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS)
    all_names = [joint_name(s) for s in states]
    other_names = [n for n in all_names if n and n not in expected]

    summary_ok = (
        left["ok"]
        and right["ok"]
        and not duplicates
        and left["found_joint_count"] == 7
        and right["found_joint_count"] == 7
    )

    report = {
        "program": PROGRAM_NAME,
        "version": PROGRAM_VERSION,
        "timestamp_local": datetime.now().astimezone().isoformat(),
        "safety": {
            "read_only_probe": READ_ONLY_PROBE,
            "enable_real_motion": ENABLE_REAL_MOTION,
            "motion_api_present": False,
        },
        "environment": {
            "python_executable": sys.executable,
            "python_version": sys.version,
            "ros_distro": os.environ.get("ROS_DISTRO"),
            "agibot_gdk_module": getattr(agibot_gdk, "__file__", None),
            "agibot_gdk_version": getattr(agibot_gdk, "__version__", None),
        },
        "urdf": {
            "path": str(urdf_path) if urdf_path else None,
            "limits_loaded": len(limits),
        },
        "joint_state": {
            "total_returned": len(states),
            "duplicate_names": duplicates,
            "unnamed_entries": unnamed,
            "other_joint_names": other_names,
        },
        "left_arm": left,
        "right_arm": right,
        "overall_ok": summary_ok,
    }

    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_path = REPORT_DIR / f"101_dual_arm_joint_probe_{stamp}.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    print("\n" + "=" * 88)
    print("PROBE SUMMARY")
    print("=" * 88)
    print(f"Left arm             : {left['found_joint_count']}/7")
    print(f"Right arm            : {right['found_joint_count']}/7")
    print(f"Duplicate joint names: {duplicates if duplicates else 'NONE'}")
    print(f"Unnamed states       : {unnamed if unnamed else 'NONE'}")
    print(f"Overall              : {'PASS' if summary_ok else 'CHECK REQUIRED'}")
    print(f"JSON report          : {report_path}")
    print("Motion executed      : NO")
    print("=" * 88)

    # CHECK REQUIRED 使用非零返回码，方便以后脚本化检测；它不会触发任何动作。
    return 0 if summary_ok else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[STOP] User interrupted. No motion command was issued.")
        raise SystemExit(130)
    except Exception as exc:
        print(f"\n[FATAL] {type(exc).__name__}: {exc}")
        traceback.print_exc()
        print("[SAFETY] Program ended. No motion command was issued by this probe.")
        raise SystemExit(1)
