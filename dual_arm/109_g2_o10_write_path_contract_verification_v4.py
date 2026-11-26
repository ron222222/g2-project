#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
109_v4 G2/O10 Write-Path Contract Verification
================================================

Purpose
-------
Verify the three candidate O10 write-path contracts against the offline
GDK SDK 4.1.5 Markdown and, when available, cross-check the live O10 state
using read-only GDK methods.

SAFETY BOUNDARY
---------------
This program never invokes a motion, servo, planning, or write API.  It may:
  * parse a local documentation file;
  * initialize/release GDK;
  * create a Robot object;
  * read end-effector and joint state;
  * construct detached request/data objects in local memory.

Detached objects are NEVER transmitted.  A source-level AST audit runs before
GDK initialization and aborts if a forbidden call expression is present.

Recommended robot-side location:
  /home/isaac-workstation/g2-project/dual_arm/

Typical invocation:
  /usr/bin/python3.10 109_g2_o10_write_path_contract_verification_v4.py \
      --doc /home/isaac-workstation/g2-project/dual_arm/GDK_SDK_4.1.5.md

Documentation-only invocation (no GDK import/network use):
  /usr/bin/python3.10 109_g2_o10_write_path_contract_verification_v4.py \
      --doc ./GDK_SDK_4.1.5.md --doc-only
"""

from __future__ import annotations

import argparse
import ast
import contextlib
import datetime as _dt
import hashlib
import importlib
import inspect
import io
import json
import os
from pathlib import Path
import platform
import re
import shlex
import subprocess
import sys
import traceback
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


TITLE = "109_v4 O10 Write-Path Contract Verification"
REPORT_STEM = "109_g2_o10_write_path_contract_verification_v4_report"
OFFICIAL_ENV = Path("/home/isaac-workstation/.cache/agibot/app/env.sh")
RESTART_MARKER = "G2_109_V4_ENV_RESTARTED"

# These names may appear as documentation strings/data, but must never occur as
# the callee of an AST Call node in this source file.
FORBIDDEN_CALL_NAMES = frozenset(
    {
        "joint_control_request",
        "joint_servo_control",
        "move_ee_pos",
        "move_end_effector_joint",
        "move_head_joint",
        "move_waist_joint",
        "move_arm_joint",
        "end_effector_pose_control",
        "joint_trajectory_control",
        "set_control_mode",
        "execute_action",
    }
)

READ_ONLY_ROBOT_METHODS = frozenset({"get_end_state", "get_joint_states"})

LEFT_O10_LIMITS: Tuple[Tuple[str, float, float], ...] = (
    ("idx31_hand_l_thumb_roll_joint", -1.1213740444063567, 0.029670597283903602),
    ("idx32_hand_l_thumb_abad_joint", -0.04537856055185257, 1.642354826126664),
    ("idx33_hand_l_thumb_mcp_joint", -0.8415977653116657, 0.0),
    ("idx36_hand_l_index_abad_joint", 0.0, 0.16406094968746698),
    ("idx37_hand_l_index_pip_joint", 0.0, 1.4835298641951802),
    ("idx39_hand_l_middle_pip_joint", 0.0, 1.4835298641951802),
    ("idx41_hand_l_ring_abad_joint", -0.16929693744344995, 0.0),
    ("idx42_hand_l_ring_pip_joint", 0.0, 1.4835298641951802),
    ("idx44_hand_l_pinky_abad_joint", -0.1850049007113989, 0.0),
    ("idx45_hand_l_pinky_pip_joint", 0.0, 1.4835298641951802),
)

RIGHT_O10_LIMITS: Tuple[Tuple[str, float, float], ...] = (
    ("idx71_hand_r_thumb_roll_joint", -0.029670597283903602, 1.1213740444063567),
    ("idx72_hand_r_thumb_abad_joint", -1.642354826126664, 0.04537856055185257),
    ("idx73_hand_r_thumb_mcp_joint", 0.0, 0.8415977653116657),
    ("idx76_hand_r_index_abad_joint", -0.16406094968746698, 0.0),
    ("idx77_hand_r_index_pip_joint", 0.0, 1.4835298641951802),
    ("idx79_hand_r_middle_pip_joint", 0.0, 1.4835298641951802),
    ("idx81_hand_r_ring_abad_joint", 0.0, 0.16929693744344995),
    ("idx82_hand_r_ring_pip_joint", 0.0, 1.4835298641951802),
    ("idx84_hand_r_pinky_abad_joint", 0.0, 0.1850049007113989),
    ("idx85_hand_r_pinky_pip_joint", 0.0, 1.4835298641951802),
)


@dataclass(frozen=True)
class Check:
    key: str
    area: str
    description: str
    passed: bool
    evidence: str
    required: bool = True


@dataclass
class LiveSide:
    side: str
    names: List[str]
    positions: List[float]
    controlled: Optional[bool]
    error_code: Optional[int]


class Tee(io.TextIOBase):
    def __init__(self, *streams: io.TextIOBase) -> None:
        self.streams = streams

    def write(self, data: str) -> int:
        for stream in self.streams:
            stream.write(data)
            stream.flush()
        return len(data)

    def flush(self) -> None:
        for stream in self.streams:
            stream.flush()


def banner(text: str, char: str = "=") -> None:
    print(char * 108)
    print(text)
    print(char * 108)


def clean_markdown(text: str) -> str:
    """Normalize Markdown escaping/whitespace for resilient phrase matching."""
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    text = text.replace("\\_", "_")
    text = text.replace("｜", "|")
    text = re.sub(r"[ \t]+", " ", text)
    return text


def short(value: Any, limit: int = 180) -> str:
    try:
        raw = repr(value)
    except Exception as exc:  # pragma: no cover - defensive
        raw = f"<repr failed: {type(exc).__name__}: {exc}>"
    return raw if len(raw) <= limit else raw[: limit - 3] + "..."


def field(obj: Any, name: str, default: Any = None) -> Any:
    if obj is None:
        return default
    if isinstance(obj, Mapping):
        return obj.get(name, default)
    return getattr(obj, name, default)


def as_list(value: Any) -> List[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    try:
        return list(value)
    except TypeError:
        return [value]


def call_name(node: ast.Call) -> Optional[str]:
    callee = node.func
    if isinstance(callee, ast.Attribute):
        return callee.attr
    if isinstance(callee, ast.Name):
        return callee.id
    return None


def source_safety_audit(source_path: Path) -> Tuple[bool, List[str]]:
    """Reject direct or literal-getattr calls to any motion/write API."""
    source = source_path.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(source_path))
    findings: List[str] = []

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = call_name(node)
        if name in FORBIDDEN_CALL_NAMES:
            findings.append(f"line {node.lineno}: forbidden direct call '{name}'")

        # Prevent obvious dynamic-call bypass: getattr(obj, "forbidden")(...) or
        # a literal forbidden method lookup anywhere in the source.
        if name == "getattr" and len(node.args) >= 2:
            literal = node.args[1]
            if isinstance(literal, ast.Constant) and isinstance(literal.value, str):
                if literal.value in FORBIDDEN_CALL_NAMES:
                    findings.append(
                        f"line {node.lineno}: forbidden literal getattr '{literal.value}'"
                    )

    return not findings, findings


def restart_with_official_environment_if_needed(args: argparse.Namespace) -> None:
    if args.doc_only or args.no_env_restart:
        return
    if os.environ.get(RESTART_MARKER) == "1":
        return
    if not OFFICIAL_ENV.is_file():
        print(f"[ENV] Official environment file not found; continuing: {OFFICIAL_ENV}")
        return

    print("[ENV] Restarting once with the official GDK environment ...")
    command = f"source {shlex.quote(str(OFFICIAL_ENV))} >/dev/null 2>&1 && env -0"
    proc = subprocess.run(
        ["/bin/bash", "-lc", command],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if proc.returncode != 0:
        print(
            "[ENV] WARNING: failed to source official environment; continuing with "
            f"current environment (rc={proc.returncode})."
        )
        if proc.stderr:
            print(proc.stderr.decode("utf-8", errors="replace").strip())
        return

    env = dict(os.environ)
    for item in proc.stdout.split(b"\0"):
        if not item or b"=" not in item:
            continue
        key, value = item.split(b"=", 1)
        env[key.decode(errors="surrogateescape")] = value.decode(errors="surrogateescape")
    env[RESTART_MARKER] = "1"
    os.execve(sys.executable, [sys.executable, *sys.argv], env)


def locate_doc(explicit: Optional[str], source_path: Path) -> Optional[Path]:
    candidates: List[Path] = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    candidates.extend(
        [
            source_path.parent / "GDK_SDK_4.1.5.md",
            Path.cwd() / "GDK_SDK_4.1.5.md",
            Path("/home/isaac-workstation/g2-project/dual_arm/GDK_SDK_4.1.5.md"),
            Path.home() / "Downloads/GDK_SDK_4.1.5.md",
        ]
    )
    seen: set[str] = set()
    for candidate in candidates:
        key = str(candidate.resolve(strict=False))
        if key in seen:
            continue
        seen.add(key)
        if candidate.is_file():
            return candidate.resolve()
    return None


def section(text: str, heading_pattern: str) -> str:
    """Return a Markdown #### section through the next peer heading."""
    match = re.search(heading_pattern, text, flags=re.IGNORECASE | re.MULTILINE)
    if not match:
        return ""
    tail = text[match.start() :]
    next_heading = re.search(r"\n####\s+", tail[1:])
    if next_heading:
        return tail[: next_heading.start() + 1]
    return tail


def contains_all(text: str, parts: Iterable[str]) -> bool:
    lowered = text.lower()
    return all(part.lower() in lowered for part in parts)


def doc_checks(doc_text: str) -> List[Check]:
    normalized = clean_markdown(doc_text)
    joint = section(normalized, r"^####\s+9\.\s+`?joint_control_request\(\)`?")
    servo = section(normalized, r"^####\s+13\.\s+`?joint_servo_control\(\)`?")
    move = section(normalized, r"^####\s+17\.\s+`?move_ee_pos\(\)`?")

    checks: List[Check] = []

    def add(
        key: str,
        area: str,
        description: str,
        passed: bool,
        evidence: str,
        required: bool = True,
    ) -> None:
        checks.append(Check(key, area, description, bool(passed), evidence, required))

    add("JCR_SECTION", "joint_control_request", "Python API section exists", bool(joint),
        "Heading: 9. joint_control_request()")
    add("JCR_BLOCKING", "joint_control_request", "Path-planning control is blocking",
        contains_all(joint, ["阻塞式", "执行到目标位置后", "接口返回"]),
        "Document says the call returns after reaching the target.")
    add("JCR_LIFETIME", "joint_control_request", "life_time unit is seconds",
        bool(re.search(r"life_time[^\n]*秒", joint, flags=re.IGNORECASE)),
        "life_time | float | 生命周期 | 秒")
    add("JCR_UNITS", "joint_control_request", "Position/velocity units are rad and rad/s",
        contains_all(joint, ["joint_positions", "弧度", "joint_velocities", "弧度/秒"]),
        "joint_positions: rad; joint_velocities: rad/s")
    add("JCR_RETURN", "joint_control_request", "Python success value is int 0; failures raise",
        contains_all(joint, ["int", "0表示成功", "失败时抛出异常"]),
        "Return: int; 0 success; failure raises.")
    add("JCR_O10_EXPLICIT", "joint_control_request", "Section explicitly documents O10 use",
        bool(re.search(r"\bo10(?:_t2)?\b", joint, flags=re.IGNORECASE)),
        "No O10-specific statement/example in this section.", required=False)

    add("SERVO_SECTION", "joint_servo_control", "Python API section exists", bool(servo),
        "Heading: 13. joint_servo_control()")
    add("SERVO_STREAM", "joint_servo_control", "Non-blocking streaming at 100 Hz",
        contains_all(servo, ["非阻塞式", "100hz"]),
        "Document requires 100 Hz control and labels the API non-blocking.")
    add("SERVO_EE", "joint_servo_control", "End-effector control is supported",
        "支持同时控制末端执行器" in servo,
        "Document explicitly says the interface can simultaneously control an end effector.")
    add("SERVO_PERIOD", "joint_servo_control", "control_period is in seconds",
        bool(re.search(r"control_period[^\n]*秒", servo, flags=re.IGNORECASE)),
        "control_period | float | 控制周期 | 秒")
    add("SERVO_LENGTH", "joint_servo_control", "Name and position lengths must match",
        contains_all(servo, ["joint_names", "joint_positions", "长度必须相同"]),
        "joint_names and joint_positions lengths must match.")
    add("SERVO_VELOCITY_OPTIONAL", "joint_servo_control", "Velocity list is reserved and may be empty",
        contains_all(servo, ["joint_velocities", "预留参数", "可以为空"]),
        "joint_velocities is currently unused and may be empty.")
    add("SERVO_LIMIT", "joint_servo_control", "Positions must remain within joint limits",
        contains_all(servo, ["joint_positions", "限位范围内"]),
        "Out-of-range position produces invalid input/an exception.")
    add("SERVO_LOW_LATENCY", "joint_servo_control", "Low-latency bypass risk is documented",
        contains_all(servo, ["低延时模式", "绕过运控算法保护", "自碰撞"]),
        "Low latency bypasses motion-control protections; default is normal mode.")
    add("SERVO_RETURN", "joint_servo_control", "Python success value is int 0; failures raise",
        contains_all(servo, ["int", "0表示成功", "失败时抛出异常"]),
        "Return: int; 0 success; failure raises.")
    add("SERVO_WATCHDOG", "joint_servo_control", "Watchdog/drop-frame semantics are documented",
        contains_all(servo, ["watchdog", "掉帧"]),
        "No watchdog/drop-frame contract located in this section.", required=False)

    add("MOVE_SECTION", "move_ee_pos", "Python API section exists", bool(move),
        "Heading: 17. move_ee_pos()")
    add("MOVE_NONBLOCKING", "move_ee_pos", "End-effector position control is non-blocking",
        "非阻塞式" in move,
        "Document labels the API non-blocking.")
    add("MOVE_EXCLUSIVE", "move_ee_pos", "Cannot be used concurrently with servo",
        contains_all(move, ["不可与伺服接口同时使用", "伺服接口控制末端执行器"]),
        "Use the servo interface when concurrent servo/end-effector control is needed.")
    add("MOVE_GROUPS", "move_ee_pos", "Exact group vocabulary is documented",
        contains_all(move, ['"left_tool"', '"right_tool"', '"dual_tool"']),
        'Allowed strings: "left_tool", "right_tool", "dual_tool".')
    add("MOVE_SHAPE", "move_ee_pos", "nums equals len(states); only position is required",
        contains_all(move, ["nums", "len(states)", "只需要设置", "position"]),
        "nums == len(states); each JointState only needs position.")
    add("MOVE_O10", "move_ee_pos", "Explicit O10_T2 10-joint contract/example exists",
        contains_all(move, ["o10_t2", "需要10个关节", "states_list", "range(10)"]),
        "Example 4 builds 10 JointState objects for left_tool O10_T2.")

    move_names_ok = all(name in move for name, _, _ in LEFT_O10_LIMITS + RIGHT_O10_LIMITS)
    add("MOVE_O10_NAMES", "move_ee_pos", "All 20 documented left/right O10 names are present",
        move_names_ok,
        "Ordered O10 limit tables contain ten names per side.")

    limits_ok = True
    for name, lower, upper in LEFT_O10_LIMITS + RIGHT_O10_LIMITS:
        # Require name and the two exact numeric tokens to occur close together.
        row_pattern = (
            re.escape(name)
            + r"[^\n]*"
            + re.escape(str(lower))
            + r"[^\n]*"
            + re.escape(str(upper))
        )
        if not re.search(row_pattern, move):
            limits_ok = False
            break
    add("MOVE_O10_LIMITS", "move_ee_pos", "All documented O10 limits match v4 constants",
        limits_ok,
        "Twenty exact min/max rows verified against the embedded 4.1.5 table.")

    add("MOVE_TARGET_TYPE", "move_ee_pos", "target_type is part of the documented input contract",
        "target_type" in move,
        "The documented MoveEEPos input lists group, states, and nums; not target_type.",
        required=False)
    add("MOVE_TIMESTAMP", "move_ee_pos", "timestamp is part of the documented input contract",
        "timestamp" in move,
        "The documented MoveEEPos input lists group, states, and nums; not timestamp.",
        required=False)

    return checks


def print_checks(checks: Sequence[Check]) -> None:
    banner("[DOCUMENT CONTRACT CHECKS]")
    current = None
    for check in checks:
        if check.area != current:
            current = check.area
            print(f"\n{current}")
        status = "PASS" if check.passed else ("UNRESOLVED" if not check.required else "FAIL")
        print(f"  [{status:10}] {check.key:25} {check.description}")
        print(f"               evidence: {check.evidence}")


def required_pass(checks: Sequence[Check], area: Optional[str] = None) -> bool:
    selected = [c for c in checks if c.required and (area is None or c.area == area)]
    return bool(selected) and all(c.passed for c in selected)


def parse_end_state(end_state: Any, side: str) -> LiveSide:
    key = f"{side}_end_state"
    state = field(end_state, key)
    if state is None:
        raise RuntimeError(f"get_end_state result has no {key}")
    names = [str(x) for x in as_list(field(state, "names", []))]
    motors = as_list(field(state, "end_states", []))
    positions = [float(field(motor, "position")) for motor in motors]
    controlled_raw = field(state, "controlled", None)
    error_raw = field(state, "err_code", field(state, "error_code", None))
    controlled = None if controlled_raw is None else bool(controlled_raw)
    error_code = None if error_raw is None else int(error_raw)
    return LiveSide(side, names, positions, controlled, error_code)


def verify_live_side(live: LiveSide, limits: Sequence[Tuple[str, float, float]]) -> Dict[str, Any]:
    expected_names = [row[0] for row in limits]
    length_ok = len(live.names) == 10 and len(live.positions) == 10
    order_ok = live.names == expected_names
    bounds: List[Dict[str, Any]] = []
    within = length_ok and order_ok
    if length_ok and order_ok:
        for (name, lower, upper), value in zip(limits, live.positions):
            ok = lower <= value <= upper
            within = within and ok
            bounds.append(
                {"name": name, "position": value, "min": lower, "max": upper, "within": ok}
            )
    return {
        "side": live.side,
        "count_ok": length_ok,
        "order_ok": order_ok,
        "within_limits": within,
        "controlled": live.controlled,
        "error_code": live.error_code,
        "rows": bounds,
    }


def safe_read_robot(robot: Any, method_name: str) -> Any:
    if method_name not in READ_ONLY_ROBOT_METHODS:
        raise RuntimeError(f"read method is not allowlisted: {method_name}")
    method = getattr(robot, method_name)
    return method()


def snapshot_object(obj: Any, fields: Sequence[str]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for name in fields:
        value = field(obj, name, "<missing>")
        if isinstance(value, (str, int, float, bool)) or value is None:
            result[name] = value
        else:
            seq = as_list(value)
            result[name] = {
                "type": f"{type(value).__module__}.{type(value).__name__}",
                "length": len(seq),
                "preview": [short(x, 100) for x in seq[:3]],
            }
    return result


def build_detached_candidates(gdk: Any, live_sides: Sequence[LiveSide]) -> Dict[str, Any]:
    """Build current-state-shaped objects only; no Robot method is touched."""
    built: Dict[str, Any] = {}
    for live in live_sides:
        if len(live.names) != 10 or len(live.positions) != 10:
            continue
        prefix = live.side

        move_req = gdk.JointStates()
        move_states: List[Any] = []
        for value in live.positions:
            state = gdk.JointState()
            state.position = float(value)
            move_states.append(state)
        move_req.group = "left_tool" if live.side == "left" else "right_tool"
        move_req.states = move_states
        move_req.nums = len(move_states)
        # Intentionally do not set target_type or timestamp: they are not part
        # of the documented MoveEEPos input contract in SDK 4.1.5.
        built[f"{prefix}.JointStates"] = snapshot_object(
            move_req, ["group", "nums", "states", "target_type", "timestamp"]
        )

        planned_req = gdk.JointControlReq()
        planned_req.joint_names = list(live.names)
        planned_req.joint_positions = list(live.positions)
        planned_req.joint_velocities = [0.0] * len(live.names)
        planned_req.life_time = 5.0
        planned_req.detail = "109_v4_detached_contract_candidate_never_transmitted"
        built[f"{prefix}.JointControlReq"] = snapshot_object(
            planned_req,
            ["joint_names", "joint_positions", "joint_velocities", "life_time", "detail"],
        )

        servo_req = gdk.JointServoControlReq()
        servo_req.control_period = 0.01
        servo_req.joint_names = list(live.names)
        servo_req.joint_positions = list(live.positions)
        # SDK 4.1.5 explicitly says velocities are reserved and may be empty.
        servo_req.joint_velocities = []
        built[f"{prefix}.JointServoControlReq"] = snapshot_object(
            servo_req,
            ["control_period", "joint_names", "joint_positions", "joint_velocities"],
        )
    return built


def runtime_probe() -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "attempted": True,
        "imported": False,
        "initialized": False,
        "robot_created": False,
        "released": False,
        "live_sides": [],
        "side_verification": [],
        "detached_candidates": {},
        "errors": [],
    }
    gdk = None
    initialized = False
    try:
        gdk = importlib.import_module("agibot_gdk")
        result["imported"] = True
        result["module_file"] = getattr(gdk, "__file__", "<not exposed>")

        init_value = gdk.gdk_init()
        success_value = getattr(getattr(gdk, "GDKRes", object()), "kSuccess", 0)
        if init_value != success_value:
            raise RuntimeError(f"gdk_init returned {init_value!r}, expected {success_value!r}")
        initialized = True
        result["initialized"] = True

        robot = gdk.Robot()
        result["robot_created"] = True

        end_state = safe_read_robot(robot, "get_end_state")
        joint_states = safe_read_robot(robot, "get_joint_states")
        result["joint_state_summary"] = {
            "type": f"{type(joint_states).__module__}.{type(joint_states).__name__}",
            "nums": field(joint_states, "nums", None),
            "state_count": len(as_list(field(joint_states, "states", []))),
        }

        live_sides = [parse_end_state(end_state, "left"), parse_end_state(end_state, "right")]
        result["live_sides"] = [
            {
                "side": side.side,
                "names": side.names,
                "positions": side.positions,
                "controlled": side.controlled,
                "error_code": side.error_code,
            }
            for side in live_sides
        ]
        result["side_verification"] = [
            verify_live_side(live_sides[0], LEFT_O10_LIMITS),
            verify_live_side(live_sides[1], RIGHT_O10_LIMITS),
        ]
        result["detached_candidates"] = build_detached_candidates(gdk, live_sides)
    except Exception as exc:
        result["errors"].append(f"{type(exc).__name__}: {exc}")
        result["traceback"] = traceback.format_exc()
    finally:
        if gdk is not None and initialized:
            try:
                release_value = gdk.gdk_release()
                result["release_result"] = short(release_value)
                result["released"] = True
            except Exception as exc:  # pragma: no cover - depends on robot runtime
                result["errors"].append(f"gdk_release: {type(exc).__name__}: {exc}")
    return result


def print_runtime(runtime: Dict[str, Any]) -> None:
    banner("[LIVE READ-ONLY CROSS-CHECK]")
    print(f"import agibot_gdk     : {'PASS' if runtime.get('imported') else 'FAIL'}")
    print(f"GDK initialized       : {'PASS' if runtime.get('initialized') else 'FAIL'}")
    print(f"Robot created         : {'PASS' if runtime.get('robot_created') else 'FAIL'}")
    print(f"GDK released          : {'PASS' if runtime.get('released') else 'FAIL'}")
    if runtime.get("module_file"):
        print(f"module file           : {runtime['module_file']}")
    if runtime.get("joint_state_summary"):
        print(f"get_joint_states      : {json.dumps(runtime['joint_state_summary'], ensure_ascii=False)}")

    for side, verdict in zip(runtime.get("live_sides", []), runtime.get("side_verification", [])):
        label = str(side["side"]).upper()
        print("-" * 108)
        print(f"{label} O10")
        print(f"  names/positions     : {len(side['names'])}/{len(side['positions'])}")
        print(f"  documented order    : {'PASS' if verdict['order_ok'] else 'FAIL'}")
        print(f"  documented limits   : {'PASS' if verdict['within_limits'] else 'FAIL'}")
        print(f"  controlled          : {side['controlled']!r}")
        print(f"  error_code          : {side['error_code']!r}")
        for row in verdict.get("rows", []):
            print(
                f"    {row['name']:<36} {row['position']:+.9f}  "
                f"[{row['min']:+.9f}, {row['max']:+.9f}]  "
                f"{'OK' if row['within'] else 'OUT-OF-RANGE'}"
            )

    banner("[DETACHED CURRENT-STATE OBJECTS - NEVER TRANSMITTED]", "-")
    candidates = runtime.get("detached_candidates", {})
    if not candidates:
        print("<none built>")
    for name, snapshot in candidates.items():
        print(name)
        print(json.dumps(snapshot, ensure_ascii=False, indent=2, sort_keys=True))

    if runtime.get("errors"):
        print("runtime errors:")
        for error in runtime["errors"]:
            print(f"  - {error}")


def contract_matrix(checks: Sequence[Check]) -> List[Dict[str, str]]:
    move_ready = required_pass(checks, "move_ee_pos")
    joint_ready = required_pass(checks, "joint_control_request")
    servo_ready = required_pass(checks, "joint_servo_control")
    explicit_joint_o10 = any(c.key == "JCR_O10_EXPLICIT" and c.passed for c in checks)
    watchdog = any(c.key == "SERVO_WATCHDOG" and c.passed for c in checks)

    return [
        {
            "interface": "move_ee_pos",
            "document_status": "DOCUMENTED-O10" if move_ready else "FAILED-DOC-CHECK",
            "O10_evidence": (
                "Explicit O10_T2 10-state example, ordered O10 limit tables, exact group strings"
                if move_ready else "Required MoveEEPos checks did not all pass"
            ),
            "first_micro_motion": "FIRST CANDIDATE" if move_ready else "BLOCKED",
            "remaining_boundary": (
                "Non-blocking; complete ordered 10-position vector; no concurrent servo; runtime review still required"
            ),
        },
        {
            "interface": "joint_control_request",
            "document_status": "GENERIC-CONTRACT" if joint_ready else "FAILED-DOC-CHECK",
            "O10_evidence": (
                "EXPLICIT" if explicit_joint_o10 else "NOT EXPLICIT IN THE PYTHON API SECTION"
            ),
            "first_micro_motion": "NOT SELECTED",
            "remaining_boundary": (
                "Blocking path-planning API; generic joint-name schema but no O10-specific statement/example"
            ),
        },
        {
            "interface": "joint_servo_control",
            "document_status": "STREAMING-END-EFFECTOR" if servo_ready else "FAILED-DOC-CHECK",
            "O10_evidence": "End-effector joint names may be included; 100 Hz stream is required",
            "first_micro_motion": "DEFER",
            "remaining_boundary": (
                "Normal mode only; low-latency bypasses protections; watchdog/drop-frame semantics "
                + ("documented" if watchdog else "remain undocumented")
            ),
        },
    ]


def print_matrix(matrix: Sequence[Dict[str, str]]) -> None:
    banner("[WRITE-PATH CONTRACT MATRIX]")
    for row in matrix:
        print(row["interface"])
        print(f"  document status    : {row['document_status']}")
        print(f"  O10 evidence       : {row['O10_evidence']}")
        print(f"  first micro-motion : {row['first_micro_motion']}")
        print(f"  boundary           : {row['remaining_boundary']}")


def live_contract_pass(runtime: Dict[str, Any]) -> bool:
    if runtime.get("errors"):
        return False
    if not all(runtime.get(k) for k in ("imported", "initialized", "robot_created", "released")):
        return False
    sides = runtime.get("side_verification", [])
    return len(sides) == 2 and all(
        side.get("count_ok") and side.get("order_ok") and side.get("within_limits") for side in sides
    )


def print_summary(
    audit_ok: bool,
    doc_path: Optional[Path],
    checks: Sequence[Check],
    runtime: Optional[Dict[str, Any]],
    matrix: Sequence[Dict[str, str]],
) -> None:
    banner("109_v4 NON-MOTION WRITE-PATH CONTRACT VERIFICATION SUMMARY")
    print(f"source safety audit              : {'PASS' if audit_ok else 'FAIL'}")
    print(f"SDK document located             : {'PASS' if doc_path else 'FAIL'}")
    print(f"required document checks         : {'PASS' if required_pass(checks) else 'FAIL'}")
    print(
        "live read-only cross-check       : "
        + ("SKIPPED (--doc-only)" if runtime is None else ("PASS" if live_contract_pass(runtime) else "FAIL"))
    )
    print("motion/control API invoked       : NO")
    print("O10 commanded                    : NO")
    print("Arm commanded                    : NO")
    print("Motion executed                  : NO")
    print()
    selected = next((row for row in matrix if row["interface"] == "move_ee_pos"), None)
    print("Final contract verdict:")
    if selected and selected["first_micro_motion"] == "FIRST CANDIDATE":
        print("  DOCUMENT CONTRACT SELECTS: move_ee_pos + JointStates as the first O10 candidate.")
        print("  Reason: SDK 4.1.5 explicitly documents O10_T2, exact 10-state shape,")
        print("          ordered limits, and left_tool/right_tool group strings.")
    else:
        print("  BLOCKED: required MoveEEPos/O10 document checks did not pass.")
    print()
    print("This script is evidence collection only. It does not authorize or perform 110 motion.")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=TITLE)
    parser.add_argument("--doc", help="Path to GDK_SDK_4.1.5.md")
    parser.add_argument(
        "--doc-only",
        action="store_true",
        help="Parse/verify the SDK document only; do not import or initialize GDK",
    )
    parser.add_argument(
        "--no-env-restart",
        action="store_true",
        help="Do not restart under the official GDK environment",
    )
    parser.add_argument(
        "--report",
        help="Optional report path (default: timestamped file beside this script)",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    restart_with_official_environment_if_needed(args)

    source_path = Path(__file__).resolve()
    timestamp = _dt.datetime.now().astimezone()
    if args.report:
        report_path = Path(args.report).expanduser().resolve()
    else:
        report_path = source_path.with_name(
            f"{REPORT_STEM}_{timestamp.strftime('%Y%m%d_%H%M%S')}.txt"
        )
    report_path.parent.mkdir(parents=True, exist_ok=True)

    buffer = io.StringIO()
    with contextlib.redirect_stdout(Tee(sys.__stdout__, buffer)):
        banner(TITLE)
        print(f"script       : {source_path}")
        print(f"timestamp    : {timestamp.isoformat(timespec='seconds')}")
        print(f"python       : {sys.version.replace(os.linesep, ' ')}")
        print(f"platform     : {platform.platform()}")
        print("safety mode  : DOC PARSE / READ-ONLY STATE / DETACHED CONSTRUCT ONLY")

        banner("[SOURCE SAFETY AUDIT]")
        audit_ok, findings = source_safety_audit(source_path)
        print(f"status : {'PASS' if audit_ok else 'FAIL'}")
        if findings:
            for finding in findings:
                print(f"  {finding}")
        else:
            print("No direct or literal-getattr motion/control/write API call found in the AST.")
        if not audit_ok:
            print("ABORT: source safety boundary violated before any GDK initialization.")
            report_path.write_text(buffer.getvalue(), encoding="utf-8")
            return 2

        doc_path = locate_doc(args.doc, source_path)
        checks: List[Check] = []
        banner("[SDK DOCUMENT PROVENANCE]")
        if doc_path is None:
            print("status       : FAIL")
            print("required file: GDK_SDK_4.1.5.md")
            print("searched explicit path, script directory, cwd, project directory, and Downloads")
        else:
            raw_doc = doc_path.read_bytes()
            doc_text = raw_doc.decode("utf-8-sig", errors="replace")
            print("status       : PASS")
            print(f"path         : {doc_path}")
            print(f"size         : {len(raw_doc):,} bytes")
            print(f"sha256       : {hashlib.sha256(raw_doc).hexdigest()}")
            print(f"line count   : {doc_text.count(chr(10)) + 1:,}")
            checks = doc_checks(doc_text)
            print_checks(checks)

        runtime: Optional[Dict[str, Any]] = None
        if args.doc_only:
            banner("[LIVE READ-ONLY CROSS-CHECK]")
            print("SKIPPED by --doc-only")
        else:
            runtime = runtime_probe()
            print_runtime(runtime)

        matrix = contract_matrix(checks) if checks else []
        print_matrix(matrix)
        print_summary(audit_ok, doc_path, checks, runtime, matrix)

    report_path.write_text(buffer.getvalue(), encoding="utf-8")
    print(f"Report path: {report_path}")

    doc_ok = bool(doc_path) and required_pass(checks)
    runtime_ok = True if args.doc_only else bool(runtime and live_contract_pass(runtime))
    return 0 if audit_ok and doc_ok and runtime_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
