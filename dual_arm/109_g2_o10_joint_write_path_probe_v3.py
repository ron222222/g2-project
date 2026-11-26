#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
109_v3 G2 + O10 Joint Write-Path Probe
======================================

Purpose
-------
Confirm, without commanding motion, the Python-side contracts and detached
request-object layouts for these three candidate O10 write paths:

1. Robot.joint_control_request(JointControlReq)
2. Robot.move_ee_pos(JointStates)
3. Robot.joint_servo_control(JointServoControlReq, enable_low_latency=False)

Safety model
------------
* This program NEVER calls a motion/control/write API.
* It calls only GDK initialization/release and read-only state APIs.
* Candidate request objects are built only in local Python memory and are never
  passed to Robot.
* A source-level AST safety audit runs before GDK initialization and aborts if
  a forbidden Robot control call is present.
* No joint target is changed: detached candidates mirror the current measured
  positions and use zero velocities.

The probe cannot prove undocumented server-side semantics (for example exact
units or the accepted vocabulary for JointStates.group/target_type) without
transmitting a request. It reports those points as UNRESOLVED instead of
inventing values.
"""

from __future__ import annotations

import ast
import datetime as _dt
import inspect
import json
import os
from pathlib import Path
import platform
import re
import shlex
import subprocess
import sys
import time
import traceback
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


PROGRAM_NAME = "109_v3 O10 Joint Write-Path Probe"
ENV_SCRIPT = Path("~/.cache/agibot/app/env.sh").expanduser()
ENV_MARKER = "AGIBOT_109_V3_ENV_READY"
REPORT_PREFIX = "109_g2_o10_joint_write_path_probe_v3_report"
MAX_STATE_ATTEMPTS = 10
STATE_RETRY_SECONDS = 1.0

# These names may be inspected or printed, but must never be invoked by this
# non-motion probe. The AST audit detects direct attribute calls and simple
# name calls whose final identifier is one of these values.
FORBIDDEN_CALL_NAMES = {
    "joint_control_request",
    "joint_servo_control",
    "move_ee_pos",
    "move_end_effector_joint",
    "end_effector_pose_control",
    "force_position_control",
    "motion_plan_request",
    "move_arm_joint",
    "move_arm_joint_servo",
    "move_head_joint",
    "move_head_joint_servo",
    "move_waist_joint",
    "move_waist_joint_servo",
    "servo_control_arm_pos",
    "set_control_mode",
    "set_reference_frame_pose",
    "set_reference_frame_poses",
    "trajectory_tracking_control",
}
READ_ONLY_CALL_NAMES = {
    "get_end_state",
    "get_joint_states",
    "get_motion_control_status",
}
CANDIDATE_METHODS = (
    "joint_control_request",
    "move_ee_pos",
    "joint_servo_control",
)
CANDIDATE_TYPES = (
    "JointControlReq",
    "JointServoControlReq",
    "JointState",
    "JointStates",
)


class Tee:
    """Write console output to both the terminal and a UTF-8 report file."""

    def __init__(self, *streams: Any) -> None:
        self.streams = streams

    def write(self, text: str) -> int:
        for stream in self.streams:
            stream.write(text)
            stream.flush()
        return len(text)

    def flush(self) -> None:
        for stream in self.streams:
            stream.flush()


class ProbeLedger:
    def __init__(self) -> None:
        self.source_audit_pass = False
        self.gdk_initialized = False
        self.robot_created = False
        self.end_state_read = False
        self.joint_states_read = False
        self.gdk_released = False
        self.control_api_invoked = False
        self.candidate_objects_built: Dict[str, int] = {
            "JointControlReq": 0,
            "JointServoControlReq": 0,
            "JointStates": 0,
        }
        self.errors: List[str] = []
        self.unresolved: List[str] = []


LEDGER = ProbeLedger()


def heading(title: str, char: str = "=") -> None:
    print("\n" + char * 108)
    print(title)
    print(char * 108)


def safe_repr(value: Any, limit: int = 320) -> str:
    try:
        text = repr(value)
    except Exception as exc:  # pragma: no cover - defensive for opaque pybind objects
        text = f"<repr failed: {type(exc).__name__}: {exc}>"
    return text if len(text) <= limit else text[: limit - 3] + "..."


def type_name(value: Any) -> str:
    t = type(value)
    return f"{getattr(t, '__module__', '?')}.{getattr(t, '__name__', str(t))}"


def visible_public_names(value: Any) -> List[str]:
    try:
        return sorted(name for name in dir(value) if not name.startswith("_"))
    except Exception:
        return []


def ensure_gdk_environment() -> None:
    """Restart once through the official env.sh when needed."""
    if os.environ.get(ENV_MARKER) == "1":
        return
    if not ENV_SCRIPT.is_file():
        print(f"[ENV] Official environment script not found: {ENV_SCRIPT}")
        print("[ENV] Continuing in the current environment; import may fail.")
        os.environ[ENV_MARKER] = "1"
        return

    script = str(Path(__file__).resolve())
    argv = [script, *sys.argv[1:]]
    python_bin = "/usr/bin/python3.10" if Path("/usr/bin/python3.10").is_file() else sys.executable
    cmd = " ".join(shlex.quote(x) for x in [python_bin, *argv])
    shell = (
        f"export {ENV_MARKER}=1; "
        f"source {shlex.quote(str(ENV_SCRIPT))}; "
        f"exec {cmd}"
    )
    print("[ENV] Restarting once with the official GDK environment ...", flush=True)
    os.execv("/bin/bash", ["bash", "-lc", shell])


def source_safety_audit() -> Tuple[bool, List[str]]:
    """Reject source containing a direct call to any known write API."""
    findings: List[str] = []
    source_path = Path(__file__).resolve()
    tree = ast.parse(source_path.read_text(encoding="utf-8"), filename=str(source_path))

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        called: Optional[str] = None
        if isinstance(node.func, ast.Attribute):
            called = node.func.attr
        elif isinstance(node.func, ast.Name):
            called = node.func.id
        if called in FORBIDDEN_CALL_NAMES:
            findings.append(f"line {node.lineno}: forbidden call {called}(...)")

    # Prevent accidental bypass through eval/exec. getattr is allowed because
    # the probe uses it only for metadata and detached data fields.
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            if node.func.id in {"eval", "exec"}:
                findings.append(f"line {node.lineno}: dynamic execution {node.func.id}(...) is forbidden")

    return not findings, findings


def describe_symbol(label: str, value: Any) -> None:
    print(f"\n{label}")
    print(f"  Python type     : {type_name(value)}")
    print(f"  callable        : {callable(value)}")
    try:
        sig = inspect.signature(value)
    except Exception as exc:
        sig = f"<unavailable: {type(exc).__name__}: {exc}>"
    print(f"  signature       : {sig}")
    doc = inspect.getdoc(value) or "<none>"
    print("  __doc__         :")
    for line in doc.splitlines() or [doc]:
        print(f"    {line}")
    for attr in ("__text_signature__", "__module__", "__qualname__"):
        try:
            print(f"  {attr:<15}: {safe_repr(getattr(value, attr, None))}")
        except Exception as exc:
            print(f"  {attr:<15}: <error: {type(exc).__name__}: {exc}>")


def describe_descriptor(cls: Any, field: str) -> None:
    print(f"  field {field!r}:")
    try:
        descriptor = inspect.getattr_static(cls, field)
    except Exception as exc:
        print(f"    descriptor     : <unavailable: {type(exc).__name__}: {exc}>")
        return
    print(f"    descriptor type: {type_name(descriptor)}")
    print(f"    descriptor repr: {safe_repr(descriptor)}")
    doc = inspect.getdoc(descriptor) or "<none>"
    print(f"    descriptor doc : {doc}")


def scalar_snapshot(obj: Any) -> Dict[str, Any]:
    snap: Dict[str, Any] = {}
    for name in visible_public_names(obj):
        try:
            value = getattr(obj, name)
        except Exception as exc:
            snap[name] = f"<read error: {type(exc).__name__}: {exc}>"
            continue
        if callable(value):
            continue
        if isinstance(value, list):
            snap[name] = {
                "type": type_name(value),
                "length": len(value),
                "value": [safe_repr(x, 120) for x in value[:3]],
            }
        elif isinstance(value, (str, int, float, bool, type(None))):
            snap[name] = value
        else:
            snap[name] = {
                "type": type_name(value),
                "repr": safe_repr(value),
            }
    return snap


def round_trip_assignment(obj: Any, field: str, value: Any) -> Tuple[bool, str]:
    """Assign to a detached object, read back, then restore the original value."""
    try:
        original = getattr(obj, field)
    except Exception as exc:
        return False, f"field read failed: {type(exc).__name__}: {exc}"
    try:
        setattr(obj, field, value)
        observed = getattr(obj, field)
        if isinstance(observed, list):
            detail = f"accepted; read-back type=list length={len(observed)} value={safe_repr(observed)}"
        else:
            detail = f"accepted; read-back type={type_name(observed)} value={safe_repr(observed)}"
        return True, detail
    except Exception as exc:
        return False, f"rejected: {type(exc).__name__}: {exc}"
    finally:
        try:
            setattr(obj, field, original)
        except Exception:
            pass


def constructor_and_descriptor_probe(gdk: Any, type_name_: str, tests: Mapping[str, Any]) -> None:
    heading(f"[DETACHED DATA-CONTAINER PROBE: {type_name_}]", "-")
    cls = getattr(gdk, type_name_, None)
    if cls is None:
        print("type present    : NO")
        LEDGER.errors.append(f"Missing required type: {type_name_}")
        return
    print("type present    : YES")
    describe_symbol(f"agibot_gdk.{type_name_}", cls)
    try:
        obj = cls()
    except Exception as exc:
        print(f"constructor     : FAIL ({type(exc).__name__}: {exc})")
        LEDGER.errors.append(f"{type_name_}() failed: {type(exc).__name__}: {exc}")
        return
    print("constructor     : PASS (detached local object only)")
    print("default snapshot:")
    print(json.dumps(scalar_snapshot(obj), ensure_ascii=False, indent=2, default=str))

    print("\nfield descriptors and safe round-trip assignment tests:")
    for field, test_value in tests.items():
        describe_descriptor(cls, field)
        ok, detail = round_trip_assignment(obj, field, test_value)
        print(f"    assignment    : {'PASS' if ok else 'FAIL'} - {detail}")


def get_mapping_value(container: Any, key: str, default: Any = None) -> Any:
    if isinstance(container, Mapping):
        return container.get(key, default)
    return getattr(container, key, default)


def float_or_none(value: Any) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def extract_end_state(end_state_result: Any) -> Dict[str, Dict[str, Any]]:
    extracted: Dict[str, Dict[str, Any]] = {}
    for side_key in ("left_end_state", "right_end_state"):
        side = get_mapping_value(end_state_result, side_key)
        if side is None:
            continue
        names = list(get_mapping_value(side, "names", []) or [])
        states = list(get_mapping_value(side, "end_states", []) or [])
        positions: List[Optional[float]] = []
        velocities: List[Optional[float]] = []
        state_rows: List[Dict[str, Any]] = []
        for index, name in enumerate(names):
            state = states[index] if index < len(states) else None
            position = float_or_none(get_mapping_value(state, "position"))
            velocity = float_or_none(get_mapping_value(state, "velocity"))
            positions.append(position)
            velocities.append(velocity)
            state_rows.append(
                {
                    "index": index,
                    "name": str(name),
                    "position": position,
                    "velocity": velocity,
                    "state_type": type_name(state) if state is not None else None,
                }
            )
        extracted[side_key] = {
            "controlled": get_mapping_value(side, "controlled"),
            "type": get_mapping_value(side, "type"),
            "names": names,
            "states_count": len(states),
            "positions": positions,
            "velocities": velocities,
            "rows": state_rows,
        }
    return extracted


def read_end_state_with_retry(robot: Any) -> Tuple[Any, Dict[str, Dict[str, Any]]]:
    heading("[READ-ONLY get_end_state() WITH RETRY]")
    last_error: Optional[BaseException] = None
    for attempt in range(1, MAX_STATE_ATTEMPTS + 1):
        try:
            raw = robot.get_end_state()
            extracted = extract_end_state(raw)
            total_names = sum(len(x.get("names", [])) for x in extracted.values())
            if total_names:
                print(f"attempt {attempt}/{MAX_STATE_ATTEMPTS}: PASS ({total_names} joint names)")
                LEDGER.end_state_read = True
                return raw, extracted
            print(f"attempt {attempt}/{MAX_STATE_ATTEMPTS}: empty or unrecognized state")
        except Exception as exc:
            last_error = exc
            print(f"attempt {attempt}/{MAX_STATE_ATTEMPTS}: no state ({type(exc).__name__}: {exc})")
        if attempt < MAX_STATE_ATTEMPTS:
            time.sleep(STATE_RETRY_SECONDS)
    if last_error:
        LEDGER.errors.append(f"get_end_state failed: {type(last_error).__name__}: {last_error}")
    else:
        LEDGER.errors.append("get_end_state returned no usable joint state")
    return None, {}


def summarize_end_state(extracted: Mapping[str, Mapping[str, Any]]) -> None:
    heading("[CURRENT O10 STATE - READ ONLY]")
    if not extracted:
        print("No usable O10 end state was read.")
        return
    for side_key, info in extracted.items():
        print(f"\n{side_key}:")
        print(f"  controlled : {info.get('controlled')}")
        print(f"  type       : {info.get('type')}")
        print(f"  names      : {len(info.get('names', []))}")
        print(f"  states     : {info.get('states_count')}")
        for row in info.get("rows", []):
            print(
                f"  [{row['index']:02d}] {row['name']}: "
                f"position={row['position']}, velocity={row['velocity']}, "
                f"state_type={row['state_type']}"
            )


def summarize_unknown(value: Any, depth: int = 0, max_depth: int = 4) -> Any:
    if depth >= max_depth:
        return {"type": type_name(value), "repr": safe_repr(value)}
    if isinstance(value, Mapping):
        return {
            str(k): summarize_unknown(v, depth + 1, max_depth)
            for k, v in list(value.items())[:80]
        }
    if isinstance(value, (list, tuple)):
        return [summarize_unknown(v, depth + 1, max_depth) for v in list(value)[:30]]
    if isinstance(value, (str, int, float, bool, type(None))):
        return value
    snap = scalar_snapshot(value)
    return {"type": type_name(value), "fields": snap, "repr": safe_repr(value)}


def read_joint_states_once(robot: Any) -> Any:
    heading("[READ-ONLY get_joint_states()]")
    try:
        raw = robot.get_joint_states()
        LEDGER.joint_states_read = True
        print("call status : PASS")
        print(f"raw type    : {type_name(raw)}")
        print("bounded structural summary:")
        print(json.dumps(summarize_unknown(raw), ensure_ascii=False, indent=2, default=str))
        return raw
    except Exception as exc:
        print(f"call status : FAIL ({type(exc).__name__}: {exc})")
        LEDGER.errors.append(f"get_joint_states failed: {type(exc).__name__}: {exc}")
        return None


def candidate_method_contracts(gdk: Any, robot: Any) -> None:
    heading("[CANDIDATE WRITE-PATH METHOD CONTRACTS - METADATA ONLY]")
    robot_cls = getattr(gdk, "Robot", None)
    for method_name in CANDIDATE_METHODS:
        print(f"\n{method_name}")
        for owner_name, owner in (("Robot class", robot_cls), ("Robot instance", robot)):
            if owner is None or not hasattr(owner, method_name):
                print(f"  {owner_name:<14}: ABSENT")
                continue
            value = getattr(owner, method_name)
            print(f"  {owner_name:<14}: PRESENT ({type_name(value)})")
            doc = inspect.getdoc(value) or "<none>"
            print(f"    doc: {doc}")
            try:
                sig = inspect.signature(value)
            except Exception as exc:
                sig = f"<unavailable: {type(exc).__name__}: {exc}>"
            print(f"    inspect.signature: {sig}")
    print("\nIMPORTANT: presence and pybind docstrings were inspected; none of these methods was called.")


def make_joint_state(gdk: Any, name: str, position: float, velocity: float) -> Any:
    state = gdk.JointState()
    # These assignments affect a detached local object only.
    state.name = str(name)
    state.position = float(position)
    state.velocity = float(velocity)
    return state


def build_detached_candidates(gdk: Any, extracted: Mapping[str, Mapping[str, Any]]) -> Dict[str, Any]:
    heading("[DETACHED CURRENT-STATE CANDIDATES - NEVER TRANSMITTED]")
    candidates: Dict[str, Any] = {}
    if not extracted:
        print("SKIP: current O10 state unavailable; no candidate objects built.")
        return candidates

    for side_key, info in extracted.items():
        names = [str(x) for x in info.get("names", [])]
        raw_positions = info.get("positions", [])
        if not names or len(raw_positions) != len(names) or any(x is None for x in raw_positions):
            print(f"{side_key}: SKIP - incomplete names/positions")
            continue
        positions = [float(x) for x in raw_positions]
        zero_velocities = [0.0] * len(names)
        side_tag = "left" if side_key.startswith("left") else "right"

        try:
            req = gdk.JointControlReq()
            req.joint_names = list(names)
            req.joint_positions = list(positions)
            req.joint_velocities = list(zero_velocities)
            # No unit is assumed. Zero is retained as the binding default.
            req.life_time = 0.0
            # Non-semantic marker on a detached object; never sent to Robot.
            req.detail = "109_v3_detached_probe_never_transmitted"
            candidates[f"{side_tag}.JointControlReq"] = req
            LEDGER.candidate_objects_built["JointControlReq"] += 1
            print(f"{side_key} JointControlReq : PASS; vector lengths "
                  f"names/positions/velocities={len(names)}/{len(positions)}/{len(zero_velocities)}")
        except Exception as exc:
            print(f"{side_key} JointControlReq : FAIL ({type(exc).__name__}: {exc})")
            LEDGER.errors.append(f"Detached JointControlReq build failed for {side_key}: {exc}")

        try:
            req = gdk.JointServoControlReq()
            req.joint_names = list(names)
            req.joint_positions = list(positions)
            req.joint_velocities = list(zero_velocities)
            # No unit is assumed. Zero is retained as the binding default.
            req.control_period = 0.0
            candidates[f"{side_tag}.JointServoControlReq"] = req
            LEDGER.candidate_objects_built["JointServoControlReq"] += 1
            print(f"{side_key} JointServoControlReq : PASS; vector lengths "
                  f"names/positions/velocities={len(names)}/{len(positions)}/{len(zero_velocities)}")
        except Exception as exc:
            print(f"{side_key} JointServoControlReq : FAIL ({type(exc).__name__}: {exc})")
            LEDGER.errors.append(f"Detached JointServoControlReq build failed for {side_key}: {exc}")

        try:
            states = [
                make_joint_state(gdk, name, position, 0.0)
                for name, position in zip(names, positions)
            ]
            req = gdk.JointStates()
            req.nums = len(states)
            req.states = states
            # group and target_type are deliberately not guessed. Their empty
            # defaults are preserved until documentary/read-only evidence gives
            # their accepted string vocabulary.
            candidates[f"{side_tag}.JointStates"] = req
            LEDGER.candidate_objects_built["JointStates"] += 1
            print(f"{side_key} JointStates : PASS; nums={req.nums}, states={len(req.states)}, "
                  f"group={safe_repr(req.group)}, target_type={safe_repr(req.target_type)}")
        except Exception as exc:
            print(f"{side_key} JointStates : FAIL ({type(exc).__name__}: {exc})")
            LEDGER.errors.append(f"Detached JointStates build failed for {side_key}: {exc}")

    print("\nCandidate snapshots (bounded; all objects remain detached):")
    for key, obj in candidates.items():
        print(f"\n{key}")
        print(json.dumps(scalar_snapshot(obj), ensure_ascii=False, indent=2, default=str))
    return candidates


def static_text_search(gdk: Any) -> None:
    """Search nearby text/stub files for exact API terms; never execute them."""
    heading("[LOCAL PACKAGE TEXT/STUB SEARCH - READ ONLY]")
    module_file = getattr(gdk, "__file__", None)
    if not module_file:
        print("Module file unavailable; search skipped.")
        return
    module_path = Path(module_file).resolve()
    print(f"module file : {module_path}")
    root = module_path.parent
    allowed_suffixes = {".py", ".pyi", ".txt", ".md", ".rst"}
    terms = [
        "joint_control_request",
        "JointControlReq",
        "joint_servo_control",
        "JointServoControlReq",
        "move_ee_pos",
        "JointStates",
        "life_time",
        "control_period",
        "target_type",
    ]
    hits = 0
    files_seen = 0
    try:
        paths = sorted(root.rglob("*"))
    except Exception as exc:
        print(f"package tree scan failed: {type(exc).__name__}: {exc}")
        return
    for path in paths:
        if files_seen >= 250 or hits >= 80:
            break
        try:
            if not path.is_file() or path.suffix.lower() not in allowed_suffixes:
                continue
            if path.stat().st_size > 2_000_000:
                continue
            files_seen += 1
            text = path.read_text(encoding="utf-8", errors="replace")
        except Exception:
            continue
        for line_no, line in enumerate(text.splitlines(), 1):
            if any(term in line for term in terms):
                rel = path.relative_to(root)
                clean = line.strip()
                if len(clean) > 300:
                    clean = clean[:297] + "..."
                print(f"{rel}:{line_no}: {clean}")
                hits += 1
                if hits >= 80:
                    break
    print(f"searched text/stub files: {files_seen}; matching lines: {hits}")
    if not hits:
        print("No additional textual contract evidence found beside pybind metadata.")


def report_contract_matrix(gdk: Any) -> None:
    heading("[WRITE-PATH CONTRACT MATRIX]")
    rows = [
        (
            "joint_control_request",
            "JointControlReq",
            "joint_names:list; joint_positions:list; joint_velocities:list; life_time:float; detail:str",
            "life_time unit; detail grammar; whether velocities may be empty; server length validation; return-code semantics",
        ),
        (
            "move_ee_pos",
            "JointStates",
            "nums:int; states:list[JointState]; group:str; target_type:str; timestamp:int",
            "group vocabulary; target_type vocabulary; timestamp unit/clock; accepted O10 group; return-code semantics",
        ),
        (
            "joint_servo_control",
            "JointServoControlReq",
            "joint_names:list; joint_positions:list; joint_velocities:list; control_period:float; enable_low_latency:bool",
            "control_period unit/range; stream cadence/watchdog; velocity optionality; server length validation; return-code semantics",
        ),
    ]
    for method, req_type, fields, unresolved in rows:
        method_present = hasattr(getattr(gdk, "Robot", object), method)
        type_present = hasattr(gdk, req_type)
        print(f"\n{method}")
        print(f"  method present          : {'YES' if method_present else 'NO'}")
        print(f"  request type present    : {'YES' if type_present else 'NO'} ({req_type})")
        print(f"  Python-side fields      : {fields}")
        print(f"  server semantics        : UNRESOLVED - {unresolved}")
        LEDGER.unresolved.append(f"{method}: {unresolved}")


def final_summary(report_path: Path) -> None:
    heading("109_v3 NON-MOTION JOINT WRITE-PATH PROBE SUMMARY")
    print(f"source safety audit          : {'PASS' if LEDGER.source_audit_pass else 'FAIL'}")
    print(f"GDK initialized              : {'PASS' if LEDGER.gdk_initialized else 'FAIL'}")
    print(f"Robot created                : {'PASS' if LEDGER.robot_created else 'FAIL'}")
    print(f"get_end_state read           : {'PASS' if LEDGER.end_state_read else 'FAIL'}")
    print(f"get_joint_states read        : {'PASS' if LEDGER.joint_states_read else 'FAIL'}")
    print(f"GDK released                 : {'PASS' if LEDGER.gdk_released else 'FAIL'}")
    print(f"motion/control API invoked   : {'YES - SAFETY FAILURE' if LEDGER.control_api_invoked else 'NO'}")
    print("O10 commanded                : NO")
    print("Arm commanded                : NO")
    print("Motion executed              : NO")
    print("\nDetached candidate objects built (never transmitted):")
    for key, count in LEDGER.candidate_objects_built.items():
        print(f"  {key:<24}: {count}")
    print("\nFinal boundary:")
    print("  CONFIRMED: binding presence, pybind docstrings, writable detached fields,")
    print("             exact live O10 joint-name vectors, current-state-shaped local payloads.")
    print("  UNRESOLVED: server-side semantic validation and units that require vendor")
    print("              documentation or a separately reviewed motion-enabled test.")
    if LEDGER.errors:
        print("\nErrors/limitations:")
        for item in LEDGER.errors:
            print(f"  - {item}")
    print(f"\nReport path: {report_path}")


def main() -> int:
    ensure_gdk_environment()

    timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    report_path = Path(__file__).resolve().with_name(f"{REPORT_PREFIX}_{timestamp}.txt")
    original_stdout = sys.stdout

    with report_path.open("w", encoding="utf-8") as report_file:
        sys.stdout = Tee(original_stdout, report_file)
        try:
            heading(PROGRAM_NAME)
            print(f"script        : {Path(__file__).resolve()}")
            print(f"timestamp     : {_dt.datetime.now().isoformat(timespec='seconds')}")
            print(f"python        : {sys.version.replace(chr(10), ' ')}")
            print(f"platform      : {platform.platform()}")
            print(f"safety mode   : READ/INTROSPECT/DETACHED-CONSTRUCT ONLY")

            heading("[SOURCE SAFETY AUDIT]")
            ok, findings = source_safety_audit()
            LEDGER.source_audit_pass = ok
            print(f"status : {'PASS' if ok else 'FAIL'}")
            if findings:
                for finding in findings:
                    print(f"  - {finding}")
                print("ABORT: a forbidden control/write call exists in the source.")
                return 20
            print("No direct motion/control/write API call found in the AST.")

            heading("[IMPORT AND MODULE PROVENANCE]")
            try:
                import agibot_gdk as gdk
            except Exception as exc:
                print(f"import agibot_gdk : FAIL ({type(exc).__name__}: {exc})")
                LEDGER.errors.append(f"agibot_gdk import failed: {type(exc).__name__}: {exc}")
                return 21
            print("import agibot_gdk : PASS")
            print(f"module file       : {getattr(gdk, '__file__', None)}")
            print(f"module version    : {getattr(gdk, '__version__', '<not exposed>')}")

            heading("[CANDIDATE TYPE PRESENCE]")
            for name in CANDIDATE_TYPES:
                print(f"{name:<28}: {'YES' if hasattr(gdk, name) else 'NO'}")

            # Assignment probes use only detached local objects.
            constructor_and_descriptor_probe(
                gdk,
                "JointControlReq",
                {
                    "detail": "probe-only",
                    "joint_names": ["probe_joint_a"],
                    "joint_positions": [0.0],
                    "joint_velocities": [0.0],
                    "life_time": 0.0,
                },
            )
            constructor_and_descriptor_probe(
                gdk,
                "JointServoControlReq",
                {
                    "control_period": 0.0,
                    "joint_names": ["probe_joint_a"],
                    "joint_positions": [0.0],
                    "joint_velocities": [0.0],
                },
            )
            constructor_and_descriptor_probe(
                gdk,
                "JointState",
                {
                    "name": "probe_joint_a",
                    "position": 0.0,
                    "velocity": 0.0,
                    "effort": 0.0,
                    "mode": 0,
                },
            )
            joint_state_sample: List[Any] = []
            if hasattr(gdk, "JointState"):
                try:
                    joint_state_sample = [make_joint_state(gdk, "probe_joint_a", 0.0, 0.0)]
                except Exception as exc:
                    print(f"JointState sample creation failed: {type(exc).__name__}: {exc}")
            constructor_and_descriptor_probe(
                gdk,
                "JointStates",
                {
                    "group": "probe-only",
                    "nums": len(joint_state_sample),
                    "states": joint_state_sample,
                    "target_type": "probe-only",
                    "timestamp": 0,
                },
            )

            static_text_search(gdk)

            heading("[GDK INITIALIZATION]")
            init_result = gdk.gdk_init()
            print(f"gdk_init result : {safe_repr(init_result)}")
            success_value = getattr(getattr(gdk, "GDKRes", object), "kSuccess", None)
            LEDGER.gdk_initialized = bool(init_result == success_value) if success_value is not None else True
            print(f"gdk_init status : {'PASS' if LEDGER.gdk_initialized else 'FAIL'}")
            if not LEDGER.gdk_initialized:
                LEDGER.errors.append(f"gdk_init did not return kSuccess: {safe_repr(init_result)}")
                return 22

            try:
                robot = gdk.Robot()
                LEDGER.robot_created = True
                print("Robot() status : PASS")
            except Exception as exc:
                print(f"Robot() status : FAIL ({type(exc).__name__}: {exc})")
                LEDGER.errors.append(f"Robot() failed: {type(exc).__name__}: {exc}")
                return 23

            candidate_method_contracts(gdk, robot)
            _raw_end_state, extracted = read_end_state_with_retry(robot)
            summarize_end_state(extracted)
            read_joint_states_once(robot)
            build_detached_candidates(gdk, extracted)
            report_contract_matrix(gdk)

            return 0
        except KeyboardInterrupt:
            print("\nInterrupted by user.")
            LEDGER.errors.append("Interrupted by user")
            return 130
        except Exception as exc:
            print(f"\nUNEXPECTED ERROR: {type(exc).__name__}: {exc}")
            traceback.print_exc()
            LEDGER.errors.append(f"Unexpected error: {type(exc).__name__}: {exc}")
            return 99
        finally:
            # gdk_release is lifecycle cleanup, not a motion API.
            gdk_obj = locals().get("gdk")
            if LEDGER.gdk_initialized and gdk_obj is not None:
                try:
                    result = gdk_obj.gdk_release()
                    print(f"\n[GDK] gdk_release result : {safe_repr(result)}")
                    success_value = getattr(getattr(gdk_obj, "GDKRes", object), "kSuccess", None)
                    LEDGER.gdk_released = bool(result == success_value) if success_value is not None else True
                except Exception as exc:
                    print(f"\n[GDK] gdk_release failed: {type(exc).__name__}: {exc}")
                    LEDGER.errors.append(f"gdk_release failed: {type(exc).__name__}: {exc}")
            final_summary(report_path)
            sys.stdout = original_stdout


if __name__ == "__main__":
    raise SystemExit(main())
