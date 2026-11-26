#!/usr/bin/env python3
"""
109_v4.1 Hold Payload Validation for G2 + O10 (GDK 4.1.5).

PURPOSE
-------
Read the live LEFT O10 state, construct a detached JointStates payload that
holds the exact current positions, and validate the payload against the
GDK 4.1.5 move_ee_pos() contract.

HARD SAFETY BOUNDARY
--------------------
This program never calls move_ee_pos() or any other motion/control/write API.
It only calls Robot.get_end_state(), constructs Python objects in local memory,
validates them, prints a report, and releases GDK.

A source-level AST audit runs before GDK import/initialization. Any direct call
or literal getattr call to a forbidden motion/control/write method terminates
the program before a Robot is created.

DOCUMENT TRUTH SOURCE
---------------------
File: GDK_SDK_4.1.5.md
SHA256: e952f9d0c383c468b7384c46274b12d3059827c2c95ecb0506082a48c49df90d
Python section: Robot.move_ee_pos() / O10_T2
Embedded contract:
  * group == "left_tool"
  * nums == len(states) == 10
  * each state only needs position
  * positions are ordered by the documented LEFT O10 joint table
  * units are radians for O10_T2 limits

This program is evidence collection only. Passing v4.1 does not authorize or
perform the later 110 micro-motion.
"""

from __future__ import annotations

import argparse
import ast
import copy
import hashlib
import importlib
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


SCRIPT_NAME = "109_v4.1 Hold Payload Validation"
SDK_DOCUMENT_NAME = "GDK_SDK_4.1.5.md"
SDK_DOCUMENT_SHA256 = "e952f9d0c383c468b7384c46274b12d3059827c2c95ecb0506082a48c49df90d"
EXPECTED_GROUP = "left_tool"
EXPECTED_COUNT = 10
DEFAULT_RETRIES = 10
DEFAULT_RETRY_INTERVAL_SECONDS = 0.5
DEFAULT_GDK_ENV = Path.home() / ".cache/agibot/app/env.sh"
BOOTSTRAP_MARKER = "AGIBOT_GDK_ENV_RESTARTED_FOR_109_V4_1"

# Exact LEFT O10_T2 order and limits from GDK_SDK_4.1.5.md.
# Limit units: radians.
LEFT_O10_CONTRACT: Tuple[Tuple[str, float, float], ...] = (
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
EXPECTED_NAMES: Tuple[str, ...] = tuple(item[0] for item in LEFT_O10_CONTRACT)
LIMITS: Dict[str, Tuple[float, float]] = {
    name: (minimum, maximum) for name, minimum, maximum in LEFT_O10_CONTRACT
}

# No member of this set may appear as a directly invoked attribute/name or as
# the literal method name in getattr(...). The list intentionally covers the
# three prior candidates plus common GDK motion/control entry points.
FORBIDDEN_CALL_NAMES = frozenset(
    {
        "move_ee_pos",
        "move_end_effector_joint",
        "joint_control_request",
        "joint_servo_control",
        "arm_joint_servo_control",
        "end_effector_pose_control",
        "move_arm",
        "move_joints",
        "move_joint",
        "move_pose",
        "set_control_mode",
        "switch_end_pose_control_frame",
        "stop_motion",
        "emergency_stop",
    }
)
ALLOWED_ROBOT_CALL_NAMES = frozenset({"get_end_state"})
# NO_DEFAULT means a field is required. MISSING is a legitimate caller-visible
# marker used when an optional field is absent.
NO_DEFAULT = object()
MISSING = object()


@dataclass
class Check:
    name: str
    passed: bool
    detail: str


@dataclass
class SourceSnapshot:
    names: List[str] = field(default_factory=list)
    positions: List[float] = field(default_factory=list)
    raw_states: List[Any] = field(default_factory=list)
    controlled: Any = None
    end_type: Any = None


@dataclass
class ValidationResult:
    checks: List[Check] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    snapshot: Optional[SourceSnapshot] = None
    payload: Any = None
    attempts_used: int = 0
    last_read_error: Optional[str] = None

    @property
    def passed(self) -> bool:
        return bool(self.checks) and all(check.passed for check in self.checks)

    def add(self, name: str, passed: bool, detail: str) -> None:
        self.checks.append(Check(name=name, passed=bool(passed), detail=detail))


class Reporter:
    def __init__(self) -> None:
        self.lines: List[str] = []

    def emit(self, text: str = "") -> None:
        value = str(text)
        print(value, flush=True)
        self.lines.append(value)

    def section(self, title: str) -> None:
        self.emit("=" * 108)
        self.emit(title)
        self.emit("=" * 108)

    def write(self, path: Path) -> None:
        path.write_text("\n".join(self.lines) + "\n", encoding="utf-8")


class SafetyAuditVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.violations: List[str] = []

    @staticmethod
    def _call_name(func: ast.AST) -> Optional[str]:
        if isinstance(func, ast.Name):
            return func.id
        if isinstance(func, ast.Attribute):
            return func.attr
        return None

    def visit_Call(self, node: ast.Call) -> Any:
        name = self._call_name(node.func)
        if name in FORBIDDEN_CALL_NAMES:
            self.violations.append(
                f"line {node.lineno}: forbidden direct call to {name}()"
            )

        # Detect getattr(obj, "forbidden_name")(...), including the inner
        # getattr call even when its return value is immediately invoked.
        if isinstance(node.func, ast.Name) and node.func.id == "getattr":
            if len(node.args) >= 2:
                literal = node.args[1]
                if isinstance(literal, ast.Constant) and isinstance(literal.value, str):
                    if literal.value in FORBIDDEN_CALL_NAMES:
                        self.violations.append(
                            f"line {node.lineno}: forbidden literal getattr for "
                            f"{literal.value}"
                        )

        # Stronger local boundary: a parameter/variable literally named
        # 'robot' may only be used for the one required read-only method.
        if isinstance(node.func, ast.Attribute):
            base = node.func.value
            if isinstance(base, ast.Name) and base.id == "robot":
                if node.func.attr not in ALLOWED_ROBOT_CALL_NAMES:
                    self.violations.append(
                        f"line {node.lineno}: robot.{node.func.attr}() is outside "
                        "the read-only allowlist"
                    )
        self.generic_visit(node)


def audit_source(path: Path) -> Tuple[bool, List[str], str]:
    try:
        data = path.read_bytes()
        source = data.decode("utf-8")
        tree = ast.parse(source, filename=str(path))
    except Exception as exc:
        return False, [f"source read/parse failed: {type(exc).__name__}: {exc}"], ""

    visitor = SafetyAuditVisitor()
    visitor.visit(tree)
    digest = hashlib.sha256(data).hexdigest()
    return not visitor.violations, visitor.violations, digest


def read_field(obj: Any, name: str, default: Any = NO_DEFAULT) -> Any:
    if isinstance(obj, Mapping):
        if name in obj:
            return obj[name]
    else:
        try:
            return getattr(obj, name)
        except (AttributeError, TypeError):
            pass
    if default is NO_DEFAULT:
        raise KeyError(f"missing required field: {name}")
    return default


def type_name(value: Any) -> str:
    cls = type(value)
    return f"{cls.__module__}.{cls.__name__}"


def short_exception(exc: BaseException) -> str:
    return f"{type(exc).__name__}: {exc}"


def finite_float(value: Any) -> Tuple[bool, Optional[float]]:
    if isinstance(value, bool):
        return False, None
    try:
        converted = float(value)
    except (TypeError, ValueError, OverflowError):
        return False, None
    return math.isfinite(converted), converted


def result_is_success(gdk_module: Any, value: Any) -> bool:
    try:
        return value == gdk_module.GDKRes.kSuccess
    except Exception:
        try:
            return int(value) == 0
        except Exception:
            return False


def import_gdk_or_restart(args: argparse.Namespace, reporter: Reporter) -> Any:
    try:
        return importlib.import_module("agibot_gdk")
    except Exception as first_exc:
        marker_present = os.environ.get(BOOTSTRAP_MARKER) == "1"
        if args.skip_env_bootstrap or marker_present:
            raise RuntimeError(
                "Unable to import agibot_gdk after the available bootstrap path: "
                f"{short_exception(first_exc)}"
            ) from first_exc

        env_script = Path(args.gdk_env).expanduser().resolve()
        if not env_script.is_file():
            raise RuntimeError(
                f"agibot_gdk import failed and GDK environment script was not found: "
                f"{env_script}; original error: {short_exception(first_exc)}"
            ) from first_exc

        reporter.emit("[ENV] Restarting once with the official GDK environment ...")
        command = (
            'source "$1" && shift && '
            f'export {BOOTSTRAP_MARKER}=1 && '
            'exec "$@"'
        )
        argv = [
            "/bin/bash",
            "-lc",
            command,
            "109-v4-1-bootstrap",
            str(env_script),
            sys.executable,
            str(Path(__file__).resolve()),
            *sys.argv[1:],
        ]
        sys.stdout.flush()
        sys.stderr.flush()
        os.execv("/bin/bash", argv)
        raise AssertionError("os.execv returned unexpectedly")


def get_end_state_with_retry(
    robot: Any,
    retries: int,
    interval_seconds: float,
    reporter: Reporter,
) -> Tuple[Optional[Any], int, Optional[str]]:
    reporter.section("[READ-ONLY get_end_state() WITH RETRY]")
    last_error: Optional[str] = None
    for attempt in range(1, retries + 1):
        try:
            value = robot.get_end_state()
            if value is None:
                raise RuntimeError("get_end_state() returned None")
            left = read_field(value, "left_end_state", None)
            names = read_field(left, "names", []) if left is not None else []
            states = read_field(left, "end_states", []) if left is not None else []
            names = list(names)
            states = list(states)
            if not names or not states:
                raise RuntimeError(
                    "left_end_state is not populated yet "
                    f"(names={len(names)}, end_states={len(states)})"
                )
            reporter.emit(
                f"attempt {attempt}/{retries}: PASS "
                f"({len(names)} left names, {len(states)} left states)"
            )
            return value, attempt, None
        except Exception as exc:
            last_error = short_exception(exc)
            reporter.emit(f"attempt {attempt}/{retries}: no state ({last_error})")
            if attempt < retries and interval_seconds > 0:
                time.sleep(interval_seconds)
    return None, retries, last_error


def extract_left_snapshot(end_state: Any, result: ValidationResult) -> Optional[SourceSnapshot]:
    try:
        left = read_field(end_state, "left_end_state")
        names_raw = read_field(left, "names")
        states_raw = read_field(left, "end_states")
        controlled = read_field(left, "controlled", None)
        end_type = read_field(left, "type", None)
        names = list(names_raw)
        states = list(states_raw)
    except Exception as exc:
        result.add("left_end_state structure", False, short_exception(exc))
        return None

    snapshot = SourceSnapshot(
        names=names,
        raw_states=states,
        controlled=controlled,
        end_type=end_type,
    )
    result.snapshot = snapshot

    result.add(
        "left_end_state controlled",
        controlled is True,
        f"controlled={controlled!r}",
    )
    result.add(
        "left joint-name count",
        len(names) == EXPECTED_COUNT,
        f"actual={len(names)}, expected={EXPECTED_COUNT}",
    )
    result.add(
        "left end-state count",
        len(states) == EXPECTED_COUNT,
        f"actual={len(states)}, expected={EXPECTED_COUNT}",
    )

    all_strings = all(isinstance(name, str) for name in names)
    result.add("joint names are strings", all_strings, f"types={[type_name(x) for x in names]}")

    whitespace_clean = all(
        isinstance(name, str) and name == name.strip() for name in names
    )
    result.add(
        "joint names have no edge whitespace",
        whitespace_clean,
        "all exact" if whitespace_clean else repr(names),
    )

    no_right_mix = all(
        isinstance(name, str) and "_hand_r_" not in name for name in names
    )
    result.add(
        "no right-hand joint mixed into left source",
        no_right_mix,
        "left-only" if no_right_mix else repr(names),
    )

    unique = len(set(names)) == len(names) if all_strings else False
    result.add("joint names are unique", unique, f"unique={len(set(names)) if all_strings else 'n/a'}")

    exact_order = tuple(names) == EXPECTED_NAMES
    mismatch_detail = "exact SDK order"
    if not exact_order:
        mismatches = []
        for index in range(max(len(names), len(EXPECTED_NAMES))):
            actual = names[index] if index < len(names) else "<missing>"
            expected = EXPECTED_NAMES[index] if index < len(EXPECTED_NAMES) else "<extra>"
            if actual != expected:
                mismatches.append(f"[{index}] actual={actual!r}, expected={expected!r}")
        mismatch_detail = "; ".join(mismatches[:10])
    result.add("joint order matches SDK O10_T2 table", exact_order, mismatch_detail)

    positions: List[float] = []
    position_errors: List[str] = []
    for index, state in enumerate(states):
        try:
            raw_position = read_field(state, "position")
            valid, position = finite_float(raw_position)
            if not valid or position is None:
                raise ValueError(f"non-finite/non-numeric position {raw_position!r}")
            positions.append(position)
        except Exception as exc:
            position_errors.append(f"[{index}] {short_exception(exc)}")
    snapshot.positions = positions
    result.add(
        "all source positions are finite numbers",
        len(positions) == EXPECTED_COUNT and not position_errors,
        "10 finite positions" if not position_errors and len(positions) == 10 else "; ".join(position_errors),
    )

    # Treat explicit unhealthy values as blockers. Missing optional health fields
    # are warnings because payload construction only requires current position.
    for index, state in enumerate(states):
        enable = read_field(state, "enable", MISSING)
        err_code = read_field(state, "err_code", MISSING)
        name = names[index] if index < len(names) else f"index_{index}"
        if enable is MISSING:
            result.warnings.append(f"{name}: enable field unavailable")
        elif enable is False:
            result.add(f"{name} enabled", False, "enable=False")
        if err_code is MISSING:
            result.warnings.append(f"{name}: err_code field unavailable")
        else:
            try:
                healthy = int(err_code) == 0
            except Exception:
                healthy = False
            result.add(f"{name} err_code", healthy, f"err_code={err_code!r}")

    return snapshot


def validate_document_limits(snapshot: SourceSnapshot, result: ValidationResult) -> None:
    reporter_ready = len(snapshot.names) == len(snapshot.positions)
    if not reporter_ready:
        result.add(
            "name-position alignment for limit checks",
            False,
            f"names={len(snapshot.names)}, positions={len(snapshot.positions)}",
        )
        return

    result.add(
        "name-position alignment for limit checks",
        True,
        f"pairs={len(snapshot.names)}",
    )
    for index, (name, position) in enumerate(zip(snapshot.names, snapshot.positions)):
        if name not in LIMITS:
            result.add(
                f"limit[{index}] {name}",
                False,
                "joint is not in the embedded LEFT O10 contract",
            )
            continue
        minimum, maximum = LIMITS[name]
        within = minimum <= position <= maximum
        margin = min(position - minimum, maximum - position)
        result.add(
            f"limit[{index}] {name}",
            within,
            f"position={position:.15g}, range=[{minimum:.15g}, {maximum:.15g}], "
            f"nearest_margin={margin:.15g}",
        )


def build_detached_hold_payload(gdk_module: Any, positions: Sequence[float]) -> Any:
    payload = gdk_module.JointStates()
    payload.group = EXPECTED_GROUP
    detached_states = []
    for position in positions:
        state = gdk_module.JointState()
        state.position = float(position)
        detached_states.append(state)
    payload.states = detached_states
    payload.nums = len(detached_states)
    return payload


def validate_payload(payload: Any, source_positions: Sequence[float], result: ValidationResult) -> None:
    try:
        group = read_field(payload, "group")
        nums = read_field(payload, "nums")
        states = list(read_field(payload, "states"))
    except Exception as exc:
        result.add("detached JointStates structure", False, short_exception(exc))
        return

    result.add(
        "payload group",
        group == EXPECTED_GROUP,
        f"actual={group!r}, expected={EXPECTED_GROUP!r}",
    )
    result.add(
        "payload nums",
        nums == EXPECTED_COUNT,
        f"actual={nums!r}, expected={EXPECTED_COUNT}",
    )
    result.add(
        "payload state count",
        len(states) == EXPECTED_COUNT,
        f"actual={len(states)}, expected={EXPECTED_COUNT}",
    )
    result.add(
        "payload nums equals len(states)",
        nums == len(states),
        f"nums={nums!r}, len(states)={len(states)}",
    )

    payload_positions: List[float] = []
    errors: List[str] = []
    for index, state in enumerate(states):
        try:
            valid, position = finite_float(read_field(state, "position"))
            if not valid or position is None:
                raise ValueError("payload position is not finite")
            payload_positions.append(position)
        except Exception as exc:
            errors.append(f"[{index}] {short_exception(exc)}")
    result.add(
        "payload positions are finite",
        len(payload_positions) == EXPECTED_COUNT and not errors,
        "10 finite positions" if not errors and len(payload_positions) == 10 else "; ".join(errors),
    )

    exact_hold = (
        len(payload_positions) == len(source_positions) == EXPECTED_COUNT
        and all(a == b for a, b in zip(payload_positions, source_positions))
    )
    result.add(
        "payload is an exact current-position hold",
        exact_hold,
        "all 10 positions exactly equal the read-only source snapshot"
        if exact_hold
        else f"payload={payload_positions!r}; source={list(source_positions)!r}",
    )

    # Re-check limits on payload values, independent of the source checks.
    for index, contract in enumerate(LEFT_O10_CONTRACT):
        name, minimum, maximum = contract
        if index >= len(payload_positions):
            result.add(f"payload limit[{index}] {name}", False, "missing position")
            continue
        position = payload_positions[index]
        result.add(
            f"payload limit[{index}] {name}",
            minimum <= position <= maximum,
            f"position={position:.15g}, range=[{minimum:.15g}, {maximum:.15g}]",
        )


def emit_source_table(snapshot: SourceSnapshot, reporter: Reporter) -> None:
    reporter.section("[LEFT O10 CURRENT STATE + EMBEDDED SDK LIMITS]")
    reporter.emit(f"controlled : {snapshot.controlled!r}")
    reporter.emit(f"type       : {snapshot.end_type!r}")
    reporter.emit(f"names      : {len(snapshot.names)}")
    reporter.emit(f"end_states : {len(snapshot.raw_states)}")
    reporter.emit("index | joint name                              | current(rad)       | min(rad)           | max(rad)")
    reporter.emit("-" * 108)
    for index in range(max(len(snapshot.names), len(snapshot.positions))):
        name = snapshot.names[index] if index < len(snapshot.names) else "<missing-name>"
        position = snapshot.positions[index] if index < len(snapshot.positions) else math.nan
        minimum, maximum = LIMITS.get(name, (math.nan, math.nan))
        reporter.emit(
            f"{index:>5} | {name:<39} | {position:>18.12f} | "
            f"{minimum:>18.12f} | {maximum:>18.12f}"
        )


def emit_payload(payload: Any, reporter: Reporter) -> None:
    reporter.section("[DETACHED LEFT HOLD PAYLOAD - NEVER TRANSMITTED]")
    try:
        group = read_field(payload, "group")
        nums = read_field(payload, "nums")
        states = list(read_field(payload, "states"))
        reporter.emit(f"object type : {type_name(payload)}")
        reporter.emit(f"group       : {group!r}")
        reporter.emit(f"nums        : {nums!r}")
        reporter.emit(f"states      : {len(states)}")
        for index, state in enumerate(states):
            position = read_field(state, "position", None)
            expected_name = EXPECTED_NAMES[index] if index < len(EXPECTED_NAMES) else "<extra>"
            reporter.emit(f"  [{index:02d}] {expected_name}: position={position!r}")
    except Exception as exc:
        reporter.emit(f"payload inspection failed: {short_exception(exc)}")
    reporter.emit("transmission : NEVER")
    reporter.emit("write/control API invocation : NONE")


def emit_checks(result: ValidationResult, reporter: Reporter) -> None:
    reporter.section("[VALIDATION CHECKS]")
    for check in result.checks:
        reporter.emit(f"{'PASS' if check.passed else 'FAIL':4} | {check.name} | {check.detail}")
    if result.warnings:
        reporter.emit("-" * 108)
        for warning in result.warnings:
            reporter.emit(f"WARN | {warning}")


def run_hold_validation(
    gdk_module: Any,
    robot: Any,
    retries: int,
    interval_seconds: float,
    reporter: Reporter,
) -> ValidationResult:
    result = ValidationResult()
    end_state, attempts, last_error = get_end_state_with_retry(
        robot=robot,
        retries=retries,
        interval_seconds=interval_seconds,
        reporter=reporter,
    )
    result.attempts_used = attempts
    result.last_read_error = last_error
    result.add(
        "get_end_state read",
        end_state is not None,
        f"PASS on attempt {attempts}" if end_state is not None else f"failed after {attempts}: {last_error}",
    )
    if end_state is None:
        return result

    snapshot = extract_left_snapshot(end_state, result)
    if snapshot is None:
        return result
    emit_source_table(snapshot, reporter)
    validate_document_limits(snapshot, result)

    prerequisite_ok = (
        tuple(snapshot.names) == EXPECTED_NAMES
        and len(snapshot.positions) == EXPECTED_COUNT
        and snapshot.controlled is True
        and all(math.isfinite(position) for position in snapshot.positions)
    )
    if not prerequisite_ok:
        result.add(
            "payload construction gate",
            False,
            "source structure/order/control/position prerequisite failed; no payload built",
        )
        return result

    try:
        payload = build_detached_hold_payload(gdk_module, tuple(snapshot.positions))
        result.payload = payload
        result.add(
            "detached JointStates construction",
            True,
            "constructed locally; never passed to Robot",
        )
    except Exception as exc:
        result.add("detached JointStates construction", False, short_exception(exc))
        return result

    validate_payload(payload, tuple(snapshot.positions), result)
    emit_payload(payload, reporter)
    return result


class FakeJointState:
    def __init__(self) -> None:
        self.position = 0.0


class FakeJointStates:
    def __init__(self) -> None:
        self.group = ""
        self.states: List[FakeJointState] = []
        self.nums = 0
        self.target_type = ""
        self.timestamp = 0


class FakeGDK:
    JointState = FakeJointState
    JointStates = FakeJointStates


class FakeRobot:
    def __init__(self, end_state: Mapping[str, Any], failures_before_success: int = 1) -> None:
        self.end_state = end_state
        self.failures_before_success = failures_before_success
        self.calls = 0

    def get_end_state(self) -> Mapping[str, Any]:
        self.calls += 1
        if self.calls <= self.failures_before_success:
            raise RuntimeError("simulated initial DDS state delay")
        return copy.deepcopy(self.end_state)


def make_nominal_fake_end_state() -> Dict[str, Any]:
    positions = [
        -0.41865722656250004,
        0.3403173828125,
        -0.6590505453402951,
        0.0,
        1.4762954914964332,
        1.4768317618717264,
        0.0,
        1.4768317618717264,
        -0.009462890624999998,
        1.4762954914964332,
    ]
    states = [
        {
            "id": index,
            "enable": True,
            "position": position,
            "velocity": 0.0,
            "effort": 0.0,
            "err_code": 0,
        }
        for index, position in enumerate(positions)
    ]
    return {
        "left_end_state": {
            "controlled": True,
            "type": 1,
            "names": list(EXPECTED_NAMES),
            "end_states": states,
        },
        "right_end_state": {
            "controlled": True,
            "type": 1,
            "names": [],
            "end_states": [],
        },
    }


def validator_rejects(end_state: Mapping[str, Any]) -> bool:
    result = ValidationResult()
    snapshot = extract_left_snapshot(end_state, result)
    if snapshot is not None:
        validate_document_limits(snapshot, result)
    return any(not check.passed for check in result.checks)


def run_self_test(reporter: Reporter) -> bool:
    reporter.section("[SIMULATED GDK SELF-TEST - NO REAL GDK IMPORT]")
    nominal = make_nominal_fake_end_state()
    fake_robot = FakeRobot(nominal, failures_before_success=1)
    nominal_result = run_hold_validation(
        FakeGDK,
        fake_robot,
        retries=10,
        interval_seconds=0.0,
        reporter=reporter,
    )
    tests: List[Tuple[str, bool]] = [("nominal hold payload passes", nominal_result.passed)]

    wrong_order = copy.deepcopy(nominal)
    wrong_order["left_end_state"]["names"][3], wrong_order["left_end_state"]["names"][4] = (
        wrong_order["left_end_state"]["names"][4],
        wrong_order["left_end_state"]["names"][3],
    )
    tests.append(("wrong order rejected", validator_rejects(wrong_order)))

    whitespace = copy.deepcopy(nominal)
    whitespace["left_end_state"]["names"][4] += " "
    tests.append(("trailing whitespace rejected", validator_rejects(whitespace)))

    wrong_count = copy.deepcopy(nominal)
    wrong_count["left_end_state"]["names"].pop()
    wrong_count["left_end_state"]["end_states"].pop()
    tests.append(("nine-joint source rejected", validator_rejects(wrong_count)))

    right_mix = copy.deepcopy(nominal)
    right_mix["left_end_state"]["names"][4] = "idx77_hand_r_index_pip_joint"
    tests.append(("right-hand mix rejected", validator_rejects(right_mix)))

    out_of_range = copy.deepcopy(nominal)
    out_of_range["left_end_state"]["end_states"][4]["position"] = 2.0
    tests.append(("out-of-range position rejected", validator_rejects(out_of_range)))

    bad_payload = build_detached_hold_payload(FakeGDK, nominal_result.snapshot.positions)
    bad_payload.group = "right_tool"
    payload_result = ValidationResult()
    validate_payload(bad_payload, nominal_result.snapshot.positions, payload_result)
    tests.append(("wrong payload group rejected", not payload_result.passed))

    reporter.section("[SELF-TEST MATRIX]")
    for name, passed in tests:
        reporter.emit(f"{'PASS' if passed else 'FAIL':4} | {name}")
    overall = all(passed for _, passed in tests)
    reporter.emit(f"self-test overall : {'PASS' if overall else 'FAIL'}")
    reporter.emit("real GDK imported : NO")
    reporter.emit("motion/control API invoked : NO")
    return overall


def make_report_path(report_dir: Optional[str]) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    directory = Path(report_dir).expanduser().resolve() if report_dir else Path(__file__).resolve().parent
    return directory / f"109_g2_o10_hold_payload_validation_v4_1_report_{timestamp}.txt"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read the left O10 current state and validate a detached hold-position "
            "JointStates payload. No write/control API is called."
        )
    )
    parser.add_argument("--retries", type=int, default=DEFAULT_RETRIES)
    parser.add_argument(
        "--retry-interval",
        type=float,
        default=DEFAULT_RETRY_INTERVAL_SECONDS,
        help="Seconds between get_end_state() attempts (default: 0.5).",
    )
    parser.add_argument(
        "--gdk-env",
        default=str(DEFAULT_GDK_ENV),
        help="Official GDK env.sh used for one automatic restart if import fails.",
    )
    parser.add_argument(
        "--skip-env-bootstrap",
        action="store_true",
        help="Do not restart through the official GDK environment on import failure.",
    )
    parser.add_argument("--report-dir", default=None)
    parser.add_argument(
        "--no-report",
        action="store_true",
        help="Print results only; do not write a timestamped text report.",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run mock-only tests; do not import or initialize the real GDK.",
    )
    args = parser.parse_args(argv)
    if args.retries < 1 or args.retries > 100:
        parser.error("--retries must be in [1, 100]")
    if args.retry_interval < 0 or args.retry_interval > 60:
        parser.error("--retry-interval must be in [0, 60]")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    reporter = Reporter()
    script_path = Path(__file__).resolve()

    reporter.section(SCRIPT_NAME)
    reporter.emit(f"script    : {script_path}")
    reporter.emit(f"timestamp : {datetime.now().isoformat(timespec='seconds')}")
    reporter.emit(f"python    : {sys.version.splitlines()[0]}")
    reporter.emit("mode      : SELF-TEST" if args.self_test else "mode      : LIVE READ-ONLY")
    reporter.emit(f"SDK truth : {SDK_DOCUMENT_NAME}")
    reporter.emit(f"SDK SHA256: {SDK_DOCUMENT_SHA256}")

    reporter.section("[SOURCE SAFETY AUDIT]")
    audit_ok, violations, source_sha = audit_source(script_path)
    reporter.emit(f"status : {'PASS' if audit_ok else 'FAIL'}")
    reporter.emit(f"source SHA256 : {source_sha or '<unavailable>'}")
    if violations:
        for violation in violations:
            reporter.emit(f" - {violation}")
    else:
        reporter.emit("No direct or literal-getattr motion/control/write API call found in the AST.")
        reporter.emit("Robot call allowlist: get_end_state() only.")
    if not audit_ok:
        reporter.emit("ABORTED before GDK import/initialization.")
        return 2

    if args.self_test:
        passed = run_self_test(reporter)
        reporter.section("109_v4.1 HOLD PAYLOAD VALIDATION SUMMARY")
        reporter.emit(f"source safety audit : {'PASS' if audit_ok else 'FAIL'}")
        reporter.emit(f"simulated validation tests : {'PASS' if passed else 'FAIL'}")
        reporter.emit("motion/control API invoked : NO")
        reporter.emit("O10 commanded : NO")
        reporter.emit("Arm commanded : NO")
        reporter.emit("Motion executed : NO")
        if not args.no_report:
            path = make_report_path(args.report_dir)
            path.parent.mkdir(parents=True, exist_ok=True)
            reporter.emit(f"Report path: {path}")
            reporter.write(path)
        return 0 if passed else 1

    gdk_module: Any = None
    robot: Any = None
    initialized = False
    released = False
    result = ValidationResult()
    runtime_errors: List[str] = []

    try:
        gdk_module = import_gdk_or_restart(args, reporter)
        reporter.section("[LIVE READ-ONLY GDK SESSION]")
        reporter.emit(f"module file : {getattr(gdk_module, '__file__', '<unknown>')}")

        init_result = gdk_module.gdk_init()
        initialized = result_is_success(gdk_module, init_result)
        reporter.emit(f"GDK initialized : {'PASS' if initialized else 'FAIL'} ({init_result!r})")
        if not initialized:
            raise RuntimeError(f"gdk_init() returned {init_result!r}")

        robot = gdk_module.Robot()
        reporter.emit(f"Robot created : PASS ({type_name(robot)})")
        result = run_hold_validation(
            gdk_module=gdk_module,
            robot=robot,
            retries=args.retries,
            interval_seconds=args.retry_interval,
            reporter=reporter,
        )
    except Exception as exc:
        runtime_errors.append(short_exception(exc))
        reporter.emit(f"runtime error : {runtime_errors[-1]}")
    finally:
        if initialized and gdk_module is not None:
            try:
                release_result = gdk_module.gdk_release()
                released = result_is_success(gdk_module, release_result)
                reporter.emit(f"GDK released : {'PASS' if released else 'FAIL'} ({release_result!r})")
            except Exception as exc:
                runtime_errors.append(f"gdk_release: {short_exception(exc)}")
                reporter.emit(f"GDK released : FAIL ({runtime_errors[-1]})")

    if result.checks:
        emit_checks(result, reporter)

    payload_ready = (
        audit_ok
        and initialized
        and released
        and not runtime_errors
        and result.payload is not None
        and result.passed
    )

    reporter.section("109_v4.1 HOLD PAYLOAD VALIDATION SUMMARY")
    reporter.emit(f"source safety audit : {'PASS' if audit_ok else 'FAIL'}")
    reporter.emit(f"GDK initialized : {'PASS' if initialized else 'FAIL'}")
    reporter.emit(f"Robot created : {'PASS' if robot is not None else 'FAIL'}")
    reporter.emit(
        "get_end_state read : "
        + ("PASS" if result.snapshot is not None else "FAIL")
        + f" (attempts={result.attempts_used})"
    )
    reporter.emit(f"GDK released : {'PASS' if released else 'FAIL'}")
    reporter.emit(f"detached hold payload built : {'PASS' if result.payload is not None else 'FAIL'}")
    reporter.emit(f"all payload/contract checks : {'PASS' if result.passed else 'FAIL'}")
    reporter.emit("motion/control API invoked : NO")
    reporter.emit("O10 commanded : NO")
    reporter.emit("Arm commanded : NO")
    reporter.emit("Motion executed : NO")
    reporter.emit("")
    if payload_ready:
        reporter.emit("Final verdict: PASS - LEFT O10 hold payload is structurally ready for later review.")
        reporter.emit("Boundary: payload remained detached and was NEVER transmitted.")
    else:
        reporter.emit("Final verdict: BLOCKED - do not proceed to 110 micro-motion.")
        if runtime_errors:
            for error in runtime_errors:
                reporter.emit(f" - {error}")
        for check in result.checks:
            if not check.passed:
                reporter.emit(f" - {check.name}: {check.detail}")
    reporter.emit("")
    reporter.emit("This script does not authorize or perform 110 motion.")

    if not args.no_report:
        try:
            report_path = make_report_path(args.report_dir)
            report_path.parent.mkdir(parents=True, exist_ok=True)
            reporter.emit(f"Report path: {report_path}")
            reporter.write(report_path)
        except Exception as exc:
            reporter.emit(f"Report write failed: {short_exception(exc)}")
            payload_ready = False

    return 0 if payload_ready else 1


if __name__ == "__main__":
    raise SystemExit(main())
