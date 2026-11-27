#!/usr/bin/env python3
"""
110 v1.3 - G2 LEFT O10 index-PIP first micro-motion (GDK 4.1.5).

Intent
------
Issue one complete 10-position move_ee_pos(JointStates) command for the LEFT
O10_T2 hand. Only idx37_hand_l_index_pip_joint changes, by -0.020 rad
(approximately -1.15 degrees, toward the documented open position). The other
nine targets are copied from a fresh live state immediately before execution.

Default behavior is DRY-RUN. Real transmission requires BOTH:
  --execute --confirm EXECUTE_110_LEFT_INDEX_PIP

No automatic return command is sent. This program transmits at most one motion
command per process. It blocks execution if state, order, limits, health,
physical stillness, the normal mode=1 SERVO status contract, payload shape,
confirmation, or the fresh-state recheck fails. If this Python binding exposes
control_mode it must be the documented value 1; if absent it is recorded as
NOT_EXPOSED_BY_BINDING and does not independently block. The source audit also
proves that this process contains no joint_servo_control transmission flow.

Document truth source
---------------------
GDK_SDK_4.1.5.md
SHA256: e952f9d0c383c468b7384c46274b12d3059827c2c95ecb0506082a48c49df90d
Python Robot.move_ee_pos() contract:
  * non-blocking and must not be used concurrently with servo control
  * group is left_tool, right_tool, or dual_tool
  * nums == len(states)
  * O10_T2 single-hand payload contains exactly 10 ordered positions
  * return value 0 means success; failures may raise RuntimeError

Important
---------
This is motion-capable software. Review the printed payload, keep the hand clear,
keep an emergency stop available, and run DRY-RUN first. A successful API return
means the non-blocking command was accepted; post-command state polling is used
to determine whether the target was observed.
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
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple


PROGRAM_NAME = "110 G2 LEFT O10 INDEX-PIP MICRO-OPEN v1.3"
SDK_DOCUMENT_NAME = "GDK_SDK_4.1.5.md"
SDK_DOCUMENT_SHA256 = "e952f9d0c383c468b7384c46274b12d3059827c2c95ecb0506082a48c49df90d"

EXPECTED_GROUP = "left_tool"
EXPECTED_COUNT = 10
TARGET_INDEX = 4
TARGET_JOINT = "idx37_hand_l_index_pip_joint"
FIXED_DELTA_RAD = -0.020
FIXED_DELTA_DEG = FIXED_DELTA_RAD * 180.0 / math.pi
EXECUTION_TOKEN = "EXECUTE_110_LEFT_INDEX_PIP"

DEFAULT_RETRIES = 10
DEFAULT_RETRY_INTERVAL_SECONDS = 0.5
DEFAULT_COUNTDOWN_SECONDS = 5
DEFAULT_VERIFY_TIMEOUT_SECONDS = 4.0
VERIFY_POLL_SECONDS = 0.10
TARGET_TOLERANCE_RAD = 0.010
OTHER_JOINT_TOLERANCE_RAD = 0.030
FRESHNESS_TOLERANCE_RAD = 0.003
MAX_STILL_VELOCITY_RAD_S = 0.050
MAX_FRAME_LINEAR_VELOCITY_M_S = 0.020
MAX_FRAME_ANGULAR_VELOCITY_RAD_S = 0.050
MIN_TARGET_LIMIT_MARGIN_RAD = 0.010
MOTION_STATUS_SAMPLE_COUNT = 3
MOTION_STATUS_SAMPLE_INTERVAL_SECONDS = 0.10
MOTION_MODE_MAP = {0: "STOP", 1: "SERVO", 2: "PLANNING"}
CONTROL_MODE_MAP = {
    1: "JOINT_POSITION_PD_TRAJECTORY",
    2: "CARTESIAN_IMPEDANCE",
    3: "JOINT_IMPEDANCE",
}
CONTROL_MODE_NOT_EXPOSED = "NOT_EXPOSED_BY_BINDING"
DEFAULT_GDK_ENV = Path.home() / ".cache/agibot/app/env.sh"
BOOTSTRAP_MARKER = "AGIBOT_GDK_ENV_RESTARTED_FOR_110_V1_3"

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
EXPECTED_NAMES: Tuple[str, ...] = tuple(row[0] for row in LEFT_O10_CONTRACT)
LIMITS: Dict[str, Tuple[float, float]] = {
    name: (minimum, maximum) for name, minimum, maximum in LEFT_O10_CONTRACT
}

READ_ONLY_ROBOT_METHODS = frozenset({"get_end_state", "get_motion_control_status"})
CONTROL_METHODS = frozenset(
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
NO_DEFAULT = object()
MISSING = object()


@dataclass
class Check:
    name: str
    passed: bool
    detail: str


@dataclass
class Snapshot:
    names: List[str]
    positions: List[float]
    velocities: List[float]
    controlled: bool
    end_type: Any
    raw_states: List[Any]


@dataclass
class GateResult:
    checks: List[Check] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    def add(self, name: str, passed: bool, detail: str) -> None:
        self.checks.append(Check(name, bool(passed), detail))

    @property
    def passed(self) -> bool:
        return bool(self.checks) and all(item.passed for item in self.checks)


@dataclass
class CommandOutcome:
    requested: bool = False
    transmitted: bool = False
    api_return: Any = None
    api_accepted: bool = False
    target_observed: bool = False
    observed_positions: Optional[List[float]] = None
    verification_error: Optional[str] = None


class Reporter:
    def __init__(self) -> None:
        self.lines: List[str] = []

    def emit(self, text: str = "") -> None:
        value = str(text)
        print(value, flush=True)
        self.lines.append(value)

    def section(self, title: str) -> None:
        self.emit("=" * 112)
        self.emit(title)
        self.emit("=" * 112)

    def write(self, path: Path) -> None:
        path.write_text("\n".join(self.lines) + "\n", encoding="utf-8")


class SourceAudit(ast.NodeVisitor):
    """Require one explicit move call and reject every other control call path."""

    def __init__(self) -> None:
        self.control_calls: List[Tuple[int, str]] = []
        self.robot_calls: List[Tuple[int, str]] = []
        self.literal_getattr_controls: List[Tuple[int, str]] = []
        self.dynamic_robot_dispatch: List[int] = []

    @staticmethod
    def call_name(func: ast.AST) -> Optional[str]:
        if isinstance(func, ast.Name):
            return func.id
        if isinstance(func, ast.Attribute):
            return func.attr
        return None

    def visit_Call(self, node: ast.Call) -> Any:
        name = self.call_name(node.func)
        if name in CONTROL_METHODS:
            self.control_calls.append((node.lineno, str(name)))
        if isinstance(node.func, ast.Attribute):
            base = node.func.value
            if isinstance(base, ast.Name) and base.id == "robot":
                self.robot_calls.append((node.lineno, node.func.attr))
        if isinstance(node.func, ast.Name) and node.func.id == "getattr":
            if len(node.args) >= 2:
                owner = node.args[0]
                literal = node.args[1]
                if isinstance(literal, ast.Constant) and isinstance(literal.value, str):
                    if literal.value in CONTROL_METHODS:
                        self.literal_getattr_controls.append((node.lineno, literal.value))
                elif isinstance(owner, ast.Name) and owner.id == "robot":
                    self.dynamic_robot_dispatch.append(node.lineno)
        self.generic_visit(node)


def audit_source(path: Path) -> Tuple[bool, List[str], str]:
    try:
        data = path.read_bytes()
        source = data.decode("utf-8")
        tree = ast.parse(source, filename=str(path))
    except Exception as exc:
        return False, [f"source parse failed: {type(exc).__name__}: {exc}"], ""

    visitor = SourceAudit()
    visitor.visit(tree)
    errors: List[str] = []
    expected_control = [(line, name) for line, name in visitor.control_calls if name == "move_ee_pos"]
    unexpected_control = [(line, name) for line, name in visitor.control_calls if name != "move_ee_pos"]
    if len(expected_control) != 1:
        errors.append(
            f"expected exactly one direct move_ee_pos call, found {len(expected_control)}: "
            f"{expected_control}"
        )
    if unexpected_control:
        errors.append(f"unexpected control calls: {unexpected_control}")
    if visitor.literal_getattr_controls:
        errors.append(f"literal getattr control paths: {visitor.literal_getattr_controls}")
    if visitor.dynamic_robot_dispatch:
        errors.append(
            f"dynamic getattr(robot, ...) paths are forbidden: {visitor.dynamic_robot_dispatch}"
        )

    allowed_robot_calls = READ_ONLY_ROBOT_METHODS | {"move_ee_pos"}
    invalid_robot_calls = [item for item in visitor.robot_calls if item[1] not in allowed_robot_calls]
    if invalid_robot_calls:
        errors.append(f"Robot calls outside allowlist: {invalid_robot_calls}")
    if expected_control and visitor.robot_calls.count(expected_control[0]) != 1:
        errors.append("the move_ee_pos call is not an explicit robot.move_ee_pos call")

    digest = hashlib.sha256(data).hexdigest()
    return not errors, errors, digest


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


def short_exception(exc: BaseException) -> str:
    return f"{type(exc).__name__}: {exc}"


def type_name(value: Any) -> str:
    cls = type(value)
    return f"{cls.__module__}.{cls.__name__}"


def finite_float(value: Any) -> Tuple[bool, Optional[float]]:
    if isinstance(value, bool):
        return False, None
    try:
        converted = float(value)
    except (TypeError, ValueError, OverflowError):
        return False, None
    return math.isfinite(converted), converted


def finite_vector(value: Any, label: str, expected_count: int = 3) -> List[float]:
    """Convert one documented Twist vector to finite floats."""
    if isinstance(value, (str, bytes, bytearray)):
        raise TypeError(f"{label} is text/bytes, not a numeric vector")
    converted = list(value)
    if len(converted) != expected_count:
        raise ValueError(
            f"{label} count={len(converted)}, expected={expected_count}"
        )
    result: List[float] = []
    for index, item in enumerate(converted):
        ok, numeric = finite_float(item)
        if not ok or numeric is None:
            raise ValueError(f"{label}[{index}] is not finite: {item!r}")
        result.append(numeric)
    return result


def extract_twist_components(status: Any) -> Tuple[List[str], List[Tuple[List[float], List[float]]]]:
    """Read documented frame_names + Twist.linear/angular vectors."""
    frame_names = list(read_field(status, "frame_names"))
    twists = list(read_field(status, "twists"))
    if not frame_names:
        raise ValueError("frame_names is empty; robot physical stillness is unverified")
    if len(twists) != len(frame_names):
        raise ValueError(
            f"twists count={len(twists)} does not match frame_names count={len(frame_names)}"
        )
    if not all(isinstance(name, str) and name for name in frame_names):
        raise ValueError(f"invalid frame_names: {frame_names!r}")

    components: List[Tuple[List[float], List[float]]] = []
    for index, twist in enumerate(twists):
        linear = finite_vector(read_field(twist, "linear"), f"twists[{index}].linear")
        angular = finite_vector(read_field(twist, "angular"), f"twists[{index}].angular")
        components.append((linear, angular))
    return frame_names, components


def result_is_success(gdk_module: Any, value: Any) -> bool:
    try:
        if value == gdk_module.GDKRes.kSuccess:
            return True
    except Exception:
        pass
    try:
        return int(value) == 0
    except Exception:
        return False


def import_gdk_or_restart(args: argparse.Namespace, reporter: Reporter) -> Any:
    try:
        return importlib.import_module("agibot_gdk")
    except Exception as first_exc:
        if args.skip_env_bootstrap or os.environ.get(BOOTSTRAP_MARKER) == "1":
            raise RuntimeError(
                "Unable to import agibot_gdk: " + short_exception(first_exc)
            ) from first_exc
        env_script = Path(args.gdk_env).expanduser().resolve()
        if not env_script.is_file():
            raise RuntimeError(
                f"agibot_gdk import failed and GDK env script was not found: {env_script}; "
                f"original error: {short_exception(first_exc)}"
            ) from first_exc
        reporter.emit("[ENV] Restarting once with the official GDK environment ...")
        # env.sh may interpret its first positional argument as the GDK app root.
        # Source it inside a no-argument shell function, while preserving the
        # Python restart command in an array. This prevents env.sh itself from
        # being received as $1 and incorrectly becoming .../env.sh/lib.
        command = (
            '_g2_env_script="$1"; shift; '
            '_g2_restart_argv=("$@"); '
            '_g2_source_env_without_args() { source "$_g2_env_script"; }; '
            '_g2_source_env_without_args && '
            f'export {BOOTSTRAP_MARKER}=1 && '
            'exec "${_g2_restart_argv[@]}"'
        )
        argv = [
            "/bin/bash",
            "-lc",
            command,
            "110-bootstrap",
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
    interval: float,
    reporter: Optional[Reporter] = None,
    label: str = "read",
) -> Tuple[Any, int]:
    last_error: Optional[str] = None
    for attempt in range(1, retries + 1):
        try:
            value = robot.get_end_state()
            left = read_field(value, "left_end_state")
            names = list(read_field(left, "names"))
            states = list(read_field(left, "end_states"))
            if not names or not states:
                raise RuntimeError(
                    f"left state not populated (names={len(names)}, states={len(states)})"
                )
            if reporter:
                reporter.emit(
                    f"{label} attempt {attempt}/{retries}: PASS "
                    f"({len(names)} names, {len(states)} states)"
                )
            return value, attempt
        except Exception as exc:
            last_error = short_exception(exc)
            if reporter:
                reporter.emit(f"{label} attempt {attempt}/{retries}: no state ({last_error})")
            if attempt < retries and interval > 0:
                time.sleep(interval)
    raise RuntimeError(f"get_end_state failed after {retries} attempts: {last_error}")


def extract_snapshot(end_state: Any, gates: GateResult, prefix: str) -> Optional[Snapshot]:
    try:
        left = read_field(end_state, "left_end_state")
        names = list(read_field(left, "names"))
        states = list(read_field(left, "end_states"))
        controlled = read_field(left, "controlled")
        end_type = read_field(left, "type", None)
    except Exception as exc:
        gates.add(f"{prefix} left state structure", False, short_exception(exc))
        return None

    gates.add(f"{prefix} controlled", controlled is True, f"controlled={controlled!r}")
    gates.add(f"{prefix} name count", len(names) == EXPECTED_COUNT, f"actual={len(names)}")
    gates.add(f"{prefix} state count", len(states) == EXPECTED_COUNT, f"actual={len(states)}")
    gates.add(
        f"{prefix} exact SDK joint order",
        tuple(names) == EXPECTED_NAMES,
        "exact" if tuple(names) == EXPECTED_NAMES else f"actual={names!r}",
    )
    gates.add(
        f"{prefix} no edge whitespace",
        all(isinstance(name, str) and name == name.strip() for name in names),
        "clean" if all(isinstance(name, str) and name == name.strip() for name in names) else repr(names),
    )
    gates.add(
        f"{prefix} no right-hand mix",
        all(isinstance(name, str) and "_hand_r_" not in name for name in names),
        "left-only",
    )
    gates.add(f"{prefix} unique names", len(set(names)) == len(names), f"unique={len(set(names))}")

    positions: List[float] = []
    velocities: List[float] = []
    field_errors: List[str] = []
    health_ok = True
    for index, state in enumerate(states):
        name = names[index] if index < len(names) else f"index_{index}"
        try:
            valid, position = finite_float(read_field(state, "position"))
            if not valid or position is None:
                raise ValueError("position is not finite")
            positions.append(position)
        except Exception as exc:
            field_errors.append(f"{name}.position: {short_exception(exc)}")
        try:
            valid, velocity = finite_float(read_field(state, "velocity"))
            if not valid or velocity is None:
                raise ValueError("velocity is not finite")
            velocities.append(velocity)
        except Exception as exc:
            field_errors.append(f"{name}.velocity: {short_exception(exc)}")

        enable = read_field(state, "enable", MISSING)
        err_code = read_field(state, "err_code", MISSING)
        if enable is MISSING:
            health_ok = False
            field_errors.append(f"{name}.enable missing")
        elif enable is not True:
            health_ok = False
            field_errors.append(f"{name}.enable={enable!r}")
        if err_code is MISSING:
            health_ok = False
            field_errors.append(f"{name}.err_code missing")
        else:
            try:
                if int(err_code) != 0:
                    health_ok = False
                    field_errors.append(f"{name}.err_code={err_code!r}")
            except Exception:
                health_ok = False
                field_errors.append(f"{name}.err_code={err_code!r}")

    gates.add(
        f"{prefix} finite positions/velocities",
        len(positions) == EXPECTED_COUNT and len(velocities) == EXPECTED_COUNT,
        "10 finite positions + 10 finite velocities" if not field_errors else "; ".join(field_errors),
    )
    gates.add(
        f"{prefix} all joints enabled and error-free",
        health_ok,
        "enable=True and err_code=0 for all 10" if health_ok else "; ".join(field_errors),
    )

    still = len(velocities) == EXPECTED_COUNT and all(
        abs(value) <= MAX_STILL_VELOCITY_RAD_S for value in velocities
    )
    gates.add(
        f"{prefix} hand is still",
        still,
        f"max_abs_velocity={max((abs(v) for v in velocities), default=math.inf):.6f} rad/s; "
        f"limit={MAX_STILL_VELOCITY_RAD_S:.6f}",
    )

    limits_ok = len(positions) == EXPECTED_COUNT and tuple(names) == EXPECTED_NAMES
    limit_failures: List[str] = []
    if limits_ok:
        for index, (name, position) in enumerate(zip(names, positions)):
            minimum, maximum = LIMITS[name]
            if not (minimum <= position <= maximum):
                limits_ok = False
                limit_failures.append(
                    f"[{index}] {name}={position:.12f} outside [{minimum:.12f}, {maximum:.12f}]"
                )
    gates.add(
        f"{prefix} current positions within SDK limits",
        limits_ok,
        "all within limits" if limits_ok else "; ".join(limit_failures) or "shape/order prerequisite failed",
    )

    if len(positions) != EXPECTED_COUNT or len(velocities) != EXPECTED_COUNT:
        return None
    return Snapshot(
        names=names,
        positions=positions,
        velocities=velocities,
        controlled=controlled is True,
        end_type=end_type,
        raw_states=states,
    )


def validate_motion_status(robot: Any, gates: GateResult, prefix: str) -> Any:
    """Validate the v1.3 mode=1 idle-SERVO contract, fail-closed.

    GDK 4.1.5 labels mode=1 as SERVO. On this G2, that is the stable normal
    controller state while the robot and O10 are physically still. It is not,
    by itself, evidence that a JointServoControl stream is active. Concurrency
    is instead excluded by this program's AST allow-list: the only control call
    is one explicit move_ee_pos(payload); no joint_servo_control call exists.

    The gate therefore requires stable mode=1, finite and still documented frame
    twists, error_code=0, and empty collision lists. If control_mode is exposed,
    it must be the documented value 1 and remain stable. If the current Python
    binding does not expose it, the report records NOT_EXPOSED_BY_BINDING and no
    value is invented. mode=2, undocumented modes, non-still frames, errors, and
    collisions are blocked. Left O10 physical stillness is validated separately.
    """
    statuses: List[Any] = []
    records: List[
        Tuple[
            Optional[int],
            int,
            int,
            str,
            List[Any],
            List[Any],
            List[str],
            List[Tuple[List[float], List[float]]],
        ]
    ] = []
    for sample_index in range(1, MOTION_STATUS_SAMPLE_COUNT + 1):
        try:
            status = robot.get_motion_control_status()
            mode = int(read_field(status, "mode"))
            control_mode_raw = read_field(status, "control_mode", MISSING)
            control_mode = (
                None if control_mode_raw is MISSING else int(control_mode_raw)
            )
            error_code = int(read_field(status, "error_code"))
            error_msg = str(read_field(status, "error_msg", ""))
            collision_1 = list(read_field(status, "collision_pairs_1", []))
            collision_2 = list(read_field(status, "collision_pairs_2", []))
            frame_names, twist_components = extract_twist_components(status)
        except Exception as exc:
            gates.add(
                f"{prefix} motion status sample {sample_index:02d} readable",
                False,
                short_exception(exc),
            )
            return None
        statuses.append(status)
        records.append(
            (
                mode,
                control_mode,
                error_code,
                error_msg,
                collision_1,
                collision_2,
                frame_names,
                twist_components,
            )
        )
        max_linear = max(
            (abs(value) for linear, _ in twist_components for value in linear),
            default=math.inf,
        )
        max_angular = max(
            (abs(value) for _, angular in twist_components for value in angular),
            default=math.inf,
        )
        gates.add(
            f"{prefix} motion status sample {sample_index:02d} readable",
            True,
            f"type={type_name(status)}, mode={mode}({MOTION_MODE_MAP.get(mode, 'UNDOCUMENTED')}), "
            f"control_mode="
            f"{CONTROL_MODE_NOT_EXPOSED if control_mode is None else control_mode}, "
            f"control_mode_label="
            f"{CONTROL_MODE_NOT_EXPOSED if control_mode is None else CONTROL_MODE_MAP.get(control_mode, 'UNDOCUMENTED')}, "
            f"frames={len(frame_names)}, max_linear={max_linear:.6f} m/s, "
            f"max_angular={max_angular:.6f} rad/s",
        )
        if sample_index < MOTION_STATUS_SAMPLE_COUNT:
            time.sleep(MOTION_STATUS_SAMPLE_INTERVAL_SECONDS)

    modes = [item[0] for item in records]
    control_modes = [item[1] for item in records]
    error_codes = [item[2] for item in records]
    error_messages = [item[3] for item in records]
    collisions_1 = [item[4] for item in records]
    collisions_2 = [item[5] for item in records]
    frame_names_samples = [item[6] for item in records]
    twist_samples = [item[7] for item in records]

    gates.add(
        f"{prefix} documented motion-mode values",
        all(value in MOTION_MODE_MAP for value in modes),
        f"modes={modes!r}, labels={[MOTION_MODE_MAP.get(value, 'UNDOCUMENTED') for value in modes]!r}",
    )
    exposed_control_modes = [value for value in control_modes if value is not None]
    control_mode_exposure_consistent = (
        len(exposed_control_modes) == 0
        or len(exposed_control_modes) == len(control_modes)
    )
    gates.add(
        f"{prefix} control_mode binding exposure consistent",
        control_mode_exposure_consistent,
        f"control_modes="
        f"{[CONTROL_MODE_NOT_EXPOSED if value is None else value for value in control_modes]!r}",
    )
    gates.add(
        f"{prefix} exposed control_mode contract",
        not exposed_control_modes
        or all(value in CONTROL_MODE_MAP for value in exposed_control_modes),
        (
            CONTROL_MODE_NOT_EXPOSED
            if not exposed_control_modes
            else f"values={exposed_control_modes!r}, "
            f"labels={[CONTROL_MODE_MAP.get(value, 'UNDOCUMENTED') for value in exposed_control_modes]!r}"
        ),
    )
    gates.add(
        f"{prefix} motion status stable across samples",
        len(set(modes)) == 1
        and control_mode_exposure_consistent
        and len(set(exposed_control_modes)) <= 1,
        f"modes={modes!r}, control_modes="
        f"{[CONTROL_MODE_NOT_EXPOSED if value is None else value for value in control_modes]!r}",
    )
    gates.add(
        f"{prefix} normal idle-SERVO status",
        all(value == 1 for value in modes),
        f"modes={modes!r}; GDK 4.1.5: 0=STOP, 1=SERVO, 2=PLANNING; "
        "v1.3 requires stable mode=1 while physical-stillness and source-concurrency gates pass",
    )
    gates.add(
        f"{prefix} joint-position control mode",
        not exposed_control_modes or all(value == 1 for value in exposed_control_modes),
        (
            f"{CONTROL_MODE_NOT_EXPOSED}; not independently blocking"
            if not exposed_control_modes
            else f"control_modes={exposed_control_modes!r}; "
            "required 1=JOINT_POSITION_PD_TRAJECTORY"
        ),
    )
    frame_names_stable = all(
        names == frame_names_samples[0] for names in frame_names_samples[1:]
    )
    gates.add(
        f"{prefix} motion-status frame set stable",
        frame_names_stable,
        f"frame_names={frame_names_samples!r}",
    )
    max_linear = max(
        (
            abs(value)
            for sample in twist_samples
            for linear, _ in sample
            for value in linear
        ),
        default=math.inf,
    )
    max_angular = max(
        (
            abs(value)
            for sample in twist_samples
            for _, angular in sample
            for value in angular
        ),
        default=math.inf,
    )
    gates.add(
        f"{prefix} robot frames physically still",
        max_linear <= MAX_FRAME_LINEAR_VELOCITY_M_S
        and max_angular <= MAX_FRAME_ANGULAR_VELOCITY_RAD_S,
        f"max_linear={max_linear:.6f} m/s (limit={MAX_FRAME_LINEAR_VELOCITY_M_S:.6f}); "
        f"max_angular={max_angular:.6f} rad/s "
        f"(limit={MAX_FRAME_ANGULAR_VELOCITY_RAD_S:.6f})",
    )
    gates.add(
        f"{prefix} motion status error-free",
        all(value == 0 for value in error_codes),
        f"error_codes={error_codes!r}, error_messages={error_messages!r}",
    )
    gates.add(
        f"{prefix} no reported collision pairs",
        all(not value for value in collisions_1) and all(not value for value in collisions_2),
        f"collision_pairs_1={collisions_1!r}, collision_pairs_2={collisions_2!r}",
    )
    return statuses[-1]


def build_target_positions(current: Sequence[float]) -> List[float]:
    target = [float(value) for value in current]
    target[TARGET_INDEX] = target[TARGET_INDEX] + FIXED_DELTA_RAD
    return target


def validate_target(
    current: Sequence[float],
    target: Sequence[float],
    gates: GateResult,
    prefix: str,
) -> None:
    gates.add(f"{prefix} current vector count", len(current) == EXPECTED_COUNT, f"actual={len(current)}")
    gates.add(f"{prefix} target vector count", len(target) == EXPECTED_COUNT, f"actual={len(target)}")
    if len(current) != EXPECTED_COUNT or len(target) != EXPECTED_COUNT:
        return

    finite = all(math.isfinite(float(value)) for value in target)
    gates.add(f"{prefix} target values finite", finite, "all finite")

    differences = [float(t) - float(c) for c, t in zip(current, target)]
    changed_indices = [index for index, delta in enumerate(differences) if delta != 0.0]
    gates.add(
        f"{prefix} exactly one changed joint",
        changed_indices == [TARGET_INDEX],
        f"changed_indices={changed_indices}",
    )
    # Subtraction after floating-point addition may produce, for example,
    # -0.020000000000000018. Treat only machine-rounding noise as exact here.
    exact_delta = math.isclose(
        differences[TARGET_INDEX], FIXED_DELTA_RAD, rel_tol=0.0, abs_tol=1e-15
    )
    gates.add(
        f"{prefix} fixed opening delta",
        exact_delta,
        f"actual={differences[TARGET_INDEX]:.15g}, required={FIXED_DELTA_RAD:.15g} rad",
    )
    unchanged = all(
        target[index] == current[index]
        for index in range(EXPECTED_COUNT)
        if index != TARGET_INDEX
    )
    gates.add(f"{prefix} other nine joints unchanged", unchanged, "exact equality")

    limits_ok = True
    failures: List[str] = []
    for index, (name, minimum, maximum) in enumerate(LEFT_O10_CONTRACT):
        value = float(target[index])
        if not minimum <= value <= maximum:
            limits_ok = False
            failures.append(f"{name}={value} outside [{minimum}, {maximum}]")
    gates.add(
        f"{prefix} all target positions within SDK limits",
        limits_ok,
        "all within limits" if limits_ok else "; ".join(failures),
    )

    minimum, maximum = LIMITS[TARGET_JOINT]
    target_value = float(target[TARGET_INDEX])
    margin = min(target_value - minimum, maximum - target_value)
    gates.add(
        f"{prefix} changed joint limit margin",
        margin >= MIN_TARGET_LIMIT_MARGIN_RAD,
        f"target={target_value:.12f}, margin={margin:.12f}, required>={MIN_TARGET_LIMIT_MARGIN_RAD:.12f}",
    )


def build_payload(gdk_module: Any, target: Sequence[float]) -> Any:
    payload = gdk_module.JointStates()
    payload.group = EXPECTED_GROUP
    states = []
    for value in target:
        state = gdk_module.JointState()
        state.position = float(value)
        states.append(state)
    payload.states = states
    payload.nums = len(states)
    return payload


def payload_positions(payload: Any) -> List[float]:
    return [float(read_field(state, "position")) for state in list(read_field(payload, "states"))]


def validate_payload(payload: Any, target: Sequence[float], gates: GateResult, prefix: str) -> None:
    try:
        group = read_field(payload, "group")
        nums = read_field(payload, "nums")
        states = list(read_field(payload, "states"))
        positions = payload_positions(payload)
    except Exception as exc:
        gates.add(f"{prefix} payload structure", False, short_exception(exc))
        return
    gates.add(f"{prefix} payload group", group == EXPECTED_GROUP, f"actual={group!r}")
    gates.add(f"{prefix} payload nums", nums == EXPECTED_COUNT, f"actual={nums!r}")
    gates.add(f"{prefix} payload state count", len(states) == EXPECTED_COUNT, f"actual={len(states)}")
    gates.add(f"{prefix} nums equals states", nums == len(states), f"nums={nums!r}, len={len(states)}")
    gates.add(
        f"{prefix} payload exactly equals target",
        positions == [float(value) for value in target],
        "exact 10-value equality" if positions == [float(value) for value in target] else repr(positions),
    )


def snapshots_fresh(baseline: Snapshot, fresh: Snapshot, gates: GateResult) -> bool:
    names_same = baseline.names == fresh.names == list(EXPECTED_NAMES)
    gates.add("fresh recheck exact joint order", names_same, "exact" if names_same else "changed")
    if len(baseline.positions) != EXPECTED_COUNT or len(fresh.positions) != EXPECTED_COUNT:
        gates.add("fresh recheck drift", False, "invalid vector length")
        return False
    drifts = [abs(a - b) for a, b in zip(baseline.positions, fresh.positions)]
    max_drift = max(drifts)
    stable = max_drift <= FRESHNESS_TOLERANCE_RAD
    gates.add(
        "fresh recheck drift",
        stable,
        f"max_drift={max_drift:.12f} rad, tolerance={FRESHNESS_TOLERANCE_RAD:.12f}",
    )
    return names_same and stable


def emit_snapshot(snapshot: Snapshot, target: Sequence[float], reporter: Reporter, title: str) -> None:
    reporter.section(title)
    reporter.emit(f"controlled : {snapshot.controlled}")
    reporter.emit(f"type       : {snapshot.end_type!r}")
    reporter.emit(f"fixed delta: {FIXED_DELTA_RAD:+.6f} rad ({FIXED_DELTA_DEG:+.3f} deg)")
    reporter.emit("idx | joint name                              | current(rad)   | target(rad)    | delta(rad)")
    reporter.emit("-" * 112)
    for index, name in enumerate(snapshot.names):
        current = snapshot.positions[index]
        target_value = float(target[index])
        reporter.emit(
            f"{index:>3} | {name:<39} | {current:>14.9f} | {target_value:>14.9f} | "
            f"{target_value-current:>+11.6f}"
        )


def emit_checks(gates: GateResult, reporter: Reporter, title: str) -> None:
    reporter.section(title)
    for item in gates.checks:
        reporter.emit(f"{'PASS' if item.passed else 'FAIL':4} | {item.name} | {item.detail}")
    for warning in gates.warnings:
        reporter.emit(f"WARN | {warning}")
    reporter.emit(f"gate verdict : {'PASS' if gates.passed else 'FAIL'}")


def transmit_once(robot: Any, payload: Any) -> Any:
    """The one and only control API call in this source file."""
    return robot.move_ee_pos(payload)


def verify_target_observed(
    robot: Any,
    target: Sequence[float],
    timeout_seconds: float,
    reporter: Reporter,
) -> Tuple[bool, Optional[List[float]], Optional[str]]:
    deadline = time.monotonic() + timeout_seconds
    last_positions: Optional[List[float]] = None
    last_error: Optional[str] = None
    poll = 0
    while time.monotonic() <= deadline:
        poll += 1
        try:
            value, _ = get_end_state_with_retry(robot, retries=1, interval=0.0)
            local_gates = GateResult()
            snapshot = extract_snapshot(value, local_gates, f"post[{poll}]")
            if snapshot is None:
                raise RuntimeError("post-command snapshot structure invalid")
            last_positions = snapshot.positions
            target_error = abs(last_positions[TARGET_INDEX] - float(target[TARGET_INDEX]))
            other_error = max(
                abs(last_positions[index] - float(target[index]))
                for index in range(EXPECTED_COUNT)
                if index != TARGET_INDEX
            )
            reporter.emit(
                f"post poll {poll}: target_error={target_error:.6f} rad, "
                f"other_max_error={other_error:.6f} rad"
            )
            if (
                target_error <= TARGET_TOLERANCE_RAD
                and other_error <= OTHER_JOINT_TOLERANCE_RAD
            ):
                return True, last_positions, None
        except Exception as exc:
            last_error = short_exception(exc)
            reporter.emit(f"post poll {poll}: read error ({last_error})")
        time.sleep(VERIFY_POLL_SECONDS)
    return False, last_positions, last_error or "target not observed before timeout"


def countdown(seconds: int, reporter: Reporter) -> None:
    reporter.section("[FINAL COUNTDOWN - CTRL+C TO ABORT]")
    for remaining in range(seconds, 0, -1):
        reporter.emit(f"transmission in {remaining} ...")
        time.sleep(1.0)


def make_report_path(report_dir: Optional[str]) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    directory = Path(report_dir).expanduser().resolve() if report_dir else Path(__file__).resolve().parent
    return directory / f"110_g2_left_o10_index_pip_micro_open_v1_3_report_{timestamp}.txt"


class FakeJointState:
    def __init__(self) -> None:
        self.position = 0.0


class FakeJointStates:
    def __init__(self) -> None:
        self.group = ""
        self.states: List[FakeJointState] = []
        self.nums = 0


class FakeGDKRes:
    kSuccess = 0


class FakeGDK:
    JointState = FakeJointState
    JointStates = FakeJointStates
    GDKRes = FakeGDKRes


class FakeTwist:
    def __init__(
        self,
        linear: Optional[Sequence[float]] = None,
        angular: Optional[Sequence[float]] = None,
    ) -> None:
        self.linear = list(linear if linear is not None else (0.0, 0.0, 0.0))
        self.angular = list(angular if angular is not None else (0.0, 0.0, 0.0))


class FakeMotionStatus:
    def __init__(
        self,
        mode: int = 1,
        control_mode: int = 1,
        error_code: int = 0,
    ) -> None:
        self.mode = mode
        self.control_mode = control_mode
        self.error_code = error_code
        self.error_msg = ""
        self.frame_names = ["arm_l_end_link", "arm_r_end_link"]
        self.twists = [FakeTwist(), FakeTwist()]
        self.collision_pairs_1: List[str] = []
        self.collision_pairs_2: List[str] = []


class FakeRobot:
    def __init__(
        self,
        end_state: Mapping[str, Any],
        motion_mode: int = 1,
        control_mode: int = 1,
    ) -> None:
        self.state = copy.deepcopy(end_state)
        self.motion_status = FakeMotionStatus(mode=motion_mode, control_mode=control_mode)
        self.transmit_count = 0
        self.last_payload: Any = None
        self.initial_read_failure = True

    def get_end_state(self) -> Mapping[str, Any]:
        if self.initial_read_failure:
            self.initial_read_failure = False
            raise RuntimeError("simulated initial DDS delay")
        return copy.deepcopy(self.state)

    def get_motion_control_status(self) -> FakeMotionStatus:
        return copy.deepcopy(self.motion_status)

    def move_ee_pos(self, payload: Any) -> int:
        self.transmit_count += 1
        if self.transmit_count > 1:
            raise AssertionError("more than one command attempted")
        self.last_payload = copy.deepcopy(payload)
        values = payload_positions(payload)
        for index, value in enumerate(values):
            self.state["left_end_state"]["end_states"][index]["position"] = value
            self.state["left_end_state"]["end_states"][index]["velocity"] = 0.0
        return 0


def nominal_fake_state() -> Dict[str, Any]:
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


def validate_fake_state(end_state: Mapping[str, Any]) -> bool:
    gates = GateResult()
    snapshot = extract_snapshot(end_state, gates, "test")
    if snapshot is not None:
        target = build_target_positions(snapshot.positions)
        validate_target(snapshot.positions, target, gates, "test")
    return gates.passed


def run_self_test(reporter: Reporter) -> bool:
    reporter.section("[SELF-TEST: SIMULATED SINGLE MICRO-MOTION]")
    source = nominal_fake_state()
    robot = FakeRobot(source)
    gates = GateResult()
    try:
        end_state, attempts = get_end_state_with_retry(robot, 10, 0.0, reporter, "self-test read")
        snapshot = extract_snapshot(end_state, gates, "self-test")
        validate_motion_status(robot, gates, "self-test")
        if snapshot is None:
            raise RuntimeError("nominal snapshot extraction failed")
        target = build_target_positions(snapshot.positions)
        validate_target(snapshot.positions, target, gates, "self-test")
        payload = build_payload(FakeGDK, target)
        validate_payload(payload, target, gates, "self-test")
        result = transmit_once(robot, payload)
        accepted = result_is_success(FakeGDK, result)
        observed, _, error = verify_target_observed(robot, target, 0.25, reporter)
        nominal_pass = gates.passed and accepted and observed and robot.transmit_count == 1
    except Exception as exc:
        nominal_pass = False
        error = short_exception(exc)

    tests: List[Tuple[str, bool, str]] = [
        (
            "nominal stable mode=1 SERVO command passes",
            nominal_pass,
            error or "mode=1, control_mode=1, physically still, one command, target observed",
        ),
        ("exactly one fake transmission", robot.transmit_count == 1, f"count={robot.transmit_count}"),
        (
            "no joint_servo_control flow exists",
            not hasattr(robot, "joint_servo_control"),
            "Fake path and AST allow-list contain no servo-stream transmitter",
        ),
    ]

    wrong_order = nominal_fake_state()
    wrong_order["left_end_state"]["names"][3], wrong_order["left_end_state"]["names"][4] = (
        wrong_order["left_end_state"]["names"][4],
        wrong_order["left_end_state"]["names"][3],
    )
    tests.append(("wrong order rejected", not validate_fake_state(wrong_order), ""))

    moving = nominal_fake_state()
    moving["left_end_state"]["end_states"][TARGET_INDEX]["velocity"] = 0.2
    tests.append(("moving hand rejected", not validate_fake_state(moving), ""))

    unhealthy = nominal_fake_state()
    unhealthy["left_end_state"]["end_states"][TARGET_INDEX]["err_code"] = 7
    tests.append(("joint error rejected", not validate_fake_state(unhealthy), ""))

    missing_health = nominal_fake_state()
    missing_health["left_end_state"]["end_states"][TARGET_INDEX].pop("enable")
    tests.append(("missing health field rejected", not validate_fake_state(missing_health), ""))

    out_of_range = nominal_fake_state()
    out_of_range["left_end_state"]["end_states"][TARGET_INDEX]["position"] = 2.0
    tests.append(("out-of-range source rejected", not validate_fake_state(out_of_range), ""))

    status_robot = FakeRobot(nominal_fake_state(), motion_mode=1)
    status_gates = GateResult()
    validate_motion_status(status_robot, status_gates, "test")
    tests.append(("stable documented SERVO mode accepted", status_gates.passed, "mode=1"))

    no_control_mode_robot = FakeRobot(nominal_fake_state(), motion_mode=1)
    delattr(no_control_mode_robot.motion_status, "control_mode")
    no_control_mode_gates = GateResult()
    validate_motion_status(no_control_mode_robot, no_control_mode_gates, "test")
    tests.append(
        (
            "control_mode absent in binding accepted",
            no_control_mode_gates.passed,
            CONTROL_MODE_NOT_EXPOSED,
        )
    )

    stop_robot = FakeRobot(nominal_fake_state(), motion_mode=0)
    stop_gates = GateResult()
    validate_motion_status(stop_robot, stop_gates, "test")
    tests.append(("STOP mode rejected by v1.3 normal-state contract", not stop_gates.passed, "mode=0"))

    planning_robot = FakeRobot(nominal_fake_state(), motion_mode=2)
    planning_gates = GateResult()
    validate_motion_status(planning_robot, planning_gates, "test")
    tests.append(("documented PLANNING mode rejected for MoveEEPos", not planning_gates.passed, "mode=2"))

    unknown_robot = FakeRobot(nominal_fake_state(), motion_mode=9)
    unknown_gates = GateResult()
    validate_motion_status(unknown_robot, unknown_gates, "test")
    tests.append(("undocumented motion mode rejected", not unknown_gates.passed, "mode=9"))

    impedance_robot = FakeRobot(nominal_fake_state(), motion_mode=1, control_mode=2)
    impedance_gates = GateResult()
    validate_motion_status(impedance_robot, impedance_gates, "test")
    tests.append(("non-joint-position control mode rejected", not impedance_gates.passed, "control_mode=2"))

    frame_moving_robot = FakeRobot(nominal_fake_state(), motion_mode=1)
    frame_moving_robot.motion_status.twists[0].linear[0] = (
        MAX_FRAME_LINEAR_VELOCITY_M_S + 0.001
    )
    frame_moving_gates = GateResult()
    validate_motion_status(frame_moving_robot, frame_moving_gates, "test")
    tests.append(("moving robot frame rejected", not frame_moving_gates.passed, "linear velocity above limit"))

    status_error_robot = FakeRobot(nominal_fake_state(), motion_mode=1)
    status_error_robot.motion_status.error_code = 7
    status_error_robot.motion_status.error_msg = "simulated motion-controller error"
    status_error_gates = GateResult()
    validate_motion_status(status_error_robot, status_error_gates, "test")
    tests.append(("motion status error rejected", not status_error_gates.passed, "error_code=7"))

    collision_robot = FakeRobot(nominal_fake_state(), motion_mode=1)
    collision_robot.motion_status.collision_pairs_1 = ["link_a/link_b"]
    collision_gates = GateResult()
    validate_motion_status(collision_robot, collision_gates, "test")
    tests.append(("reported collision rejected", not collision_gates.passed, "collision_pairs_1 non-empty"))

    baseline_gates = GateResult()
    baseline = extract_snapshot(nominal_fake_state(), baseline_gates, "base")
    changed_state = nominal_fake_state()
    changed_state["left_end_state"]["end_states"][0]["position"] += 0.01
    fresh_gates = GateResult()
    changed = extract_snapshot(changed_state, fresh_gates, "fresh")
    drift_gates = GateResult()
    drift_rejected = baseline is not None and changed is not None and not snapshots_fresh(baseline, changed, drift_gates)
    tests.append(("stale-state drift rejected", drift_rejected, ""))

    reporter.section("[SELF-TEST MATRIX]")
    for name, passed, detail in tests:
        reporter.emit(f"{'PASS' if passed else 'FAIL':4} | {name} | {detail}")
    overall = all(passed for _, passed, _ in tests)
    reporter.emit(f"self-test overall : {'PASS' if overall else 'FAIL'}")
    reporter.emit("real GDK imported : NO")
    reporter.emit("real robot commanded : NO")
    return overall


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Guarded first LEFT O10 index-PIP micro-open. Default is dry-run; "
            "real transmission requires --execute plus the exact confirmation token."
        )
    )
    parser.add_argument("--execute", action="store_true", help="Enable the one real command after all gates pass.")
    parser.add_argument("--confirm", default="", help=f"Required with --execute: {EXECUTION_TOKEN}")
    parser.add_argument("--retries", type=int, default=DEFAULT_RETRIES)
    parser.add_argument("--retry-interval", type=float, default=DEFAULT_RETRY_INTERVAL_SECONDS)
    parser.add_argument("--countdown", type=int, default=DEFAULT_COUNTDOWN_SECONDS)
    parser.add_argument("--verify-timeout", type=float, default=DEFAULT_VERIFY_TIMEOUT_SECONDS)
    parser.add_argument("--gdk-env", default=str(DEFAULT_GDK_ENV))
    parser.add_argument("--skip-env-bootstrap", action="store_true")
    parser.add_argument("--report-dir", default=None)
    parser.add_argument("--no-report", action="store_true")
    parser.add_argument("--self-test", action="store_true", help="Use mocks only; never import the real GDK.")
    args = parser.parse_args(argv)
    if not 1 <= args.retries <= 100:
        parser.error("--retries must be in [1, 100]")
    if not 0.0 <= args.retry_interval <= 60.0:
        parser.error("--retry-interval must be in [0, 60]")
    if not 0 <= args.countdown <= 30:
        parser.error("--countdown must be in [0, 30]")
    if args.execute and args.countdown < 3:
        parser.error("--countdown must be at least 3 seconds for real execution")
    if not 0.5 <= args.verify_timeout <= 30.0:
        parser.error("--verify-timeout must be in [0.5, 30]")
    if args.self_test and args.execute:
        parser.error("--self-test and --execute cannot be combined")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    reporter = Reporter()
    script_path = Path(__file__).resolve()
    mode = "SELF-TEST" if args.self_test else "EXECUTE" if args.execute else "DRY-RUN"

    reporter.section(PROGRAM_NAME)
    reporter.emit(f"script      : {script_path}")
    reporter.emit(f"timestamp   : {datetime.now().isoformat(timespec='seconds')}")
    reporter.emit(f"python      : {sys.version.splitlines()[0]}")
    reporter.emit(f"mode        : {mode}")
    reporter.emit(f"target      : {TARGET_JOINT}")
    reporter.emit(f"fixed delta : {FIXED_DELTA_RAD:+.6f} rad ({FIXED_DELTA_DEG:+.3f} deg, opening)")
    reporter.emit("max commands: 1")
    reporter.emit("auto-return : NO")
    reporter.emit(f"SDK truth   : {SDK_DOCUMENT_NAME}")
    reporter.emit(f"SDK SHA256  : {SDK_DOCUMENT_SHA256}")

    reporter.section("[SOURCE CONTROL-PATH AUDIT]")
    audit_ok, audit_errors, source_sha = audit_source(script_path)
    reporter.emit(f"status : {'PASS' if audit_ok else 'FAIL'}")
    reporter.emit(f"source SHA256 : {source_sha or '<unavailable>'}")
    if audit_errors:
        for error in audit_errors:
            reporter.emit(f" - {error}")
    else:
        reporter.emit("Exactly one explicit robot.move_ee_pos(payload) call exists.")
        reporter.emit("No other direct or literal-getattr motion/control call exists.")
        reporter.emit("Read-only Robot calls: get_end_state(), get_motion_control_status().")
        reporter.emit("Concurrent joint_servo_control flow in this process: NONE.")
        reporter.emit(
            "MotionStatus gate: 3 stable samples; mode=1 SERVO; robot frames and LEFT O10 "
            "physically still; control_mode=1 when exposed, otherwise "
            "NOT_EXPOSED_BY_BINDING (non-blocking)."
        )
    if not audit_ok:
        reporter.emit("ABORTED before GDK import/initialization.")
        return 2

    if args.self_test:
        success = run_self_test(reporter)
        reporter.section("110 SUMMARY")
        reporter.emit(f"source control-path audit : {'PASS' if audit_ok else 'FAIL'}")
        reporter.emit(f"simulated motion-path test : {'PASS' if success else 'FAIL'}")
        reporter.emit("real GDK imported : NO")
        reporter.emit("real robot commanded : NO")
        if not args.no_report:
            report_path = make_report_path(args.report_dir)
            report_path.parent.mkdir(parents=True, exist_ok=True)
            reporter.emit(f"Report path: {report_path}")
            reporter.write(report_path)
        return 0 if success else 1

    confirmation_ok = (not args.execute) or (args.confirm == EXECUTION_TOKEN)
    if args.execute and not confirmation_ok:
        reporter.section("[EXECUTION INTERLOCK]")
        reporter.emit("status : FAIL")
        reporter.emit(f"Required: --confirm {EXECUTION_TOKEN}")
        reporter.emit("No GDK import, initialization, or command occurred.")
        return 2

    gdk_module: Any = None
    robot: Any = None
    initialized = False
    released = False
    gates = GateResult()
    outcome = CommandOutcome(requested=args.execute)
    runtime_errors: List[str] = []
    baseline: Optional[Snapshot] = None
    target: Optional[List[float]] = None
    payload: Any = None

    try:
        gdk_module = import_gdk_or_restart(args, reporter)
        reporter.section("[LIVE GDK SESSION]")
        reporter.emit(f"module file : {getattr(gdk_module, '__file__', '<unknown>')}")
        init_result = gdk_module.gdk_init()
        initialized = result_is_success(gdk_module, init_result)
        reporter.emit(f"GDK initialized : {'PASS' if initialized else 'FAIL'} ({init_result!r})")
        if not initialized:
            raise RuntimeError(f"gdk_init returned {init_result!r}")
        robot = gdk_module.Robot()
        reporter.emit(f"Robot created : PASS ({type_name(robot)})")

        reporter.section("[BASELINE READ-ONLY STATE]")
        end_state, _ = get_end_state_with_retry(
            robot, args.retries, args.retry_interval, reporter, "baseline"
        )
        baseline = extract_snapshot(end_state, gates, "baseline")
        validate_motion_status(robot, gates, "baseline")
        if baseline is None:
            raise RuntimeError("baseline snapshot unavailable")

        target = build_target_positions(baseline.positions)
        validate_target(baseline.positions, target, gates, "baseline")
        payload = build_payload(gdk_module, target)
        validate_payload(payload, target, gates, "baseline")
        emit_snapshot(baseline, target, reporter, "[BASELINE + CANDIDATE TARGET]")
        emit_checks(gates, reporter, "[PRE-EXECUTION GATES]")

        if not gates.passed:
            raise RuntimeError("pre-execution gates failed")

        if not args.execute:
            reporter.section("[DRY-RUN BOUNDARY]")
            reporter.emit("All checks passed, but --execute was not supplied.")
            reporter.emit("payload transmitted : NO")
            reporter.emit("motion executed : NO")
        else:
            countdown(args.countdown, reporter)

            reporter.section("[FRESH READ-ONLY RECHECK]")
            fresh_value, _ = get_end_state_with_retry(
                robot, args.retries, args.retry_interval, reporter, "fresh"
            )
            fresh_gates = GateResult()
            fresh = extract_snapshot(fresh_value, fresh_gates, "fresh")
            validate_motion_status(robot, fresh_gates, "fresh")
            if fresh is None:
                raise RuntimeError("fresh snapshot unavailable")
            snapshots_fresh(baseline, fresh, fresh_gates)

            # Rebuild from the fresh state. The target always changes exactly
            # one joint by the frozen -0.020 rad; all others use fresh values.
            fresh_target = build_target_positions(fresh.positions)
            validate_target(fresh.positions, fresh_target, fresh_gates, "fresh")
            fresh_payload = build_payload(gdk_module, fresh_target)
            validate_payload(fresh_payload, fresh_target, fresh_gates, "fresh")
            emit_snapshot(fresh, fresh_target, reporter, "[FRESH TARGET TO BE TRANSMITTED]")
            emit_checks(fresh_gates, reporter, "[FINAL TRANSMISSION GATES]")
            if not fresh_gates.passed:
                raise RuntimeError("final fresh-state transmission gates failed")

            reporter.section("[ONE CONTROL TRANSMISSION]")
            reporter.emit("Calling move_ee_pos exactly once ...")
            outcome.transmitted = True
            outcome.api_return = transmit_once(robot, fresh_payload)
            outcome.api_accepted = result_is_success(gdk_module, outcome.api_return)
            reporter.emit(f"API return : {outcome.api_return!r}")
            reporter.emit(f"API accepted : {'PASS' if outcome.api_accepted else 'FAIL'}")
            if not outcome.api_accepted:
                raise RuntimeError(f"move_ee_pos returned {outcome.api_return!r}")

            reporter.section("[POST-COMMAND READ-ONLY VERIFICATION]")
            observed, positions, verification_error = verify_target_observed(
                robot, fresh_target, args.verify_timeout, reporter
            )
            outcome.target_observed = observed
            outcome.observed_positions = positions
            outcome.verification_error = verification_error
            target = fresh_target
            payload = fresh_payload
            reporter.emit(f"target observed : {'PASS' if observed else 'FAIL'}")
            if verification_error:
                reporter.emit(f"verification detail : {verification_error}")
    except KeyboardInterrupt:
        runtime_errors.append("KeyboardInterrupt: operator aborted")
        reporter.emit("Operator aborted with Ctrl+C.")
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
                runtime_errors.append("gdk_release: " + short_exception(exc))
                reporter.emit(f"GDK released : FAIL ({runtime_errors[-1]})")

    dry_run_success = (
        not args.execute
        and audit_ok
        and initialized
        and released
        and gates.passed
        and not runtime_errors
        and payload is not None
        and not outcome.transmitted
    )
    execute_success = (
        args.execute
        and audit_ok
        and initialized
        and released
        and gates.passed
        and not runtime_errors
        and outcome.transmitted
        and outcome.api_accepted
        and outcome.target_observed
    )
    success = execute_success if args.execute else dry_run_success

    reporter.section("110 SUMMARY")
    reporter.emit(f"mode : {mode}")
    reporter.emit(f"source control-path audit : {'PASS' if audit_ok else 'FAIL'}")
    reporter.emit(f"GDK initialized : {'PASS' if initialized else 'FAIL'}")
    reporter.emit(f"Robot created : {'PASS' if robot is not None else 'FAIL'}")
    reporter.emit(f"baseline validation : {'PASS' if gates.passed else 'FAIL'}")
    reporter.emit(f"payload built : {'PASS' if payload is not None else 'FAIL'}")
    reporter.emit(f"command requested : {'YES' if args.execute else 'NO'}")
    reporter.emit(f"command transmitted : {'YES' if outcome.transmitted else 'NO'}")
    reporter.emit(f"API accepted : {'YES' if outcome.api_accepted else 'NO'}")
    reporter.emit(f"target observed : {'YES' if outcome.target_observed else 'NO'}")
    reporter.emit("automatic return command : NO")
    reporter.emit(f"GDK released : {'PASS' if released else 'FAIL'}")
    if success and not args.execute:
        reporter.emit("Final verdict: DRY-RUN PASS - ready for separate operator-reviewed execution.")
    elif success:
        reporter.emit("Final verdict: EXECUTION PASS - one micro-open command accepted and target observed.")
    elif outcome.transmitted and not outcome.target_observed:
        reporter.emit("Final verdict: COMMAND SENT, POST-VERIFICATION FAILED/UNKNOWN.")
        reporter.emit("The hand may have moved. Inspect the robot before any further command.")
    else:
        reporter.emit("Final verdict: BLOCKED/FAILED - no successful 110 completion.")
    for error in runtime_errors:
        reporter.emit(f" - {error}")

    if not args.no_report:
        try:
            report_path = make_report_path(args.report_dir)
            report_path.parent.mkdir(parents=True, exist_ok=True)
            reporter.emit(f"Report path: {report_path}")
            reporter.write(report_path)
        except Exception as exc:
            reporter.emit(f"Report write failed: {short_exception(exc)}")
            success = False

    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
