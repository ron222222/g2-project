#!/usr/bin/env python3
"""
110 support probe - pure read-only MotionControlStatus contract verification.

Purpose
-------
1. Verify the MotionControlStatus schema and numeric mappings against the
   complete GDK 4.1.5 Markdown document.
2. Inspect the current Python binding object and sample live status values.
3. Resolve mode values using the documented mapping only:
      0 = STOP, 1 = SERVO, 2 = PLANNING.
4. Produce evidence for review of the separate motion-capable 110 program.

Safety boundary
---------------
This program calls only Robot.get_motion_control_status(). It never calls a
motion, servo, planning, stop, mode-changing, error-clearing, or configuration
API. It constructs no command payload and cannot authorize 110 execution.

The automatic env.sh bootstrap deliberately sources env.sh with zero positional
arguments. This avoids the previous bug where env.sh received its own filename
as $1 and treated that file path as the GDK application root.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import importlib
import math
import os
from pathlib import Path
import re
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple


PROGRAM_NAME = "110 G2 MotionControlStatus CONTRACT VERIFICATION v1 (READ-ONLY)"
SDK_DOCUMENT_NAME = "GDK_SDK_4.1.5.md"
SDK_DOCUMENT_SHA256 = "e952f9d0c383c468b7384c46274b12d3059827c2c95ecb0506082a48c49df90d"
DEFAULT_GDK_ENV = Path.home() / ".cache/agibot/app/env.sh"
BOOTSTRAP_MARKER = "AGIBOT_GDK_ENV_RESTARTED_FOR_MCS_PROBE"

MODE_MAP: Dict[int, str] = {0: "STOP", 1: "SERVO", 2: "PLANNING"}
CONTROL_MODE_MAP: Dict[int, str] = {
    1: "JOINT_POSITION_PD_TRAJECTORY",
    2: "CARTESIAN_IMPEDANCE",
    3: "JOINT_IMPEDANCE",
}

EXPECTED_FIELDS: Tuple[str, ...] = (
    "mode",
    "control_mode",
    "error_code",
    "error_msg",
    "frame_names",
    "frame_poses",
    "twists",
    "wrenches",
    "arm_disturbance_force",
    "collision_pairs_1",
    "collision_pairs_2",
)
VECTOR_FIELDS: Tuple[str, ...] = (
    "frame_names",
    "frame_poses",
    "twists",
    "wrenches",
    "arm_disturbance_force",
    "collision_pairs_1",
    "collision_pairs_2",
)

# Any call to one of these names is forbidden anywhere in this read-only source.
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
        "set_tcp_transform",
        "stop_motion",
        "emergency_stop",
        "clear_motion_control_error",
    }
)
NO_DEFAULT = object()


@dataclass
class Check:
    name: str
    passed: bool
    detail: str


@dataclass
class CheckSet:
    checks: List[Check] = field(default_factory=list)

    def add(self, name: str, passed: bool, detail: str) -> None:
        self.checks.append(Check(name=name, passed=bool(passed), detail=str(detail)))

    @property
    def passed(self) -> bool:
        return bool(self.checks) and all(item.passed for item in self.checks)


@dataclass
class StatusSample:
    index: int
    timestamp: str
    object_type: str
    object_repr: str
    mode: int
    control_mode: int
    error_code: int
    error_msg: str
    vectors: Dict[str, List[Any]]


class Reporter:
    def __init__(self) -> None:
        self.lines: List[str] = []

    def emit(self, text: str = "") -> None:
        print(text, flush=True)
        self.lines.append(text)

    def section(self, title: str) -> None:
        self.emit()
        self.emit("=" * 112)
        self.emit(title)
        self.emit("=" * 112)

    def write(self, path: Path) -> None:
        path.write_text("\n".join(self.lines) + "\n", encoding="utf-8")


def short_exception(exc: BaseException) -> str:
    return f"{type(exc).__name__}: {exc}"


def type_name(value: Any) -> str:
    cls = type(value)
    return f"{cls.__module__}.{cls.__name__}"


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


def strict_int(value: Any, label: str) -> int:
    if isinstance(value, bool):
        raise TypeError(f"{label} is bool, not an integer status value")
    converted = int(value)
    try:
        numeric = float(value)
    except (TypeError, ValueError, OverflowError):
        numeric = float(converted)
    if not math.isfinite(numeric) or numeric != float(converted):
        raise ValueError(f"{label} is not an integral finite value: {value!r}")
    return converted


def as_vector(value: Any, label: str) -> List[Any]:
    if isinstance(value, (str, bytes, bytearray)):
        raise TypeError(f"{label} is text/bytes, not a vector")
    try:
        return list(value)
    except TypeError as exc:
        raise TypeError(f"{label} is not iterable: {value!r}") from exc


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


def emit_checks(checks: CheckSet, reporter: Reporter, title: str) -> None:
    reporter.section(title)
    for item in checks.checks:
        reporter.emit(f"{'PASS' if item.passed else 'FAIL':4} | {item.name} | {item.detail}")
    reporter.emit(f"overall : {'PASS' if checks.passed else 'FAIL'}")


def call_name(node: ast.Call) -> Optional[str]:
    func = node.func
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return None


def audit_read_only_source(script_path: Path) -> Tuple[CheckSet, str]:
    checks = CheckSet()
    data = script_path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    try:
        tree = ast.parse(data.decode("utf-8"), filename=str(script_path))
    except Exception as exc:
        checks.add("source parses as Python", False, short_exception(exc))
        return checks, digest

    checks.add("source parses as Python", True, f"AST nodes={sum(1 for _ in ast.walk(tree))}")
    forbidden: List[str] = []
    robot_methods: List[str] = []
    dynamic_robot_calls: List[str] = []

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = call_name(node)
        if name in FORBIDDEN_CALL_NAMES:
            forbidden.append(f"line {getattr(node, 'lineno', '?')}: {name}()")
        if isinstance(node.func, ast.Attribute):
            owner = node.func.value
            if isinstance(owner, ast.Name) and owner.id == "robot":
                robot_methods.append(node.func.attr)
        if isinstance(node.func, ast.Name) and node.func.id == "getattr" and node.args:
            owner = node.args[0]
            if isinstance(owner, ast.Name) and owner.id == "robot":
                dynamic_robot_calls.append(f"line {getattr(node, 'lineno', '?')}: getattr(robot, ...)")

    checks.add(
        "no forbidden control/mutation call",
        not forbidden,
        "none" if not forbidden else "; ".join(forbidden),
    )
    checks.add(
        "Robot method allow-list",
        bool(robot_methods) and set(robot_methods) == {"get_motion_control_status"},
        f"calls={robot_methods!r}; allowed=['get_motion_control_status']",
    )
    checks.add(
        "no dynamic Robot method dispatch",
        not dynamic_robot_calls,
        "none" if not dynamic_robot_calls else "; ".join(dynamic_robot_calls),
    )
    return checks, digest


def find_sdk_document(explicit: Optional[str], script_path: Path) -> Path:
    candidates: List[Path] = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    candidates.extend(
        [
            script_path.parent / SDK_DOCUMENT_NAME,
            script_path.parent.parent / SDK_DOCUMENT_NAME,
            Path.cwd() / SDK_DOCUMENT_NAME,
        ]
    )
    seen: set = set()
    for candidate in candidates:
        resolved = candidate.resolve()
        if str(resolved) in seen:
            continue
        seen.add(str(resolved))
        if resolved.is_file():
            return resolved
    rendered = "\n - ".join(str(item.expanduser()) for item in candidates)
    raise FileNotFoundError(f"{SDK_DOCUMENT_NAME} not found. Checked:\n - {rendered}")


def normalize_doc(text: str) -> str:
    return text.replace("\r\n", "\n").replace("\\_", "_")


def verify_document(document_path: Path) -> Tuple[CheckSet, str, str]:
    checks = CheckSet()
    data = document_path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    text = data.decode("utf-8-sig")
    normalized = normalize_doc(text)

    # The original 4.1.5 Markdown uses "#### 6. `GetMotionControlStatus()`".
    # Some exported renderings add/remove one # or the backticks, so locate the
    # numbered heading without weakening the exact contract checks below.
    heading_match = re.search(
        r"^#{3,6}\s+6\.\s+`?GetMotionControlStatus\(\)`?\s*$",
        normalized,
        flags=re.MULTILINE,
    )
    start = heading_match.start() if heading_match else -1
    next_match = (
        re.search(r"^#{3,6}\s+7\.\s+", normalized[heading_match.end():], flags=re.MULTILINE)
        if heading_match
        else None
    )
    next_heading = heading_match.end() + next_match.start() if heading_match and next_match else -1
    section = normalized[start:next_heading if next_heading >= 0 else start + 30000] if start >= 0 else ""

    checks.add("SDK document SHA256", digest == SDK_DOCUMENT_SHA256, f"actual={digest}")
    checks.add("GetMotionControlStatus section located", start >= 0, f"offset={start}")
    checks.add(
        "official motion-mode mapping",
        "运动模式，0为停止，1为伺服，2为规划" in section
        and "motion mode, 0: stop, 1: servo, 2: planning" in section,
        "0=STOP, 1=SERVO, 2=PLANNING",
    )
    checks.add(
        "official control-mode mapping",
        "控制模式，1为关节位控（PD/轨迹），2为笛卡尔阻抗，3为关节阻抗" in section
        and "control mode, 1: joint position, 2: cartesian impedance, 3: joint impedance" in section,
        "1=JOINT_POSITION, 2=CARTESIAN_IMPEDANCE, 3=JOINT_IMPEDANCE",
    )
    checks.add("error_code zero semantics", "错误码，0表示正常" in section, "0 means normal")
    checks.add(
        "collision-list semantics",
        "否则为空" in section and "collision_pairs_1" in section and "collision_pairs_2" in section,
        "document says collision lists are empty when no collision is reported",
    )

    missing_fields = [field for field in EXPECTED_FIELDS if field not in section]
    checks.add(
        "all documented MotionControlStatus fields",
        not missing_fields,
        "all 11 fields found" if not missing_fields else f"missing={missing_fields!r}",
    )
    return checks, digest, section


def bootstrap_argv(env_script: Path, script_path: Path, original_args: Sequence[str]) -> List[str]:
    # Keep the restart command in an array, then source env.sh inside a no-argument
    # shell function. A sourced script sees the function's empty positional list,
    # so env.sh cannot mistake its own filename (or Python argv) for an app root.
    command = (
        '_g2_env_script="$1"; shift; '
        '_g2_restart_argv=("$@"); '
        '_g2_source_env_without_args() { source "$_g2_env_script"; }; '
        '_g2_source_env_without_args && '
        f'export {BOOTSTRAP_MARKER}=1 && '
        'exec "${_g2_restart_argv[@]}"'
    )
    return [
        "/bin/bash",
        "-lc",
        command,
        "mcs-bootstrap",
        str(env_script),
        sys.executable,
        str(script_path),
        *original_args,
    ]


def import_gdk_or_restart(args: argparse.Namespace, reporter: Reporter, script_path: Path) -> Any:
    try:
        return importlib.import_module("agibot_gdk")
    except Exception as first_exc:
        if args.skip_env_bootstrap or os.environ.get(BOOTSTRAP_MARKER) == "1":
            raise RuntimeError(
                "Unable to import agibot_gdk after the available bootstrap path: "
                + short_exception(first_exc)
            ) from first_exc

        env_script = Path(args.gdk_env).expanduser().resolve()
        if not env_script.is_file():
            raise RuntimeError(
                f"agibot_gdk import failed and env.sh was not found: {env_script}; "
                f"original error: {short_exception(first_exc)}"
            ) from first_exc

        reporter.emit("[ENV] Restarting once with official env.sh (zero positional arguments to env.sh) ...")
        argv = bootstrap_argv(env_script, script_path, sys.argv[1:])
        sys.stdout.flush()
        sys.stderr.flush()
        os.execv("/bin/bash", argv)
        raise AssertionError("os.execv returned unexpectedly")


def parse_status(index: int, status: Any) -> Tuple[StatusSample, CheckSet]:
    checks = CheckSet()
    values: Dict[str, Any] = {}
    missing: List[str] = []
    for field_name in EXPECTED_FIELDS:
        try:
            values[field_name] = read_field(status, field_name)
        except Exception:
            missing.append(field_name)
    checks.add(
        f"sample {index:02d} fields present",
        not missing,
        "all 11 present" if not missing else f"missing={missing!r}",
    )
    if missing:
        raise RuntimeError(f"MotionControlStatus missing fields: {missing!r}")

    mode = strict_int(values["mode"], "mode")
    control_mode = strict_int(values["control_mode"], "control_mode")
    error_code = strict_int(values["error_code"], "error_code")
    error_msg_value = values["error_msg"]
    error_msg = str(error_msg_value)

    vectors: Dict[str, List[Any]] = {}
    vector_errors: List[str] = []
    for name in VECTOR_FIELDS:
        try:
            vectors[name] = as_vector(values[name], name)
        except Exception as exc:
            vector_errors.append(f"{name}: {short_exception(exc)}")
            vectors[name] = []

    checks.add(
        f"sample {index:02d} mode documented",
        mode in MODE_MAP,
        f"mode={mode} => {MODE_MAP.get(mode, 'UNDOCUMENTED')}",
    )
    checks.add(
        f"sample {index:02d} control_mode documented",
        control_mode in CONTROL_MODE_MAP,
        f"control_mode={control_mode} => {CONTROL_MODE_MAP.get(control_mode, 'UNDOCUMENTED')}",
    )
    checks.add(
        f"sample {index:02d} error_code uint8",
        0 <= error_code <= 255,
        f"error_code={error_code}",
    )
    checks.add(
        f"sample {index:02d} error_msg string-compatible",
        isinstance(error_msg_value, str),
        f"type={type_name(error_msg_value)}, value={error_msg!r}",
    )
    checks.add(
        f"sample {index:02d} vector fields iterable",
        not vector_errors,
        "all 7 iterable" if not vector_errors else "; ".join(vector_errors),
    )
    checks.add(
        f"sample {index:02d} frame_names strings",
        all(isinstance(item, str) for item in vectors["frame_names"]),
        f"count={len(vectors['frame_names'])}",
    )
    checks.add(
        f"sample {index:02d} collision pairs strings",
        all(isinstance(item, str) for name in ("collision_pairs_1", "collision_pairs_2") for item in vectors[name]),
        f"count1={len(vectors['collision_pairs_1'])}, count2={len(vectors['collision_pairs_2'])}",
    )

    sample = StatusSample(
        index=index,
        timestamp=datetime.now().isoformat(timespec="milliseconds"),
        object_type=type_name(status),
        object_repr=repr(status),
        mode=mode,
        control_mode=control_mode,
        error_code=error_code,
        error_msg=error_msg,
        vectors=vectors,
    )
    return sample, checks


def sample_live_status(robot: Any, count: int, interval: float, reporter: Reporter) -> Tuple[List[StatusSample], CheckSet]:
    samples: List[StatusSample] = []
    checks = CheckSet()
    for index in range(1, count + 1):
        try:
            status = robot.get_motion_control_status()
            sample, sample_checks = parse_status(index, status)
            samples.append(sample)
            checks.checks.extend(sample_checks.checks)
            reporter.emit(
                f"sample {index:02d}/{count:02d} | "
                f"mode={sample.mode}({MODE_MAP.get(sample.mode, 'UNDOCUMENTED')}) | "
                f"control_mode={sample.control_mode}({CONTROL_MODE_MAP.get(sample.control_mode, 'UNDOCUMENTED')}) | "
                f"error_code={sample.error_code} | "
                f"collisions={len(sample.vectors['collision_pairs_1']) + len(sample.vectors['collision_pairs_2'])}"
            )
        except Exception as exc:
            checks.add(f"sample {index:02d} readable", False, short_exception(exc))
            reporter.emit(f"sample {index:02d}/{count:02d} | FAIL | {short_exception(exc)}")
        if index < count and interval > 0:
            time.sleep(interval)
    checks.add("requested sample count", len(samples) == count, f"received={len(samples)}, requested={count}")
    return samples, checks


def emit_sample_details(samples: Sequence[StatusSample], reporter: Reporter) -> None:
    reporter.section("[RUNTIME OBJECT + FIELD EVIDENCE]")
    if not samples:
        reporter.emit("No valid samples.")
        return
    first = samples[0]
    reporter.emit(f"object type : {first.object_type}")
    reporter.emit(f"repr        : {first.object_repr}")
    for name in VECTOR_FIELDS:
        reporter.emit(f"{name:24s}: count={len(first.vectors[name])}")
    reporter.emit(f"frame_names              : {first.vectors['frame_names']!r}")
    reporter.emit(f"collision_pairs_1        : {first.vectors['collision_pairs_1']!r}")
    reporter.emit(f"collision_pairs_2        : {first.vectors['collision_pairs_2']!r}")


def emit_interpretation(samples: Sequence[StatusSample], reporter: Reporter) -> None:
    reporter.section("[DOCUMENTED INTERPRETATION]")
    if not samples:
        reporter.emit("No runtime mode could be interpreted.")
        return
    modes = [item.mode for item in samples]
    control_modes = [item.control_mode for item in samples]
    reporter.emit(f"observed modes          : {modes!r}")
    reporter.emit(f"observed control_modes  : {control_modes!r}")
    reporter.emit(f"mode labels             : {[MODE_MAP.get(value, 'UNDOCUMENTED') for value in modes]!r}")
    reporter.emit(
        "Official GDK 4.1.5 meaning: mode=0 STOP, mode=1 SERVO, mode=2 PLANNING."
    )
    if set(modes) == {1}:
        reporter.emit("CONCLUSION: every sample reported mode=1, whose official meaning is SERVO (not STOP).")
    elif set(modes) == {0}:
        reporter.emit("CONCLUSION: every sample reported mode=0, whose official meaning is STOP.")
    elif set(modes) == {2}:
        reporter.emit("CONCLUSION: every sample reported mode=2, whose official meaning is PLANNING.")
    else:
        reporter.emit("CONCLUSION: motion mode changed during sampling; inspect the sequence above.")
    reporter.emit(
        "This field identifies the controller motion mode; by itself it is not proof of physical joint velocity."
    )
    reporter.emit("No motion/control method was called. This probe does not authorize 110 execution.")


class FakeStatus:
    def __init__(self, mode: int = 1, control_mode: int = 1) -> None:
        self.mode = mode
        self.control_mode = control_mode
        self.error_code = 0
        self.error_msg = ""
        self.frame_names = ["left_hand", "right_hand"]
        self.frame_poses: List[Any] = [object(), object()]
        self.twists: List[Any] = [object(), object()]
        self.wrenches: List[Any] = [object(), object()]
        self.arm_disturbance_force = [0.0] * 14
        self.collision_pairs_1: List[str] = []
        self.collision_pairs_2: List[str] = []


class FakeRobot:
    def __init__(self) -> None:
        self.read_count = 0

    def get_motion_control_status(self) -> FakeStatus:
        self.read_count += 1
        return FakeStatus(mode=1, control_mode=1)


def run_self_test(reporter: Reporter) -> bool:
    reporter.section("[SELF-TEST - NO REAL GDK]")
    samples, checks = sample_live_status(FakeRobot(), 3, 0.0, reporter)
    emit_checks(checks, reporter, "[SELF-TEST RUNTIME CONTRACT]")
    mapping_ok = bool(samples) and all(MODE_MAP[item.mode] == "SERVO" for item in samples)
    reporter.emit(f"mode=1 maps to SERVO : {'PASS' if mapping_ok else 'FAIL'}")
    reporter.emit("real GDK imported     : NO")
    reporter.emit("real robot contacted  : NO")
    return checks.passed and mapping_ok


def make_report_path(report_dir: Optional[str], script_path: Path) -> Path:
    base = Path(report_dir).expanduser().resolve() if report_dir else script_path.parent
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return base / f"{script_path.stem}_report_{stamp}.txt"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Pure read-only GDK MotionControlStatus contract verification.")
    parser.add_argument("--sdk-doc", default=None, help=f"Path to {SDK_DOCUMENT_NAME}.")
    parser.add_argument("--samples", type=int, default=10, help="Read-only status samples (default: 10).")
    parser.add_argument("--interval", type=float, default=0.20, help="Seconds between samples (default: 0.20).")
    parser.add_argument("--gdk-env", default=str(DEFAULT_GDK_ENV), help="Official env.sh for one automatic restart.")
    parser.add_argument("--skip-env-bootstrap", action="store_true", help="Do not auto-source env.sh on import failure.")
    parser.add_argument("--document-only", action="store_true", help="Verify source/document only; do not import GDK.")
    parser.add_argument("--self-test", action="store_true", help="Use a fake read-only Robot; never import real GDK.")
    parser.add_argument("--report-dir", default=None)
    parser.add_argument("--no-report", action="store_true")
    args = parser.parse_args(argv)
    if not 1 <= args.samples <= 100:
        parser.error("--samples must be in [1, 100]")
    if not 0.0 <= args.interval <= 60.0:
        parser.error("--interval must be in [0, 60]")
    if args.document_only and args.self_test:
        parser.error("--document-only and --self-test cannot be combined")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    reporter = Reporter()
    script_path = Path(__file__).resolve()
    reporter.section(PROGRAM_NAME)
    reporter.emit(f"script       : {script_path}")
    reporter.emit(f"timestamp    : {datetime.now().isoformat(timespec='seconds')}")
    reporter.emit(f"python       : {sys.version.splitlines()[0]}")
    reporter.emit("safety mode  : READ-ONLY; get_motion_control_status() only")
    reporter.emit("motion APIs  : NONE")

    source_checks, source_sha = audit_read_only_source(script_path)
    emit_checks(source_checks, reporter, "[SOURCE READ-ONLY AUDIT]")
    reporter.emit(f"source SHA256: {source_sha}")

    runtime_checks = CheckSet()
    samples: List[StatusSample] = []
    initialized = False
    released = False
    runtime_errors: List[str] = []

    try:
        document_path = find_sdk_document(args.sdk_doc, script_path)
        document_checks, document_sha, _ = verify_document(document_path)
        reporter.section("[SDK DOCUMENT]")
        reporter.emit(f"path         : {document_path}")
        reporter.emit(f"SHA256       : {document_sha}")
        emit_checks(document_checks, reporter, "[DOCUMENT CONTRACT CHECKS]")
    except Exception as exc:
        document_checks = CheckSet()
        document_checks.add("SDK document readable", False, short_exception(exc))
        emit_checks(document_checks, reporter, "[DOCUMENT CONTRACT CHECKS]")

    if not source_checks.passed or not document_checks.passed:
        runtime_errors.append("Source/document gate failed; runtime inspection not started.")
    elif args.document_only:
        reporter.section("[DOCUMENT-ONLY BOUNDARY]")
        reporter.emit("GDK imported       : NO")
        reporter.emit("robot contacted    : NO")
    elif args.self_test:
        if not run_self_test(reporter):
            runtime_errors.append("self-test failed")
        else:
            runtime_checks.add("self-test", True, "fake mode=1 mapped to SERVO")
    else:
        gdk_module: Any = None
        try:
            gdk_module = import_gdk_or_restart(args, reporter, script_path)
            reporter.section("[LIVE READ-ONLY GDK SESSION]")
            reporter.emit(f"module file  : {getattr(gdk_module, '__file__', '<unknown>')}")
            init_result = gdk_module.gdk_init()
            initialized = result_is_success(gdk_module, init_result)
            reporter.emit(f"GDK initialized : {'PASS' if initialized else 'FAIL'} ({init_result!r})")
            if not initialized:
                raise RuntimeError(f"gdk_init returned {init_result!r}")
            robot = gdk_module.Robot()
            reporter.emit(f"Robot created    : PASS ({type_name(robot)})")
            reporter.section("[LIVE READ-ONLY SAMPLES]")
            samples, runtime_checks = sample_live_status(robot, args.samples, args.interval, reporter)
            emit_sample_details(samples, reporter)
            emit_checks(runtime_checks, reporter, "[RUNTIME CONTRACT CHECKS]")
            emit_interpretation(samples, reporter)
        except Exception as exc:
            runtime_errors.append(short_exception(exc))
            reporter.emit(f"runtime error: {runtime_errors[-1]}")
        finally:
            if initialized and gdk_module is not None:
                try:
                    release_result = gdk_module.gdk_release()
                    released = result_is_success(gdk_module, release_result)
                    reporter.emit(f"GDK released    : {'PASS' if released else 'FAIL'} ({release_result!r})")
                    if not released:
                        runtime_errors.append(f"gdk_release returned {release_result!r}")
                except Exception as exc:
                    runtime_errors.append("gdk_release: " + short_exception(exc))
                    reporter.emit(f"GDK released    : FAIL ({runtime_errors[-1]})")

    if args.document_only:
        success = source_checks.passed and document_checks.passed and not runtime_errors
    elif args.self_test:
        success = source_checks.passed and document_checks.passed and runtime_checks.passed and not runtime_errors
    else:
        success = (
            source_checks.passed
            and document_checks.passed
            and initialized
            and released
            and runtime_checks.passed
            and len(samples) == args.samples
            and not runtime_errors
        )

    reporter.section("MOTIONCONTROLSTATUS CONTRACT VERIFICATION SUMMARY")
    reporter.emit(f"source read-only audit : {'PASS' if source_checks.passed else 'FAIL'}")
    reporter.emit(f"document contract      : {'PASS' if document_checks.passed else 'FAIL'}")
    if args.document_only:
        reporter.emit("runtime binding        : NOT RUN (--document-only)")
    elif args.self_test:
        reporter.emit(f"runtime self-test      : {'PASS' if runtime_checks.passed else 'FAIL'}")
    else:
        reporter.emit(f"runtime binding        : {'PASS' if runtime_checks.passed else 'FAIL'}")
        reporter.emit(f"valid samples          : {len(samples)}/{args.samples}")
        reporter.emit(f"GDK released           : {'PASS' if released else 'FAIL'}")
    reporter.emit("control payload built  : NO")
    reporter.emit("control method called  : NO")
    reporter.emit("motion commanded       : NO")
    if success:
        reporter.emit("Final verdict: PASS - documented/read-only contract evidence collected.")
    else:
        reporter.emit("Final verdict: BLOCKED/FAILED - inspect the failed evidence above.")
    for error in runtime_errors:
        reporter.emit(f" - {error}")
    reporter.emit("Boundary: this probe does not authorize or execute 110 motion.")

    if not args.no_report:
        try:
            report_path = make_report_path(args.report_dir, script_path)
            report_path.parent.mkdir(parents=True, exist_ok=True)
            reporter.emit(f"Report path: {report_path}")
            reporter.write(report_path)
        except Exception as exc:
            reporter.emit(f"Report write failed: {short_exception(exc)}")
            success = False
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
