#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
109_g2_o10_control_surface_probe_v2.py

G2 + O10 runtime Python-binding control-surface probe.

Safety contract
---------------
* READ ONLY: this program never invokes a motion/control/write API.
* The only Robot method invoked is get_end_state().
* gdk_init(), Robot(), safe data-container constructors, and gdk_release()
  are used only to inspect the installed binding and read current O10 state.
* Candidate APIs are inspected with dir/getattr/inspect only.

Purpose
-------
The GDK 4.1.5 documentation and the installed Python binding can differ.
This probe inventories the actual binding and prints type, signature, and
__doc__ information for potential O10 control paths before any motion program
is designed.
"""

from __future__ import annotations

import argparse
import ast
import datetime as _datetime
import importlib
import inspect
import os
from pathlib import Path
import platform
import shlex
import sys
import time
import traceback
import types
from typing import Any, Dict, Iterable, List, Optional, Sequence, TextIO, Tuple


PROGRAM_VERSION = "109_v2_control_surface_probe"
READ_ONLY_PROBE = True
ENABLE_REAL_MOTION = False
DEFAULT_ENV_SCRIPT = "~/.cache/agibot/app/env.sh"
ENV_RESTART_GUARD = "_G2_GDK_109_V2_ENV_READY"

SEARCH_KEYWORDS: Tuple[str, ...] = (
    "hand",
    "tool",
    "end",
    "effector",
    "joint",
    "move",
    "control",
    "servo",
    "request",
)

FOCUSED_ROBOT_MEMBERS: Tuple[str, ...] = (
    "joint_control_request",
    "joint_servo_control",
    "move_arm_joint",
    "move_arm_joint_servo",
    "move_ee_pos",
    "move_end_effector_joint",
    "end_effector_pose_control",
    "set_reference_frame_poses",
    "set_reference_frame_pose",
    "get_end_state",
)

FOCUSED_MODULE_SYMBOLS: Tuple[str, ...] = (
    "Robot",
    "JointControlReq",
    "JointServoControlReq",
    "JointState",
    "JointStates",
    "EndState",
    "DualEndState",
    "EndEffectorPose",
    "EndEffectorControlGroup",
    "PlanningGroup",
    "LEFT_TOOL",
    "RIGHT_TOOL",
)

# Only these known, zero-argument data containers may be constructed for
# default-field introspection. No unknown class is ever instantiated.
SAFE_ZERO_ARG_DATA_TYPES: Tuple[str, ...] = (
    "JointControlReq",
    "JointServoControlReq",
    "JointState",
    "JointStates",
    "EndState",
    "DualEndState",
    "EndEffectorPose",
)

# Direct calls to any of these Robot members would violate this probe's safety
# contract. Names are kept as data for the AST self-audit and are never called.
FORBIDDEN_DIRECT_CALLS = frozenset(
    {
        "joint_control_request",
        "joint_servo_control",
        "move_arm_joint",
        "move_arm_joint_servo",
        "move_ee_pos",
        "move_end_effector_joint",
        "end_effector_pose_control",
        "set_reference_frame_poses",
        "set_reference_frame_pose",
        "set_control_mode",
        "set_joint_impedance",
        "set_cartesian_impedance",
        "set_tcp_transform",
        "switch_end_pose_control_frame",
    }
)

SEPARATOR = "=" * 108
SUB_SEPARATOR = "-" * 108


class Tee:
    """Mirror text output to the terminal and an optional UTF-8 report."""

    def __init__(self, *streams: TextIO) -> None:
        self._streams = streams

    def write(self, data: str) -> int:
        for stream in self._streams:
            stream.write(data)
            stream.flush()
        return len(data)

    def flush(self) -> None:
        for stream in self._streams:
            stream.flush()


class OutputCapture:
    """Temporarily tee stdout/stderr while preserving original streams."""

    def __init__(self, report_path: Optional[Path]) -> None:
        self.report_path = report_path
        self._file: Optional[TextIO] = None
        self._stdout: Optional[TextIO] = None
        self._stderr: Optional[TextIO] = None

    def __enter__(self) -> "OutputCapture":
        if self.report_path is None:
            return self
        self.report_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.report_path.open("w", encoding="utf-8", buffering=1)
        self._stdout = sys.stdout
        self._stderr = sys.stderr
        sys.stdout = Tee(self._stdout, self._file)  # type: ignore[assignment]
        sys.stderr = Tee(self._stderr, self._file)  # type: ignore[assignment]
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        if self._stdout is not None:
            sys.stdout = self._stdout
        if self._stderr is not None:
            sys.stderr = self._stderr
        if self._file is not None:
            self._file.close()


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only inventory of the installed agibot_gdk Python control "
            "surface. No motion/control API is called."
        )
    )
    parser.add_argument(
        "--env-script",
        default=DEFAULT_ENV_SCRIPT,
        help="official GDK environment script (default: %(default)s)",
    )
    parser.add_argument(
        "--no-env-restart",
        action="store_true",
        help="do not restart under the official GDK environment",
    )
    parser.add_argument(
        "--end-state-retries",
        type=int,
        default=10,
        help="maximum read-only get_end_state attempts (default: %(default)s)",
    )
    parser.add_argument(
        "--retry-interval",
        type=float,
        default=1.0,
        help="seconds between get_end_state attempts (default: %(default)s)",
    )
    parser.add_argument(
        "--doc-max-chars",
        type=int,
        default=12000,
        help="maximum __doc__ characters printed per symbol; 0 means unlimited",
    )
    parser.add_argument(
        "--report",
        default=None,
        help="report file path; default is a timestamped file beside the program",
    )
    parser.add_argument(
        "--no-report",
        action="store_true",
        help="print only to the terminal and do not write a report file",
    )
    parser.add_argument(
        "--offline-self-test",
        action="store_true",
        help="test the probe with a fake binding; does not import GDK or touch hardware",
    )
    args = parser.parse_args(argv)
    if args.end_state_retries < 0:
        parser.error("--end-state-retries must be >= 0")
    if args.retry_interval < 0:
        parser.error("--retry-interval must be >= 0")
    if args.doc_max_chars < 0:
        parser.error("--doc-max-chars must be >= 0")
    return args


def ensure_official_environment(args: argparse.Namespace) -> None:
    """Restart once through the vendor env.sh before importing agibot_gdk."""
    if args.offline_self_test or args.no_env_restart:
        return
    if os.environ.get(ENV_RESTART_GUARD) == "1":
        return

    env_script = Path(os.path.expanduser(args.env_script)).resolve()
    if not env_script.is_file():
        print(f"[ENV] Official GDK environment script not found: {env_script}")
        print("[ENV] Continuing without restart; agibot_gdk import may fail.")
        return

    script_path = Path(__file__).resolve()
    command_parts = [shlex.quote(sys.executable), shlex.quote(str(script_path))]
    command_parts.extend(shlex.quote(arg) for arg in sys.argv[1:])
    command = (
        f"source {shlex.quote(str(env_script))} && "
        f"export {ENV_RESTART_GUARD}=1 && "
        f"exec {' '.join(command_parts)}"
    )
    print("[ENV] Restarting with official GDK environment ...", flush=True)
    os.execv("/bin/bash", ["bash", "-lc", command])


def default_report_path() -> Path:
    timestamp = _datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().with_name(
        f"109_g2_o10_control_surface_probe_v2_report_{timestamp}.txt"
    )


def safe_repr(value: Any, max_chars: int = 1200) -> str:
    try:
        text = repr(value)
    except BaseException as exc:  # introspection must not abort the probe
        text = f"<repr failed: {type(exc).__name__}: {exc}>"
    if max_chars > 0 and len(text) > max_chars:
        return text[:max_chars] + f" ... <truncated {len(text) - max_chars} chars>"
    return text


def object_kind(obj: Any) -> str:
    if inspect.ismodule(obj):
        return "module"
    if inspect.isclass(obj):
        return "class"
    if inspect.ismethod(obj):
        return "bound-method"
    if inspect.isbuiltin(obj):
        return "builtin/pybind-callable"
    if inspect.isfunction(obj):
        return "function"
    if callable(obj):
        return "callable"
    return type(obj).__name__


def signature_text(obj: Any) -> str:
    try:
        return str(inspect.signature(obj))
    except BaseException as exc:
        text_signature = None
        try:
            text_signature = getattr(obj, "__text_signature__", None)
        except BaseException:
            pass
        if text_signature:
            return f"{text_signature} [from __text_signature__]"
        return f"<unavailable: {type(exc).__name__}: {exc}>"


def doc_text(obj: Any, max_chars: int) -> str:
    try:
        doc = inspect.getdoc(obj)
    except BaseException as exc:
        return f"<__doc__ read failed: {type(exc).__name__}: {exc}>"
    if not doc:
        return "<none>"
    if max_chars > 0 and len(doc) > max_chars:
        return doc[:max_chars] + f"\n... <truncated {len(doc) - max_chars} chars>"
    return doc


def keyword_match(name: str) -> bool:
    lowered = name.lower()
    return any(keyword in lowered for keyword in SEARCH_KEYWORDS)


def public_names(obj: Any) -> List[str]:
    try:
        return sorted(name for name in dir(obj) if not name.startswith("__"))
    except BaseException:
        return []


def safe_getattr(obj: Any, name: str) -> Tuple[bool, Any]:
    try:
        return True, getattr(obj, name)
    except BaseException as exc:
        return False, f"{type(exc).__name__}: {exc}"


def print_symbol_detail(
    qualified_name: str,
    obj: Any,
    doc_max_chars: int,
    include_value: bool = True,
) -> None:
    print(SUB_SEPARATOR)
    print(f"symbol    : {qualified_name}")
    print(f"kind      : {object_kind(obj)}")
    print(f"py type   : {type(obj).__module__}.{type(obj).__qualname__}")
    print(f"callable  : {callable(obj)}")
    if callable(obj):
        print(f"signature : {signature_text(obj)}")
    if include_value and not inspect.ismodule(obj) and not inspect.isclass(obj):
        print(f"value     : {safe_repr(obj)}")
    print("__doc__   :")
    doc = doc_text(obj, doc_max_chars)
    for line in doc.splitlines() or [doc]:
        print(f"  {line}")


def print_enum_or_constants(qualified_name: str, obj: Any) -> None:
    print(f"\n[{qualified_name} MEMBERS]")
    members = None
    ok, value = safe_getattr(obj, "__members__")
    if ok and isinstance(value, dict):
        members = value
    if members:
        for name in sorted(members):
            print(f"  {name} = {safe_repr(members[name])}")
        return

    found = 0
    for name in public_names(obj):
        if not (name.startswith("k") or name.isupper()):
            continue
        ok, value = safe_getattr(obj, name)
        if ok and not callable(value):
            print(f"  {name} = {safe_repr(value)}")
            found += 1
    if found == 0:
        print("  <no enum/constant members detected>")


def inspect_class_surface(
    qualified_name: str,
    cls: Any,
    doc_max_chars: int,
) -> None:
    print(f"\n[{qualified_name} PUBLIC SURFACE]")
    names = public_names(cls)
    if not names:
        print("  <no public members detected>")
        return
    for name in names:
        ok, value = safe_getattr(cls, name)
        if not ok:
            print(f"  {name}: <getattr failed: {value}>")
            continue
        if callable(value):
            print(f"  {name} [callable] signature={signature_text(value)}")
        else:
            print(f"  {name} [{object_kind(value)}] = {safe_repr(value, 500)}")
    print_symbol_detail(qualified_name, cls, doc_max_chars, include_value=False)


def inspect_safe_data_instance(
    gdk: Any,
    type_name: str,
    doc_max_chars: int,
) -> bool:
    """Construct and inspect one allow-listed pybind data container only."""
    ok, factory = safe_getattr(gdk, type_name)
    if not ok or not callable(factory):
        return False
    print(f"\n[SAFE DATA CONTAINER: {type_name}]")
    try:
        instance = factory()
    except BaseException as exc:
        print(f"  constructor: FAIL ({type(exc).__name__}: {exc})")
        return False
    print("  constructor: PASS (introspection only; no fields changed)")
    fields = 0
    for name in public_names(instance):
        ok, value = safe_getattr(instance, name)
        if not ok:
            print(f"  {name}: <read failed: {value}>")
            continue
        if callable(value):
            continue
        print(f"  {name} [{object_kind(value)}] = {safe_repr(value, 1000)}")
        fields += 1
    if fields == 0:
        print("  <no readable non-callable public fields>")
    print(f"  type signature: {signature_text(factory)}")
    brief_doc = doc_text(factory, doc_max_chars)
    print("  type __doc__:")
    for line in brief_doc.splitlines() or [brief_doc]:
        print(f"    {line}")
    return True


def inventory_module(gdk: Any, doc_max_chars: int) -> Dict[str, Any]:
    print(f"\n{SEPARATOR}\n[AGIBOT_GDK MODULE CONTROL SURFACE]\n{SEPARATOR}")
    names = public_names(gdk)
    matched = [name for name in names if keyword_match(name)]
    focused = [name for name in FOCUSED_MODULE_SYMBOLS if name in names]
    selected = sorted(set(matched + focused))

    print(f"module public symbols : {len(names)}")
    print(f"keyword matches       : {len(matched)}")
    print(f"selected unique       : {len(selected)}")
    print(f"keywords              : {', '.join(SEARCH_KEYWORDS)}")

    records: Dict[str, Any] = {}
    for name in selected:
        ok, value = safe_getattr(gdk, name)
        if not ok:
            print(f"  {name}: <getattr failed: {value}>")
            continue
        records[name] = value
        print(
            f"  {name:<46} kind={object_kind(value):<24} "
            f"callable={str(callable(value)):<5}"
        )

    print("\n[FOCUSED MODULE SYMBOL DETAILS]")
    for name in FOCUSED_MODULE_SYMBOLS:
        if name not in records:
            ok, value = safe_getattr(gdk, name)
            if ok:
                records[name] = value
            else:
                print(f"  {name:<36} PRESENT=NO")
                continue
        print(f"  {name:<36} PRESENT=YES kind={object_kind(records[name])}")
        print_symbol_detail(f"agibot_gdk.{name}", records[name], doc_max_chars)

    print("\n[MATCHED MODULE CALLABLE/CLASS DETAILS]")
    for name in selected:
        value = records.get(name)
        if value is None or not callable(value):
            continue
        if name in FOCUSED_MODULE_SYMBOLS:
            continue
        print_symbol_detail(f"agibot_gdk.{name}", value, doc_max_chars)

    for enum_name in ("EndEffectorControlGroup", "PlanningGroup"):
        if enum_name in records:
            print_enum_or_constants(f"agibot_gdk.{enum_name}", records[enum_name])

    for type_name in SAFE_ZERO_ARG_DATA_TYPES:
        inspect_safe_data_instance(gdk, type_name, doc_max_chars)

    return records


def inventory_robot_class(gdk: Any, doc_max_chars: int) -> Dict[str, Any]:
    print(f"\n{SEPARATOR}\n[ROBOT CLASS CONTROL SURFACE]\n{SEPARATOR}")
    ok, robot_cls = safe_getattr(gdk, "Robot")
    if not ok:
        print(f"agibot_gdk.Robot: NOT FOUND ({robot_cls})")
        return {}

    names = public_names(robot_cls)
    matched = [name for name in names if keyword_match(name)]
    selected = sorted(set(matched).union(FOCUSED_ROBOT_MEMBERS))
    records: Dict[str, Any] = {}

    print(f"Robot public members : {len(names)}")
    print(f"keyword matches      : {len(matched)}")
    print(f"selected names       : {len(selected)}")
    for name in selected:
        if name not in names:
            print(f"  {name:<46} PRESENT=NO")
            continue
        ok, value = safe_getattr(robot_cls, name)
        if not ok:
            print(f"  {name:<46} PRESENT=YES getattr=FAIL ({value})")
            continue
        records[name] = value
        print(
            f"  {name:<46} PRESENT=YES kind={object_kind(value):<22} "
            f"signature={signature_text(value) if callable(value) else '<not callable>'}"
        )

    print("\n[ROBOT CLASS MATCHED MEMBER DETAILS]")
    for name in sorted(records):
        print_symbol_detail(
            f"agibot_gdk.Robot.{name}", records[name], doc_max_chars
        )
    return records


def inventory_robot_instance(
    robot: Any,
    doc_max_chars: int,
) -> Dict[str, Any]:
    print(f"\n{SEPARATOR}\n[ROBOT INSTANCE CONTROL SURFACE]\n{SEPARATOR}")
    names = public_names(robot)
    matched = [name for name in names if keyword_match(name)]
    selected = sorted(set(matched).union(FOCUSED_ROBOT_MEMBERS))
    records: Dict[str, Any] = {}

    print(f"Robot instance public members : {len(names)}")
    print(f"keyword matches               : {len(matched)}")
    print(f"selected names                : {len(selected)}")
    for name in selected:
        if name not in names:
            print(f"  {name:<46} PRESENT=NO")
            continue
        ok, value = safe_getattr(robot, name)
        if not ok:
            print(f"  {name:<46} PRESENT=YES getattr=FAIL ({value})")
            continue
        records[name] = value
        print(
            f"  {name:<46} PRESENT=YES kind={object_kind(value):<22} "
            f"signature={signature_text(value) if callable(value) else '<not callable>'}"
        )

    print("\n[ROBOT INSTANCE MATCHED MEMBER DETAILS]")
    for name in sorted(records):
        print_symbol_detail(f"robot.{name}", records[name], doc_max_chars)
    return records


def find_forbidden_direct_calls(source_path: Path) -> List[Tuple[int, str]]:
    """Find syntactic direct calls to known write/control members."""
    source = source_path.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(source_path))
    violations: List[Tuple[int, str]] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        called_name: Optional[str] = None
        if isinstance(node.func, ast.Attribute):
            called_name = node.func.attr
        elif isinstance(node.func, ast.Name):
            called_name = node.func.id
        if called_name in FORBIDDEN_DIRECT_CALLS:
            violations.append((getattr(node, "lineno", -1), str(called_name)))
    return sorted(violations)


def run_source_safety_audit() -> bool:
    print(f"\n{SEPARATOR}\n[SOURCE SAFETY AUDIT]\n{SEPARATOR}")
    source_path = Path(__file__).resolve()
    try:
        violations = find_forbidden_direct_calls(source_path)
    except BaseException as exc:
        print(f"source path                     : {source_path}")
        print(f"AST parse                       : FAIL ({type(exc).__name__}: {exc})")
        return False
    print(f"source path                     : {source_path}")
    print("AST parse                       : PASS")
    print(f"forbidden direct motion calls   : {len(violations)}")
    if violations:
        for line, name in violations:
            print(f"  line {line}: {name}()")
        print("READ-ONLY SAFETY AUDIT          : FAIL")
        return False
    print("READ-ONLY SAFETY AUDIT          : PASS")
    return True


def normalize_end_side(side_value: Any) -> Tuple[List[str], List[Any], Dict[str, Any]]:
    """Accept dict-style or attribute-style EndState values."""
    if isinstance(side_value, dict):
        getter = side_value.get
    else:
        getter = lambda name, default=None: getattr(side_value, name, default)
    names = list(getter("names", []) or [])
    states = list(getter("end_states", []) or [])
    meta = {
        "controlled": getter("controlled", None),
        "enable": getter("enable", None),
        "status": getter("status", None),
        "type": getter("type", None),
    }
    return names, states, meta


def state_field(state: Any, name: str, default: Any = None) -> Any:
    if isinstance(state, dict):
        return state.get(name, default)
    return getattr(state, name, default)


def print_end_state(end_state: Any) -> None:
    print("\n[CURRENT O10 STATE - READ ONLY]")
    print(f"raw type : {type(end_state).__module__}.{type(end_state).__qualname__}")
    if isinstance(end_state, dict):
        keys = sorted(str(key) for key in end_state.keys())
        print(f"top keys : {keys}")
        side_getter = end_state.get
    else:
        print(f"raw repr : {safe_repr(end_state, 3000)}")
        side_getter = lambda name: getattr(end_state, name, None)

    for side_key in ("left_end_state", "right_end_state"):
        side_value = side_getter(side_key)
        if side_value is None:
            print(f"\n  {side_key}: MISSING")
            continue
        names, states, meta = normalize_end_side(side_value)
        print(f"\n  {side_key}:")
        print(f"    controlled : {safe_repr(meta['controlled'])}")
        print(f"    enable     : {safe_repr(meta['enable'])}")
        print(f"    status     : {safe_repr(meta['status'])}")
        print(f"    type       : {safe_repr(meta['type'])}")
        print(f"    names      : {len(names)}")
        print(f"    end_states : {len(states)}")
        row_count = max(len(names), len(states))
        for index in range(row_count):
            name = names[index] if index < len(names) else f"<unnamed_{index}>"
            state = states[index] if index < len(states) else None
            if state is None:
                print(f"    [{index:02d}] {name}: <state missing>")
                continue
            fields = []
            for field_name in (
                "position",
                "velocity",
                "current",
                "temperature",
                "status",
                "err_code",
            ):
                value = state_field(state, field_name, None)
                if value is not None:
                    fields.append(f"{field_name}={safe_repr(value, 200)}")
            print(f"    [{index:02d}] {name}: {', '.join(fields) or safe_repr(state, 800)}")


def read_end_state_with_retry(
    robot: Any,
    attempts: int,
    interval: float,
) -> Tuple[bool, Any, Optional[str]]:
    print(f"\n{SEPARATOR}\n[READ-ONLY get_end_state() WITH RETRY]\n{SEPARATOR}")
    if attempts == 0:
        print("get_end_state: SKIPPED (--end-state-retries=0)")
        return False, None, "skipped"

    ok, reader = safe_getattr(robot, "get_end_state")
    if not ok or not callable(reader):
        message = f"get_end_state unavailable: {reader}"
        print(message)
        return False, None, message

    last_error: Optional[str] = None
    for attempt in range(1, attempts + 1):
        try:
            end_state = reader()
            print(f"attempt {attempt}/{attempts}: PASS")
            print_end_state(end_state)
            return True, end_state, None
        except BaseException as exc:
            last_error = f"{type(exc).__name__}: {exc}"
            print(f"attempt {attempt}/{attempts}: no state ({last_error})")
            if attempt < attempts and interval > 0:
                time.sleep(interval)
    return False, None, last_error


def print_banner(args: argparse.Namespace, report_path: Optional[Path]) -> None:
    print(SEPARATOR)
    print("G2 O10 Runtime Python-Binding Control Surface Probe")
    print(f"Version             : {PROGRAM_VERSION}")
    print(f"Python              : {sys.version.split()[0]}")
    print(f"Python executable   : {sys.executable}")
    print(f"Platform            : {platform.platform()}")
    print(f"Timestamp           : {_datetime.datetime.now().astimezone().isoformat()}")
    print(f"READ_ONLY_PROBE     : {READ_ONLY_PROBE}")
    print(f"ENABLE_REAL_MOTION  : {ENABLE_REAL_MOTION}")
    print("Motion API calls    : NONE")
    print("Robot API called    : get_end_state() only")
    print(f"End-state retry     : {args.end_state_retries} x {args.retry_interval:.3f}s")
    print(f"Report              : {report_path if report_path else '<disabled>'}")
    print(SEPARATOR)


def build_fake_gdk() -> Any:
    """Create a hardware-free binding facsimile for offline verification."""

    class _GDKRes:
        kSuccess = "GDKRes.kSuccess"

    class JointControlReq:
        """Fake JointControlReq for offline probe validation."""

        def __init__(self) -> None:
            self.life_time = 0.0
            self.joint_names: List[str] = []
            self.joint_positions: List[float] = []
            self.joint_velocities: List[float] = []
            self.detail = ""

    class JointServoControlReq:
        """Fake JointServoControlReq for offline probe validation."""

        def __init__(self) -> None:
            self.control_period = 0.0
            self.joint_names: List[str] = []
            self.joint_positions: List[float] = []
            self.joint_velocities: List[float] = []

    class JointState:
        def __init__(self) -> None:
            self.position = 0.0
            self.velocity = 0.0

    class JointStates:
        def __init__(self) -> None:
            self.group = ""
            self.nums = 0
            self.states: List[Any] = []

    class EndState:
        def __init__(self) -> None:
            self.controlled = False
            self.names: List[str] = []
            self.end_states: List[Any] = []

    class DualEndState:
        def __init__(self) -> None:
            self.left_end_state = EndState()
            self.right_end_state = EndState()

    class EndEffectorPose:
        def __init__(self) -> None:
            self.life_time = 0.0
            self.group = 0
            self.end_effector_joint_names: List[str] = []
            self.end_effector_joint_positions: List[float] = []

    class EndEffectorControlGroup:
        kUnknown = 0
        kLeftArm = 4
        kRightArm = 8
        kBothArms = 12

    class PlanningGroup:
        LEFT_TOOL = "left_tool"
        RIGHT_TOOL = "right_tool"

    class Robot:
        motion_calls = 0

        def get_end_state(self) -> Dict[str, Any]:
            def side(prefix: str) -> Dict[str, Any]:
                return {
                    "controlled": True,
                    "enable": True,
                    "status": 0,
                    "type": "o10_t2",
                    "names": [f"{prefix}_joint_{i}" for i in range(10)],
                    "end_states": [
                        {
                            "position": 0.01 * i,
                            "velocity": 0.0,
                            "current": 0.0,
                            "status": 0,
                            "err_code": 0,
                        }
                        for i in range(10)
                    ],
                }

            return {
                "left_end_state": side("left"),
                "right_end_state": side("right"),
            }

        def joint_control_request(self, request: Any) -> int:
            Robot.motion_calls += 1
            raise AssertionError("offline self-test: motion API must not be called")

        def joint_servo_control(self, request: Any, enable_low_latency: bool = False) -> int:
            Robot.motion_calls += 1
            raise AssertionError("offline self-test: motion API must not be called")

        def move_arm_joint(self, positions: Any, velocities: Any, control_group: int) -> int:
            Robot.motion_calls += 1
            raise AssertionError("offline self-test: motion API must not be called")

        def move_ee_pos(self, joint_states: Any) -> int:
            Robot.motion_calls += 1
            raise AssertionError("offline self-test: motion API must not be called")

        def end_effector_pose_control(self, pose: Any) -> int:
            Robot.motion_calls += 1
            raise AssertionError("offline self-test: motion API must not be called")

        def set_reference_frame_poses(self, poses: Any) -> int:
            Robot.motion_calls += 1
            raise AssertionError("offline self-test: motion API must not be called")

    fake = types.SimpleNamespace(
        __name__="agibot_gdk_fake",
        GDKRes=_GDKRes,
        Robot=Robot,
        JointControlReq=JointControlReq,
        JointServoControlReq=JointServoControlReq,
        JointState=JointState,
        JointStates=JointStates,
        EndState=EndState,
        DualEndState=DualEndState,
        EndEffectorPose=EndEffectorPose,
        EndEffectorControlGroup=EndEffectorControlGroup,
        PlanningGroup=PlanningGroup,
        LEFT_TOOL="left_tool",
        RIGHT_TOOL="right_tool",
        gdk_init=lambda: _GDKRes.kSuccess,
        gdk_release=lambda: _GDKRes.kSuccess,
    )
    return fake


def gdk_success(gdk: Any, result: Any) -> bool:
    try:
        return result == gdk.GDKRes.kSuccess
    except BaseException:
        return False


def execute_probe(args: argparse.Namespace, gdk: Any) -> int:
    safety_ok = run_source_safety_audit()
    if not safety_ok:
        print("\n[ABORT] Source safety audit failed; GDK initialization was not attempted.")
        return 4

    module_records = inventory_module(gdk, args.doc_max_chars)
    class_records = inventory_robot_class(gdk, args.doc_max_chars)

    initialized = False
    robot_created = False
    end_state_ok = False
    release_ok = False
    robot: Any = None
    instance_records: Dict[str, Any] = {}
    fatal_error: Optional[str] = None

    print(f"\n{SEPARATOR}\n[GDK INITIALIZATION]\n{SEPARATOR}")
    try:
        init_result = gdk.gdk_init()
        initialized = gdk_success(gdk, init_result)
        print(f"gdk_init result : {safe_repr(init_result)}")
        print(f"gdk_init status : {'PASS' if initialized else 'FAIL'}")
        if initialized:
            robot = gdk.Robot()
            robot_created = True
            print("Robot() status  : PASS")
            instance_records = inventory_robot_instance(robot, args.doc_max_chars)
            end_state_ok, _, _ = read_end_state_with_retry(
                robot,
                args.end_state_retries,
                args.retry_interval,
            )
    except BaseException as exc:
        fatal_error = f"{type(exc).__name__}: {exc}"
        print(f"\n[ERROR] {fatal_error}")
        traceback.print_exc()
    finally:
        if initialized:
            try:
                release_result = gdk.gdk_release()
                release_ok = gdk_success(gdk, release_result)
                print(f"\n[GDK] gdk_release result : {safe_repr(release_result)}")
            except BaseException as exc:
                print(f"\n[GDK] gdk_release exception: {type(exc).__name__}: {exc}")

    print(f"\n{SEPARATOR}")
    print("109_v2 READ-ONLY CONTROL SURFACE PROBE SUMMARY")
    print(SEPARATOR)
    print(f"source safety audit                 : {'PASS' if safety_ok else 'FAIL'}")
    print(f"GDK initialized                     : {'PASS' if initialized else 'FAIL'}")
    print(f"Robot created                       : {'PASS' if robot_created else 'FAIL'}")
    print(f"get_end_state read                  : {'PASS' if end_state_ok else 'FAIL/SKIP'}")
    print(f"GDK released                        : {'PASS' if release_ok else 'FAIL/SKIP'}")
    print("motion/control API call             : NO")
    print("O10 commanded                       : NO")
    print("Arm commanded                       : NO")
    print("Motion executed                     : NO")
    print("\nFocused Robot member presence (class / instance):")
    for name in FOCUSED_ROBOT_MEMBERS:
        print(
            f"  {name:<38}: "
            f"{'YES' if name in class_records else 'NO':<3} / "
            f"{'YES' if name in instance_records else 'NO':<3}"
        )
    print("\nFocused request/data types:")
    for name in FOCUSED_MODULE_SYMBOLS:
        print(f"  {name:<38}: {'YES' if name in module_records else 'NO'}")
    if fatal_error:
        print(f"\nfatal error: {fatal_error}")

    if args.offline_self_test:
        motion_calls = getattr(gdk.Robot, "motion_calls", -1)
        print(f"\nOFFLINE SELF-TEST motion calls      : {motion_calls}")
        print(
            "OFFLINE SELF-TEST                    : "
            f"{'PASS' if motion_calls == 0 and end_state_ok else 'FAIL'}"
        )
        return 0 if motion_calls == 0 and end_state_ok and safety_ok else 9

    if not initialized:
        return 2
    if not robot_created:
        return 3
    if not end_state_ok and args.end_state_retries > 0:
        return 5
    if not release_ok:
        return 6
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    ensure_official_environment(args)

    if args.no_report:
        report_path = None
    elif args.report:
        report_path = Path(args.report).expanduser().resolve()
    else:
        report_path = default_report_path()

    with OutputCapture(report_path):
        print_banner(args, report_path)
        try:
            if args.offline_self_test:
                gdk = build_fake_gdk()
                print("[MODE] OFFLINE SELF-TEST: no GDK import and no hardware access")
            else:
                gdk = importlib.import_module("agibot_gdk")
                print(f"[IMPORT] agibot_gdk: PASS ({safe_repr(gdk, 500)})")
        except BaseException as exc:
            print(f"[IMPORT] agibot_gdk: FAIL ({type(exc).__name__}: {exc})")
            traceback.print_exc()
            return 1

        exit_code = execute_probe(args, gdk)
        print(f"\nProcess exit code: {exit_code}")
        if report_path is not None:
            print(f"Report written   : {report_path}")
        return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
