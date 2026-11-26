#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
109_g2_o10_end_effector_joint_control_api_probe_v1.py

Read-only capability probe for the G2 GDK 4.1.5 Python binding and O10 end
executor state.

This program intentionally DOES NOT call any motion/control API.  It only:
  1. prepares the official local GDK runtime paths when they are available;
  2. calls gdk_init();
  3. creates Robot();
  4. inspects move_end_effector_joint without invoking it;
  5. lists relevant end-effector/tool symbols and control-group enum members;
  6. calls get_end_state() and prints the current left/right state;
  7. calls gdk_release().

Safety invariants:
  READ_ONLY_PROBE    = True
  ENABLE_REAL_MOTION = False
  O10 commanded      = NO
  Arm commanded      = NO
  Motion executed    = NO
"""

from __future__ import annotations

import inspect
import os
import platform
import sys
import textwrap
import traceback
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any, Optional


VERSION = "109_v1_end_effector_joint_control_api_probe"
READ_ONLY_PROBE = True
ENABLE_REAL_MOTION = False
GDK_ENV_MARKER = "G2_GDK_PROBE_ENV_READY"
SEPARATOR = "=" * 108
SUB_SEPARATOR = "-" * 108
DOCUMENTED_GROUPS = ("left_tool", "right_tool", "dual_tool")

# Names are strings for reporting/static-audit purposes only.  None is invoked.
FORBIDDEN_MOTION_APIS = (
    "move_end_effector_joint",
    "end_effector_pose_control",
    "move_ee_pos",
    "move_arm_joint",
    "joint_control",
    "joint_control_request",
    "joint_servo_control",
)


def _prepend_unique(existing: str, entries: Sequence[str]) -> str:
    """Prepend existing filesystem entries without duplicating path elements."""
    current = [item for item in existing.split(os.pathsep) if item]
    result: list[str] = []
    for item in list(entries) + current:
        if item and item not in result:
            result.append(item)
    return os.pathsep.join(result)


def restart_with_official_gdk_environment() -> None:
    """Re-exec once with the standard per-user AgiBot GDK paths, if present.

    This changes only the process environment.  It does not initialize GDK and
    cannot send a robot command.
    """
    if os.environ.get(GDK_ENV_MARKER) == "1":
        return

    home = Path.home()
    app_root = home / ".cache" / "agibot" / "app"
    lib_root = app_root / "lib"
    gdk_python = app_root / "gdk" / "lib"

    ld_candidates = [
        lib_root,
        lib_root / "engines-3",
        lib_root / "ossl-modules",
        lib_root / "pulse-14.2" / "modules",
        lib_root / "pulseaudio",
        lib_root / f"python{sys.version_info.major}.{sys.version_info.minor}" / "lib-dynload",
    ]
    py_candidates = [gdk_python, Path(__file__).resolve().parent]

    existing_ld = [str(path) for path in ld_candidates if path.is_dir()]
    existing_py = [str(path) for path in py_candidates if path.is_dir()]

    # If no official cache is installed, continue normally so the import error
    # is explicit and easy to diagnose.
    if not existing_ld and not gdk_python.is_dir():
        return

    env = os.environ.copy()
    env[GDK_ENV_MARKER] = "1"
    env["LD_LIBRARY_PATH"] = _prepend_unique(env.get("LD_LIBRARY_PATH", ""), existing_ld)
    env["PYTHONPATH"] = _prepend_unique(env.get("PYTHONPATH", ""), existing_py)

    print("[ENV] Restarting with official GDK environment ...", flush=True)
    print(f"updated LD_LIBRARY_PATH: {env.get('LD_LIBRARY_PATH', '')}", flush=True)
    print(f"updated PYTHONPATH: {env.get('PYTHONPATH', '')}", flush=True)
    os.execve(sys.executable, [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]], env)


def safe_repr(value: Any, limit: int = 500) -> str:
    """Return a bounded representation that cannot break the diagnostic run."""
    try:
        result = repr(value)
    except Exception as exc:  # pragma: no cover - defensive for extension types
        result = f"<repr failed: {type(exc).__name__}: {exc}>"
    if len(result) > limit:
        return result[: limit - 3] + "..."
    return result


def field(value: Any, name: str, default: Any = None) -> Any:
    """Read a documented field from either a dict-like or object-like value."""
    if isinstance(value, Mapping):
        return value.get(name, default)
    try:
        return getattr(value, name)
    except Exception:
        return default


def sequence_or_empty(value: Any) -> list[Any]:
    if value is None or isinstance(value, (str, bytes, bytearray)):
        return []
    try:
        return list(value)
    except Exception:
        return []


def print_header() -> None:
    print(SEPARATOR)
    print("G2 O10 End-Effector Joint-Control API Capability Probe")
    print(f"Version             : {VERSION}")
    print(f"Python              : {sys.version.split()[0]}")
    print(f"Platform            : {platform.platform()}")
    print(f"READ_ONLY_PROBE     : {READ_ONLY_PROBE}")
    print(f"ENABLE_REAL_MOTION  : {ENABLE_REAL_MOTION}")
    print("Motion API          : NONE")
    print(SEPARATOR)


def describe_callable(label: str, target: Any) -> dict[str, Any]:
    """Print metadata for a callable without invoking it."""
    result = {
        "present": target is not None,
        "callable": callable(target) if target is not None else False,
        "signature": None,
        "signature_error": None,
        "text_signature": None,
        "doc_present": False,
    }

    print(f"\n[{label}]")
    if target is None:
        print("present             : NO")
        return result

    print("present             : YES")
    print(f"python type         : {type(target).__module__}.{type(target).__qualname__}")
    print(f"callable            : {'YES' if callable(target) else 'NO'}")

    text_signature = getattr(target, "__text_signature__", None)
    result["text_signature"] = text_signature
    print(f"__text_signature__  : {safe_repr(text_signature)}")

    try:
        signature = str(inspect.signature(target))
        result["signature"] = signature
        print(f"inspect.signature   : {signature}")
    except Exception as exc:
        result["signature_error"] = f"{type(exc).__name__}: {exc}"
        print(f"inspect.signature   : unavailable ({type(exc).__name__}: {exc})")

    doc = inspect.getdoc(target) or ""
    result["doc_present"] = bool(doc)
    print(f"docstring present   : {'YES' if doc else 'NO'}")
    if doc:
        print("docstring:")
        print(textwrap.indent(doc, "  "))
    return result


def public_members(container: Any) -> list[tuple[str, Any]]:
    """Safely collect public attributes without assuming a pybind11 layout."""
    names: set[str] = set()
    try:
        names.update(name for name in dir(container) if not name.startswith("_"))
    except Exception:
        pass
    try:
        names.update(name for name in vars(container) if not name.startswith("_"))
    except Exception:
        pass

    members: list[tuple[str, Any]] = []
    for name in sorted(names):
        try:
            members.append((name, getattr(container, name)))
        except Exception as exc:
            members.append((name, f"<getattr failed: {type(exc).__name__}: {exc}>"))
    return members


def print_enum_members(agibot_gdk: Any) -> bool:
    print("\n[EndEffectorControlGroup]")
    enum_type = getattr(agibot_gdk, "EndEffectorControlGroup", None)
    if enum_type is None:
        print("present             : NO")
        return False

    print("present             : YES")
    members = [
        (name, value)
        for name, value in public_members(enum_type)
        if name.startswith("k")
    ]
    if not members:
        print("public k* members   : NONE DISCOVERED")
    else:
        for name, value in members:
            print(f"  {name:<30} = {safe_repr(value)}")
    return True


def print_related_symbols(agibot_gdk: Any, robot: Any) -> None:
    print("\n[RELATED MODULE SYMBOLS: names containing 'end' or 'tool']")
    module_members = [
        (name, value)
        for name, value in public_members(agibot_gdk)
        if "end" in name.lower() or "tool" in name.lower()
    ]
    if not module_members:
        print("  NONE DISCOVERED")
    for name, value in module_members:
        kind = "callable" if callable(value) else type(value).__name__
        print(f"  {name:<44} [{kind}]")

    print("\n[RELATED ROBOT MEMBERS: names containing 'end' or 'tool']")
    robot_members = [
        (name, value)
        for name, value in public_members(robot)
        if "end" in name.lower() or "tool" in name.lower()
    ]
    if not robot_members:
        print("  NONE DISCOVERED")
    for name, value in robot_members:
        kind = "callable" if callable(value) else type(value).__name__
        print(f"  {name:<44} [{kind}]")


def print_end_side(side: str, side_state: Any) -> dict[str, Any]:
    label = side.upper()
    names = sequence_or_empty(field(side_state, "names", []))
    states = sequence_or_empty(field(side_state, "end_states", []))
    controlled = field(side_state, "controlled", None)
    end_type = field(side_state, "type", None)

    print(f"\n[{label} END STATE]")
    print(f"controlled          : {safe_repr(controlled)}")
    print(f"type                : {safe_repr(end_type)}")
    print(f"joint names count   : {len(names)}")
    print(f"joint states count  : {len(states)}")
    print(f"names               : {safe_repr(names, limit=3000)}")

    columns = (
        "id",
        "enable",
        "position",
        "velocity",
        "effort",
        "current",
        "voltage",
        "temperature",
        "status",
        "err_code",
    )
    for index, joint_state in enumerate(states):
        joint_name = names[index] if index < len(names) else f"<unnamed_{index}>"
        print(SUB_SEPARATOR)
        print(f"  [{index:02d}] {joint_name}")
        for column in columns:
            print(f"       {column:<12}: {safe_repr(field(joint_state, column, None))}")

    all_enabled = bool(states) and all(bool(field(item, "enable", False)) for item in states)
    status_zero = bool(states) and all(field(item, "status", None) == 0 for item in states)
    errors_zero = bool(states) and all(field(item, "err_code", None) == 0 for item in states)
    count_match = len(names) == len(states)

    print(SUB_SEPARATOR)
    print(f"name/state count match : {'YES' if count_match else 'NO'}")
    print(f"all joints enabled     : {'YES' if all_enabled else 'NO'}")
    print(f"all status == 0        : {'YES' if status_zero else 'NO'}")
    print(f"all err_code == 0      : {'YES' if errors_zero else 'NO'}")

    return {
        "controlled": controlled,
        "type": end_type,
        "names_count": len(names),
        "states_count": len(states),
        "count_match": count_match,
        "all_enabled": all_enabled,
        "status_zero": status_zero,
        "errors_zero": errors_zero,
    }


def print_safety_summary(
    init_ok: bool,
    method_ok: bool,
    enum_ok: bool,
    state_ok: bool,
    left_summary: Optional[dict[str, Any]],
    right_summary: Optional[dict[str, Any]],
    release_ok: bool,
) -> None:
    print("\n" + SEPARATOR)
    print("109 READ-ONLY CAPABILITY PROBE SUMMARY")
    print(SEPARATOR)
    print(f"GDK initialized                  : {'PASS' if init_ok else 'FAIL'}")
    print(f"move_end_effector_joint present : {'YES' if method_ok else 'NO'}")
    print(f"EndEffectorControlGroup present : {'YES' if enum_ok else 'NO'}")
    print(f"get_end_state read              : {'PASS' if state_ok else 'FAIL'}")
    if left_summary is not None:
        print(
            "Left end counts                 : "
            f"names={left_summary['names_count']} states={left_summary['states_count']}"
        )
    if right_summary is not None:
        print(
            "Right end counts                : "
            f"names={right_summary['names_count']} states={right_summary['states_count']}"
        )
    print(f"GDK released                     : {'PASS' if release_ok else 'FAIL'}")
    print("move_end_effector_joint() call   : NO")
    print("end_effector_pose_control() call : NO")
    print("move_ee_pos() call               : NO")
    print("O10 commanded                    : NO")
    print("Arm commanded                    : NO")
    print("Motion executed                  : NO")
    print(SEPARATOR)


def main() -> int:
    restart_with_official_gdk_environment()
    print_header()

    init_ok = False
    method_ok = False
    enum_ok = False
    state_ok = False
    release_ok = False
    left_summary: Optional[dict[str, Any]] = None
    right_summary: Optional[dict[str, Any]] = None
    agibot_gdk: Any = None
    robot: Any = None

    try:
        import agibot_gdk as imported_gdk

        agibot_gdk = imported_gdk
        print(f"[IMPORT] agibot_gdk file : {getattr(agibot_gdk, '__file__', '<extension/built-in>')}")
        print(f"[IMPORT] module version  : {safe_repr(getattr(agibot_gdk, '__version__', None))}")

        init_result = agibot_gdk.gdk_init()
        expected_success = getattr(getattr(agibot_gdk, "GDKRes", None), "kSuccess", None)
        init_ok = init_result == expected_success if expected_success is not None else int(init_result) == 0
        print(f"[GDK] gdk_init result   : {init_result}")
        if not init_ok:
            print("[FATAL] gdk_init did not report success; Robot() will not be created.")
            return 2

        robot = agibot_gdk.Robot()
        print(f"[GDK] Robot type        : {type(robot).__module__}.{type(robot).__qualname__}")

        print("\n[DOCUMENTED STRING GROUPS]")
        for group in DOCUMENTED_GROUPS:
            print(f"  {group}")

        # Read attributes only.  No retrieved callable is invoked.
        bound_method = getattr(robot, "move_end_effector_joint", None)
        class_method = getattr(type(robot), "move_end_effector_joint", None)
        bound_details = describe_callable("BOUND robot.move_end_effector_joint", bound_method)
        describe_callable("CLASS Robot.move_end_effector_joint", class_method)
        method_ok = bool(bound_details["present"] and bound_details["callable"])

        enum_ok = print_enum_members(agibot_gdk)
        print_related_symbols(agibot_gdk, robot)

        print("\n[READ-ONLY get_end_state()]")
        end_state = robot.get_end_state()
        print(f"return type         : {type(end_state).__module__}.{type(end_state).__qualname__}")
        print(f"top-level keys      : {safe_repr(list(end_state.keys()) if isinstance(end_state, Mapping) else None)}")

        left_state = field(end_state, "left_end_state", None)
        right_state = field(end_state, "right_end_state", None)
        if left_state is None or right_state is None:
            raise RuntimeError("get_end_state() did not contain both left_end_state and right_end_state")

        left_summary = print_end_side("left", left_state)
        right_summary = print_end_side("right", right_state)
        state_ok = True

    except KeyboardInterrupt:
        print("\n[INTERRUPTED] User requested stop.")
    except Exception as exc:
        print(f"\n[ERROR] {type(exc).__name__}: {exc}")
        traceback.print_exc()
    finally:
        if agibot_gdk is not None and init_ok:
            try:
                release_result = agibot_gdk.gdk_release()
                expected_success = getattr(getattr(agibot_gdk, "GDKRes", None), "kSuccess", None)
                release_ok = (
                    release_result == expected_success
                    if expected_success is not None
                    else int(release_result) == 0
                )
                print(f"\n[GDK] gdk_release result : {release_result}")
            except Exception as exc:
                print(f"\n[GDK] gdk_release failed : {type(exc).__name__}: {exc}")
        print_safety_summary(
            init_ok,
            method_ok,
            enum_ok,
            state_ok,
            left_summary,
            right_summary,
            release_ok,
        )

    # A nonzero exit identifies an incomplete capability result.  It does not
    # indicate that any motion was attempted.
    return 0 if all((init_ok, method_ok, state_ok, release_ok)) else 2


if __name__ == "__main__":
    raise SystemExit(main())
