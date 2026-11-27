#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
111_g2_gdk_config_service_readonly_probe_v1.py

G2 / GDK 4.1.5 configuration-service read-only diagnostic probe.

Purpose
-------
Diagnose the failure chain observed before O10 motion:
    Config request timeout -> Failed to get config parser

Safety contract
---------------
* READ ONLY. This program does not call any robot motion/control/write method.
* It does not publish /gdk/config_request and does not call a ROS service.
* The only Robot methods permitted by the source audit are:
    - get_end_state()
    - get_motion_control_status()
* GDK initialization may register its normal local endpoints. No control payload is
  built and no end-effector, arm, hand, mode, or configuration mutation is issued.
* External commands are restricted to read-only inventory/list/info operations.

Outputs
-------
Two files are written under ./reports by default:
    111_g2_gdk_config_service_readonly_probe_v1_<timestamp>.txt
    111_g2_gdk_config_service_readonly_probe_v1_<timestamp>.json

Recommended run
---------------
    /usr/bin/python3.10 \
      /home/isaac-workstation/g2-project/dual_arm/111_g2_gdk_config_service_readonly_probe_v1.py

Static self-test (does not import GDK):
    /usr/bin/python3.10 111_g2_gdk_config_service_readonly_probe_v1.py --self-test
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import socket
import subprocess
import sys
import time
import traceback
from dataclasses import asdict, dataclass
from datetime import datetime
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

PROGRAM = "111 G2 GDK CONFIG SERVICE READ-ONLY PROBE v1"
VERSION = "111_v1"
READ_ONLY_PROBE = True
ENABLE_REAL_MOTION = False
DEFAULT_GDK_ENV = Path.home() / ".cache/agibot/app/env.sh"
BOOTSTRAP_MARKER = "AGIBOT_GDK_ENV_RESTARTED_FOR_111_V1"
DEFAULT_DISCOVERY_WAIT = 5.0
DEFAULT_COMMAND_TIMEOUT = 8.0
DEFAULT_RETRIES = 5
DEFAULT_RETRY_INTERVAL = 0.5
MAX_CAPTURE_CHARS = 40000
MAX_NATIVE_LOG_LINES = 400
MAX_CONFIG_FILES = 200

CONFIG_REQUEST_TOPIC = "/gdk/config_request"
CONFIG_RESPONSE_TOPIC = "/gdk/config_response"

# These names are data for the AST safety audit. Calls to them are forbidden.
FORBIDDEN_ROBOT_CALLS = frozenset(
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
        "set_load",
        "stop_motion",
        "emergency_stop",
    }
)
ALLOWED_ROBOT_CALLS = frozenset({"get_end_state", "get_motion_control_status"})

SAFE_ENV_PREFIXES = (
    "AORTA_",
    "AGIBOT_",
    "GDK_",
    "ROS_",
    "RMW_",
    "CYCLONEDDS_",
    "FASTRTPS_",
)
SAFE_ENV_EXACT = frozenset(
    {
        "LD_LIBRARY_PATH",
        "PYTHONPATH",
        "PATH",
        "HOSTNAME",
        "USER",
        "HOME",
    }
)

KNOWN_CONFIG_PATHS = (
    Path("/home/agi/app/conf/aorta/function/aorta_transport_conf.json"),
    Path("/home/agi/app/conf/scene_server/dylog_sud_default.json"),
    Path.home() / ".cache/agibot/app/env.sh",
)
CONFIG_SCAN_ROOTS = (
    Path("/home/agi/app/conf"),
    Path.home() / ".cache/agibot/app/conf",
    Path.home() / ".cache/agibot/app/gdk",
)
CONFIG_NAME_HINTS = ("config", "conf", "aorta", "o10", "tool", "end_effector")


@dataclass
class Check:
    name: str
    passed: bool
    detail: str
    blocking: bool = False


class Reporter:
    def __init__(self) -> None:
        self.lines: List[str] = []

    def emit(self, text: str = "") -> None:
        value = str(text)
        self.lines.append(value)
        print(value, flush=True)

    def section(self, title: str) -> None:
        self.emit("=" * 112)
        self.emit(title)
        self.emit("=" * 112)

    def write(self, path: Path) -> None:
        path.write_text("\n".join(self.lines) + "\n", encoding="utf-8")


def short_exception(exc: BaseException) -> str:
    return f"{type(exc).__name__}: {exc}"


def sha256_file(path: Path, max_bytes: Optional[int] = None) -> Optional[str]:
    try:
        if not path.is_file():
            return None
        if max_bytes is not None and path.stat().st_size > max_bytes:
            return None
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            while True:
                chunk = stream.read(1024 * 1024)
                if not chunk:
                    break
                digest.update(chunk)
        return digest.hexdigest()
    except Exception:
        return None


def serial(value: Any, depth: int = 0) -> Any:
    if depth > 5:
        return repr(value)
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Mapping):
        return {str(k): serial(v, depth + 1) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [serial(v, depth + 1) for v in value]
    attrs: Dict[str, Any] = {}
    for name in dir(value):
        if name.startswith("_"):
            continue
        try:
            item = getattr(value, name)
            if callable(item):
                continue
            attrs[name] = serial(item, depth + 1)
        except Exception:
            continue
    if attrs:
        return {"type": f"{type(value).__module__}.{type(value).__qualname__}", "attributes": attrs}
    try:
        return repr(value)
    except Exception:
        return f"<{type(value).__name__}>"


def source_safety_audit(path: Path) -> Tuple[List[Check], Dict[str, Any]]:
    checks: List[Check] = []
    details: Dict[str, Any] = {"path": str(path)}
    data = path.read_bytes()
    details["sha256"] = hashlib.sha256(data).hexdigest()
    source = data.decode("utf-8")
    try:
        tree = ast.parse(source, filename=str(path))
        checks.append(Check("source parses as Python", True, f"AST nodes={sum(1 for _ in ast.walk(tree))}", True))
    except Exception as exc:
        checks.append(Check("source parses as Python", False, short_exception(exc), True))
        return checks, details

    calls: List[str] = []
    robot_methods: List[str] = []
    forbidden_found: List[str] = []
    dangerous_subprocess_literals: List[str] = []
    for node in ast.walk(tree):
        # Audit Robot method references as well as direct calls. The two approved
        # read methods are passed as bound callables to read_with_retry().
        if (
            isinstance(node, ast.Attribute)
            and isinstance(node.value, ast.Name)
            and node.value.id == "robot"
        ):
            robot_methods.append(node.attr)
            if node.attr in FORBIDDEN_ROBOT_CALLS:
                forbidden_found.append(node.attr)
        if not isinstance(node, ast.Call):
            continue
        name = ""
        if isinstance(node.func, ast.Attribute):
            name = node.func.attr
        elif isinstance(node.func, ast.Name):
            name = node.func.id
        if name:
            calls.append(name)
        if name in FORBIDDEN_ROBOT_CALLS:
            forbidden_found.append(name)
        if name in {"run", "Popen", "call", "check_call", "check_output"}:
            for arg in node.args:
                if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                    text = arg.value.lower()
                    if any(token in text for token in (" topic pub", " service call", " param set", " kill ")):
                        dangerous_subprocess_literals.append(arg.value)

    disallowed_robot = sorted(set(robot_methods) - ALLOWED_ROBOT_CALLS)
    checks.append(
        Check(
            "no forbidden control/mutation call",
            not forbidden_found,
            "none" if not forbidden_found else repr(sorted(set(forbidden_found))),
            True,
        )
    )
    checks.append(
        Check(
            "Robot method allow-list",
            not disallowed_robot,
            f"calls={sorted(set(robot_methods))}; allowed={sorted(ALLOWED_ROBOT_CALLS)}",
            True,
        )
    )
    checks.append(
        Check(
            "no dangerous subprocess literal",
            not dangerous_subprocess_literals,
            "none" if not dangerous_subprocess_literals else repr(dangerous_subprocess_literals),
            True,
        )
    )
    details["all_call_names"] = sorted(set(calls))
    details["robot_methods"] = sorted(set(robot_methods))
    return checks, details


def ensure_official_environment(args: argparse.Namespace, reporter: Reporter) -> None:
    if args.self_test or args.skip_env_bootstrap:
        return
    if os.environ.get(BOOTSTRAP_MARKER) == "1":
        return
    env_script = Path(args.gdk_env).expanduser().resolve()
    if not env_script.is_file():
        reporter.emit(f"[ENV] WARNING: official env script not found: {env_script}")
        return
    reporter.emit(f"[ENV] Restarting once with official GDK environment: {env_script}")
    command = (
        '_env_script="$1"; shift; '
        '_restart=("$@"); '
        '_source_without_args() { source "$_env_script"; }; '
        f'_source_without_args && export {BOOTSTRAP_MARKER}=1 && exec "${{_restart[@]}}"'
    )
    argv = [
        "/bin/bash",
        "-lc",
        command,
        "111-bootstrap",
        str(env_script),
        sys.executable,
        str(Path(__file__).resolve()),
        *sys.argv[1:],
    ]
    sys.stdout.flush()
    sys.stderr.flush()
    os.execv("/bin/bash", argv)
    raise AssertionError("os.execv returned unexpectedly")


def run_command(command: Sequence[str], timeout: float) -> Dict[str, Any]:
    started = time.monotonic()
    result: Dict[str, Any] = {
        "argv": list(command),
        "available": bool(command and shutil.which(command[0])),
        "returncode": None,
        "stdout": "",
        "stderr": "",
        "duration_seconds": None,
        "timed_out": False,
    }
    if not result["available"]:
        result["stderr"] = f"command not found: {command[0] if command else '<empty>'}"
        return result
    try:
        completed = subprocess.run(
            list(command),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            errors="replace",
            timeout=timeout,
            check=False,
            shell=False,
        )
        result["returncode"] = completed.returncode
        result["stdout"] = completed.stdout[:MAX_CAPTURE_CHARS]
        result["stderr"] = completed.stderr[:MAX_CAPTURE_CHARS]
    except subprocess.TimeoutExpired as exc:
        result["timed_out"] = True
        result["stderr"] = f"TimeoutExpired after {timeout}s: {exc}"
    except Exception as exc:
        result["stderr"] = short_exception(exc)
    result["duration_seconds"] = round(time.monotonic() - started, 6)
    return result


def safe_environment() -> Dict[str, str]:
    output: Dict[str, str] = {}
    for key, value in sorted(os.environ.items()):
        if key in SAFE_ENV_EXACT or key.startswith(SAFE_ENV_PREFIXES):
            output[key] = value
    return output


def collect_host_inventory(timeout: float) -> Dict[str, Any]:
    commands = {
        "ip_address": ["ip", "-json", "address", "show"],
        "ip_route": ["ip", "route", "show"],
        "tcp_udp_sockets": ["ss", "-tunap"],
        "processes": ["ps", "-eo", "pid,ppid,user,lstart,cmd", "--sort=pid"],
    }
    return {
        "hostname": socket.gethostname(),
        "platform": platform.platform(),
        "python_executable": sys.executable,
        "python_version": sys.version,
        "cwd": str(Path.cwd()),
        "script": str(Path(__file__).resolve()),
        "environment": safe_environment(),
        "commands": {name: run_command(argv, timeout) for name, argv in commands.items()},
    }


def collect_ros2_inventory(timeout: float) -> Dict[str, Any]:
    commands = {
        "topic_list": ["ros2", "topic", "list", "-t"],
        "node_list": ["ros2", "node", "list"],
        "service_list": ["ros2", "service", "list", "-t"],
        "config_request_info": ["ros2", "topic", "info", "-v", CONFIG_REQUEST_TOPIC],
        "config_response_info": ["ros2", "topic", "info", "-v", CONFIG_RESPONSE_TOPIC],
    }
    return {name: run_command(argv, timeout) for name, argv in commands.items()}


def file_record(path: Path) -> Dict[str, Any]:
    record: Dict[str, Any] = {"path": str(path), "exists": path.exists(), "is_file": path.is_file()}
    if path.exists():
        try:
            stat = path.stat()
            record.update(
                {
                    "size": stat.st_size,
                    "mtime": datetime.fromtimestamp(stat.st_mtime).astimezone().isoformat(),
                    "sha256": sha256_file(path, max_bytes=4 * 1024 * 1024),
                }
            )
        except Exception as exc:
            record["error"] = short_exception(exc)
    return record


def collect_config_inventory() -> Dict[str, Any]:
    known = [file_record(path) for path in KNOWN_CONFIG_PATHS]
    discovered: List[Dict[str, Any]] = []
    seen: set[str] = set()
    for root in CONFIG_SCAN_ROOTS:
        if not root.exists():
            continue
        try:
            for current, dirs, files in os.walk(root):
                rel_depth = len(Path(current).relative_to(root).parts)
                if rel_depth >= 6:
                    dirs[:] = []
                for name in files:
                    lower = name.lower()
                    if not any(hint in lower for hint in CONFIG_NAME_HINTS):
                        continue
                    path = Path(current) / name
                    key = str(path)
                    if key in seen:
                        continue
                    seen.add(key)
                    discovered.append(file_record(path))
                    if len(discovered) >= MAX_CONFIG_FILES:
                        return {"known_paths": known, "discovered": discovered, "truncated": True}
        except Exception as exc:
            discovered.append({"root": str(root), "error": short_exception(exc)})
    return {"known_paths": known, "discovered": discovered, "truncated": False}


def discover_native_logs(roots: Iterable[Path]) -> Dict[str, Dict[str, Any]]:
    records: Dict[str, Dict[str, Any]] = {}
    for root in roots:
        try:
            resolved = root.resolve()
        except Exception:
            resolved = root
        if not resolved.is_dir():
            continue
        for pattern in ("python*.log.INFO.*", "python*.INFO", "*.log.INFO.*"):
            try:
                paths = resolved.glob(pattern)
                for path in paths:
                    try:
                        if not path.is_file():
                            continue
                        stat = path.stat()
                        records[str(path.resolve())] = {"size": stat.st_size, "mtime_ns": stat.st_mtime_ns}
                    except Exception:
                        continue
            except Exception:
                continue
    return records


def select_current_native_log(before: Mapping[str, Any], after: Mapping[str, Any]) -> Optional[Path]:
    changed: List[Tuple[int, str]] = []
    for path, current in after.items():
        previous = before.get(path)
        if previous is None or previous.get("size") != current.get("size") or previous.get("mtime_ns") != current.get("mtime_ns"):
            changed.append((int(current.get("mtime_ns", 0)), path))
    if not changed:
        changed = [(int(meta.get("mtime_ns", 0)), path) for path, meta in after.items()]
    if not changed:
        return None
    changed.sort(reverse=True)
    return Path(changed[0][1])


def bounded_matching_lines(path: Path) -> List[str]:
    # Preserve every config-specific/error line even when discovery is very noisy.
    # Generic peer-connect lines are sampled only to prove that other channels work.
    critical_patterns = (
        CONFIG_REQUEST_TOPIC,
        CONFIG_RESPONSE_TOPIC,
        "Config request timeout",
        "Failed to get config parser",
        "network is ready",
        "Failed to open the file",
        "No end state message found",
    )
    generic_patterns = (
        "subscriber connect to publisher success",
        "Receive valid sub attr",
    )
    critical: List[Tuple[int, str]] = []
    generic: List[Tuple[int, str]] = []
    try:
        with path.open("r", encoding="utf-8", errors="replace") as stream:
            for number, line in enumerate(stream, 1):
                value = line.rstrip()
                if any(pattern in value for pattern in critical_patterns):
                    critical.append((number, value))
                elif len(generic) < 100 and any(pattern in value for pattern in generic_patterns):
                    generic.append((number, value))
    except Exception as exc:
        return [f"ERROR: {short_exception(exc)}"]
    merged = sorted(critical + generic, key=lambda item: item[0])
    if len(merged) > MAX_NATIVE_LOG_LINES:
        # Config-specific evidence has priority over generic connectivity samples.
        kept = critical[:MAX_NATIVE_LOG_LINES]
        room = max(0, MAX_NATIVE_LOG_LINES - len(kept))
        if room:
            kept.extend(generic[:room])
        merged = sorted(kept, key=lambda item: item[0])
    return [f"{number}:{line}" for number, line in merged]


def parse_config_topology(lines: Sequence[str]) -> Dict[str, Any]:
    text = "\n".join(lines)
    local_request = bool(
        re.search(r"create publisher.*?/gdk/config_request|create publisher:\s*/gdk/config_request", text, re.I)
    )
    local_response = bool(
        re.search(r"create subscriber.*?/gdk/config_response|create subscriber:\s*/gdk/config_response", text, re.I)
    )
    # A local request publisher logs this when a remote subscriber is matched.
    remote_request_subscriber = bool(
        re.search(r"Receive valid sub attr[^\n]*topic\s*:\s*/gdk/config_request", text, re.I)
    )
    # A local response subscriber logs this when it connects to a remote publisher.
    remote_response_publisher = bool(
        re.search(r"subscriber connect to publisher success[^\n]*topic\s+/gdk/config_response", text, re.I)
    )
    timeout_count = len(re.findall(r"Config request timeout", text, re.I))
    parser_failure_count = len(re.findall(r"Failed to get config parser", text, re.I))
    other_remote_connections = len(
        re.findall(r"subscriber connect to publisher success(?![^\n]*/gdk/config_response)", text, re.I)
    )
    if local_request and local_response and remote_request_subscriber and remote_response_publisher:
        verdict = "CONFIG_PEERS_DISCOVERED"
    elif local_request and local_response and not remote_request_subscriber and not remote_response_publisher:
        verdict = "CONFIG_SERVICE_NOT_DISCOVERED"
    elif local_request and local_response:
        verdict = "PARTIAL_CONFIG_TOPOLOGY"
    else:
        verdict = "LOCAL_CONFIG_ENDPOINTS_NOT_CONFIRMED"
    return {
        "local_request_publisher_created": local_request,
        "local_response_subscriber_created": local_response,
        "remote_request_subscriber_seen": remote_request_subscriber,
        "remote_response_publisher_seen": remote_response_publisher,
        "config_request_timeout_count": timeout_count,
        "config_parser_failure_count": parser_failure_count,
        "other_remote_subscriber_connections": other_remote_connections,
        "verdict": verdict,
        "interpretation_boundary": (
            "Peer discovery proves topology only. This probe intentionally sends no config request, "
            "so it cannot prove request/response semantics."
        ),
    }


def result_is_success(gdk: Any, value: Any) -> bool:
    try:
        if value == gdk.GDKRes.kSuccess:
            return True
    except Exception:
        pass
    try:
        return int(value) == 0
    except Exception:
        return False


def read_with_retry(method: Any, retries: int, interval: float) -> Dict[str, Any]:
    attempts: List[Dict[str, Any]] = []
    for attempt in range(1, retries + 1):
        try:
            value = method()
            return {"status": "PASS", "attempt": attempt, "attempts": attempts, "value": serial(value)}
        except Exception as exc:
            attempts.append({"attempt": attempt, "error": short_exception(exc)})
            if attempt < retries:
                time.sleep(interval)
    return {"status": "FAILED", "attempt": retries, "attempts": attempts, "value": None}


def gdk_package_inventory(module: Any) -> Dict[str, Any]:
    module_path = Path(getattr(module, "__file__", "") or "")
    result: Dict[str, Any] = {
        "module_file": str(module_path),
        "module_sha256": sha256_file(module_path),
        "distribution_versions": {},
        "native_libraries": [],
    }
    for distribution in ("agibot-gdk", "agibot_gdk"):
        try:
            result["distribution_versions"][distribution] = importlib.metadata.version(distribution)
        except Exception as exc:
            result["distribution_versions"][distribution] = f"NOT_AVAILABLE: {short_exception(exc)}"
    parent = module_path.parent if module_path.is_file() else Path()
    if parent.is_dir():
        libraries: List[Dict[str, Any]] = []
        try:
            for path in sorted(parent.rglob("*.so*"))[:80]:
                libraries.append(file_record(path))
        except Exception as exc:
            libraries.append({"error": short_exception(exc)})
        result["native_libraries"] = libraries
    return result


def run_gdk_readonly_session(args: argparse.Namespace, reporter: Reporter) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "import": "NOT_RUN",
        "initialized": False,
        "robot_created": False,
        "get_end_state": {"status": "NOT_RUN"},
        "get_motion_control_status": {"status": "NOT_RUN"},
        "released": False,
        "errors": [],
    }
    gdk: Any = None
    initialized = False
    try:
        gdk = importlib.import_module("agibot_gdk")
        result["import"] = "PASS"
        result["package"] = gdk_package_inventory(gdk)
        init_result = gdk.gdk_init()
        initialized = result_is_success(gdk, init_result)
        result["initialized"] = initialized
        result["init_result"] = repr(init_result)
        reporter.emit(f"GDK initialized : {'PASS' if initialized else 'FAIL'} ({init_result!r})")
        if not initialized:
            raise RuntimeError(f"gdk_init returned {init_result!r}")
        robot = gdk.Robot()
        result["robot_created"] = True
        result["robot_type"] = f"{type(robot).__module__}.{type(robot).__qualname__}"
        reporter.emit(f"Robot created   : PASS ({result['robot_type']})")
        time.sleep(args.discovery_wait)
        result["get_end_state"] = read_with_retry(
            robot.get_end_state, args.retries, args.retry_interval
        )
        result["get_motion_control_status"] = read_with_retry(
            robot.get_motion_control_status, args.retries, args.retry_interval
        )
    except Exception as exc:
        result["errors"].append(short_exception(exc))
        result["traceback"] = traceback.format_exc()
        reporter.emit(f"GDK session error: {short_exception(exc)}")
    finally:
        if initialized and gdk is not None:
            try:
                release_result = gdk.gdk_release()
                result["released"] = result_is_success(gdk, release_result)
                result["release_result"] = repr(release_result)
                reporter.emit(
                    f"GDK released    : {'PASS' if result['released'] else 'FAIL'} ({release_result!r})"
                )
            except Exception as exc:
                result["errors"].append("gdk_release: " + short_exception(exc))
    return result


def emit_check_table(checks: Sequence[Check], reporter: Reporter) -> None:
    for check in checks:
        label = "PASS" if check.passed else "FAIL"
        boundary = "BLOCKING" if check.blocking else "INFO"
        reporter.emit(f"{label:4} | {boundary:8} | {check.name} | {check.detail}")


def self_test(reporter: Reporter) -> bool:
    reporter.section("111 SELF-TEST")
    checks, _ = source_safety_audit(Path(__file__).resolve())
    synthetic = [
        "1:[GDK]create publisher: /gdk/config_request type: CommonRequest",
        "2:create subscriber. topic name:/gdk/config_response",
        "3:subscriber connect to publisher success, topic /hal/joint_state, local 10.0.0.2, peer 10.0.0.1",
    ]
    parsed_missing = parse_config_topology(synthetic)
    synthetic.extend(
        [
            "4:Receive valid sub attr. topic:/gdk/config_request, local 10.0.0.2, peer 10.0.0.1",
            "5:subscriber connect to publisher success, topic /gdk/config_response, local 10.0.0.2, peer 10.0.0.1",
        ]
    )
    parsed_complete = parse_config_topology(synthetic)
    checks.extend(
        [
            Check(
                "synthetic missing-peer classification",
                parsed_missing["verdict"] == "CONFIG_SERVICE_NOT_DISCOVERED",
                parsed_missing["verdict"],
                True,
            ),
            Check(
                "synthetic complete-peer classification",
                parsed_complete["verdict"] == "CONFIG_PEERS_DISCOVERED",
                parsed_complete["verdict"],
                True,
            ),
            Check(
                "safety constants",
                READ_ONLY_PROBE and not ENABLE_REAL_MOTION,
                f"READ_ONLY_PROBE={READ_ONLY_PROBE}; ENABLE_REAL_MOTION={ENABLE_REAL_MOTION}",
                True,
            ),
        ]
    )
    emit_check_table(checks, reporter)
    success = all(check.passed for check in checks if check.blocking)
    reporter.emit(f"SELF-TEST VERDICT: {'PASS' if success else 'FAIL'}")
    return success


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=PROGRAM)
    parser.add_argument("--self-test", action="store_true", help="run static/in-memory tests only")
    parser.add_argument("--skip-env-bootstrap", action="store_true", help="do not source the official GDK env")
    parser.add_argument("--gdk-env", default=str(DEFAULT_GDK_ENV), help="official GDK env.sh path")
    parser.add_argument("--discovery-wait", type=float, default=DEFAULT_DISCOVERY_WAIT)
    parser.add_argument("--command-timeout", type=float, default=DEFAULT_COMMAND_TIMEOUT)
    parser.add_argument("--retries", type=int, default=DEFAULT_RETRIES)
    parser.add_argument("--retry-interval", type=float, default=DEFAULT_RETRY_INTERVAL)
    parser.add_argument("--report-dir", default=str(Path(__file__).resolve().parent / "reports"))
    parser.add_argument("--no-ros2", action="store_true", help="skip ros2 list/info commands")
    args = parser.parse_args(argv)
    if not 0.0 <= args.discovery_wait <= 30.0:
        parser.error("--discovery-wait must be in [0, 30]")
    if not 1.0 <= args.command_timeout <= 60.0:
        parser.error("--command-timeout must be in [1, 60]")
    if not 1 <= args.retries <= 20:
        parser.error("--retries must be in [1, 20]")
    if not 0.0 <= args.retry_interval <= 5.0:
        parser.error("--retry-interval must be in [0, 5]")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    reporter = Reporter()
    if args.self_test:
        return 0 if self_test(reporter) else 1

    reporter.section(PROGRAM)
    reporter.emit(f"timestamp           : {datetime.now().astimezone().isoformat()}")
    reporter.emit(f"script              : {Path(__file__).resolve()}")
    reporter.emit(f"python              : {sys.version.splitlines()[0]}")
    reporter.emit(f"READ_ONLY_PROBE     : {READ_ONLY_PROBE}")
    reporter.emit(f"ENABLE_REAL_MOTION  : {ENABLE_REAL_MOTION}")
    reporter.emit("control payloads    : NONE")
    reporter.emit("config requests     : NONE")
    reporter.emit("ROS service calls   : NONE")

    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION:
        reporter.emit("FATAL: safety constants invalid")
        return 90

    ensure_official_environment(args, reporter)

    reporter.section("[SOURCE READ-ONLY SAFETY AUDIT]")
    checks, source_details = source_safety_audit(Path(__file__).resolve())
    emit_check_table(checks, reporter)
    if not all(check.passed for check in checks if check.blocking):
        reporter.emit("FATAL: source safety audit failed; GDK not imported")
        return 91

    report: Dict[str, Any] = {
        "program": PROGRAM,
        "version": VERSION,
        "timestamp": datetime.now().astimezone().isoformat(),
        "safety": {
            "read_only_probe": READ_ONLY_PROBE,
            "enable_real_motion": ENABLE_REAL_MOTION,
            "control_payload_built": False,
            "config_request_sent": False,
            "ros_service_called": False,
            "source_audit": [asdict(check) for check in checks],
            "source": source_details,
        },
    }

    reporter.section("[HOST / NETWORK / PROCESS INVENTORY]")
    report["host"] = collect_host_inventory(args.command_timeout)
    reporter.emit(f"hostname            : {report['host']['hostname']}")
    reporter.emit(f"platform            : {report['host']['platform']}")
    reporter.emit(f"official env marker : {os.environ.get(BOOTSTRAP_MARKER, 'NOT_SET')}")
    for name, record in report["host"]["commands"].items():
        reporter.emit(
            f"{name:<20}: available={record['available']} rc={record['returncode']} "
            f"timeout={record['timed_out']}"
        )

    reporter.section("[LOCAL CONFIGURATION INVENTORY]")
    report["configuration"] = collect_config_inventory()
    for record in report["configuration"]["known_paths"]:
        reporter.emit(
            f"{'FOUND' if record['exists'] else 'MISSING':7} | {record['path']} | "
            f"size={record.get('size')} sha256={record.get('sha256')}"
        )
    reporter.emit(
        f"candidate config files: {len(report['configuration']['discovered'])}; "
        f"truncated={report['configuration']['truncated']}"
    )

    if not args.no_ros2:
        reporter.section("[ROS2 READ-ONLY GRAPH INVENTORY]")
        report["ros2"] = collect_ros2_inventory(args.command_timeout)
        for name, record in report["ros2"].items():
            reporter.emit(
                f"{name:<24}: available={record['available']} rc={record['returncode']} "
                f"timeout={record['timed_out']}"
            )
    else:
        report["ros2"] = {"status": "SKIPPED_BY_ARGUMENT"}

    roots = {Path.cwd(), Path(__file__).resolve().parent}
    native_before = discover_native_logs(roots)

    reporter.section("[LIVE GDK READ-ONLY SESSION]")
    report["gdk"] = run_gdk_readonly_session(args, reporter)
    reporter.emit(f"get_end_state              : {report['gdk']['get_end_state'].get('status')}")
    reporter.emit(
        f"get_motion_control_status  : {report['gdk']['get_motion_control_status'].get('status')}"
    )

    # Give the native logger a short opportunity to flush discovery events.
    time.sleep(0.5)
    native_after = discover_native_logs(roots)
    current_log = select_current_native_log(native_before, native_after)
    reporter.section("[AORTA / GDK CONFIG TOPOLOGY FROM NATIVE LOG]")
    if current_log is None:
        lines: List[str] = []
        topology = {
            "verdict": "NATIVE_LOG_NOT_FOUND",
            "interpretation_boundary": "No current native log was discovered in cwd or script directory.",
        }
        reporter.emit("native log: NOT FOUND")
    else:
        lines = bounded_matching_lines(current_log)
        topology = parse_config_topology(lines)
        reporter.emit(f"native log                    : {current_log}")
        reporter.emit(f"matching evidence lines       : {len(lines)}")
        reporter.emit(
            f"local request publisher       : {topology['local_request_publisher_created']}"
        )
        reporter.emit(
            f"local response subscriber     : {topology['local_response_subscriber_created']}"
        )
        reporter.emit(
            f"remote request subscriber     : {topology['remote_request_subscriber_seen']}"
        )
        reporter.emit(
            f"remote response publisher     : {topology['remote_response_publisher_seen']}"
        )
        reporter.emit(f"config topology verdict       : {topology['verdict']}")
    report["native_log"] = {
        "path": str(current_log) if current_log else None,
        "evidence_lines": lines,
        "topology": topology,
    }

    blocking_ok = (
        report["gdk"].get("initialized")
        and report["gdk"].get("robot_created")
        and report["gdk"].get("released")
    )
    read_path_ok = (
        report["gdk"]["get_end_state"].get("status") == "PASS"
        and report["gdk"]["get_motion_control_status"].get("status") == "PASS"
    )
    topology_verdict = topology.get("verdict", "UNKNOWN")

    reporter.section("111 DIAGNOSTIC SUMMARY")
    reporter.emit(f"source read-only audit         : PASS")
    reporter.emit(f"GDK session lifecycle         : {'PASS' if blocking_ok else 'FAIL'}")
    reporter.emit(f"GDK read paths                : {'PASS' if read_path_ok else 'CHECK REQUIRED'}")
    reporter.emit(f"config topology               : {topology_verdict}")
    reporter.emit("control/motion method called  : NO")
    reporter.emit("configuration request sent    : NO")
    reporter.emit("motion executed               : NO")
    if topology_verdict == "CONFIG_SERVICE_NOT_DISCOVERED":
        final_verdict = (
            "CONFIG SERVICE PEERS NOT DISCOVERED - inspect robot-side config service, "
            "AORTA domain/interface/QoS, and workstation/robot version compatibility."
        )
    elif topology_verdict == "PARTIAL_CONFIG_TOPOLOGY":
        final_verdict = "PARTIAL CONFIG TOPOLOGY - identify the missing request or response peer."
    elif topology_verdict == "CONFIG_PEERS_DISCOVERED":
        final_verdict = (
            "CONFIG PEERS DISCOVERED - topology exists; compare versions and obtain vendor-approved "
            "request-level diagnostics before any motion retry."
        )
    else:
        final_verdict = "INCOMPLETE TOPOLOGY EVIDENCE - inspect JSON evidence and native-log location."
    reporter.emit(f"Final verdict: {final_verdict}")

    report["summary"] = {
        "gdk_session_lifecycle_ok": bool(blocking_ok),
        "gdk_read_paths_ok": bool(read_path_ok),
        "config_topology_verdict": topology_verdict,
        "final_verdict": final_verdict,
        "motion_executed": False,
    }

    report_dir = Path(args.report_dir).expanduser().resolve()
    report_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = f"111_g2_gdk_config_service_readonly_probe_v1_{stamp}"
    txt_path = report_dir / f"{stem}.txt"
    json_path = report_dir / f"{stem}.json"
    reporter.emit(f"Text report : {txt_path}")
    reporter.emit(f"JSON report : {json_path}")
    reporter.write(txt_path)
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    # A diagnosed missing/partial service is a successful probe outcome. Non-zero is
    # reserved for failures to run the read-only diagnostic lifecycle itself.
    return 0 if blocking_ok else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[STOP] User interrupted. No control/config request was sent.")
        raise SystemExit(130)
