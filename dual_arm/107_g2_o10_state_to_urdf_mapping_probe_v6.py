#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
107_g2_o10_state_to_urdf_mapping_probe_v6.py

G2 + OmniHand O10 End-State -> URDF mapping diagnostic probe v6.

Purpose:
- Read O10 10+10 DOF state with Robot.get_end_state().
- Parse active, passive and mimic joints from the latest G2 O10 URDF.
- Compare offline URDF FK with live TF progressively along every finger chain.
- Test bounded diagnostic mapping hypotheses without moving the robot:
    A: q_urdf =  q_gdk
    B: q_urdf = -q_gdk
    C: q_urdf =  q_gdk + zero_offset
    D: q_urdf = -q_gdk + zero_offset
- Report the first link where each finger chain materially diverges.
- Never solve or command a grasp while the mapping remains unverified.

Safety:
- Read-only introspection and mathematics only.
- No O10, arm, TCP, wheel or base command.
"""
from __future__ import annotations

import json
import math
import os
import shlex
import sys
import time
import traceback
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np

READ_ONLY_PROBE = True
ENABLE_REAL_MOTION = False
VERSION = "107_v6_state_to_urdf_mapping_probe"
ENV_SH = Path.home() / ".cache/agibot/app/env.sh"
BASE_DIR = Path(__file__).resolve().parent
REPORT_DIR = BASE_DIR / "reports"
URDF_NAMES = ("G2_t2_crs_o10_t2.urdf", "G2_t2_crs_o10_t2.urdf.txt")
WAIT_SEC = 2.0
PASS_TOLERANCE_M = 0.003
MATERIAL_DIVERGENCE_M = 0.005
TABLE_TOP_Z_BASE_M = 0.760

ACTIVE = {
    "left": [
        "idx31_hand_l_thumb_roll_joint", "idx32_hand_l_thumb_abad_joint",
        "idx33_hand_l_thumb_mcp_joint", "idx36_hand_l_index_abad_joint",
        "idx37_hand_l_index_pip_joint", "idx39_hand_l_middle_pip_joint",
        "idx41_hand_l_ring_abad_joint", "idx42_hand_l_ring_pip_joint",
        "idx44_hand_l_pinky_abad_joint", "idx45_hand_l_pinky_pip_joint",
    ],
    "right": [
        "idx71_hand_r_thumb_roll_joint", "idx72_hand_r_thumb_abad_joint",
        "idx73_hand_r_thumb_mcp_joint", "idx76_hand_r_index_abad_joint",
        "idx77_hand_r_index_pip_joint", "idx79_hand_r_middle_pip_joint",
        "idx81_hand_r_ring_abad_joint", "idx82_hand_r_ring_pip_joint",
        "idx84_hand_r_pinky_abad_joint", "idx85_hand_r_pinky_pip_joint",
    ],
}
BASE_LINK = {"left": "hand_l_base_link", "right": "hand_r_base_link"}
TIP_LINK = {
    "left": {
        "thumb": "hand_l_thumb_tip", "index": "hand_l_index_tip",
        "middle": "hand_l_middle_tip", "ring": "hand_l_ring_tip",
        "pinky": "hand_l_pinky_tip",
    },
    "right": {
        "thumb": "hand_r_thumb_tip", "index": "hand_r_index_tip",
        "middle": "hand_r_middle_tip", "ring": "hand_r_ring_tip",
        "pinky": "hand_r_pinky_tip",
    },
}


def ensure_env() -> None:
    if os.environ.get("G2_GDK_ENV_READY") == "1":
        return
    if not ENV_SH.is_file():
        raise FileNotFoundError(str(ENV_SH))
    args = " ".join(shlex.quote(x) for x in [str(Path(__file__).resolve())] + sys.argv[1:])
    command = "source %s && export G2_GDK_ENV_READY=1 && exec /usr/bin/python3.10 %s" % (
        shlex.quote(str(ENV_SH)), args)
    print("[ENV] Restarting with official GDK environment ...", flush=True)
    os.execv("/bin/bash", ["bash", "-lc", command])


def vector(text: Optional[str], default=(0.0, 0.0, 0.0)) -> np.ndarray:
    return np.array([float(x) for x in text.split()] if text else default, dtype=float)


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]], dtype=float)
    ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]], dtype=float)
    rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]], dtype=float)
    return rz @ ry @ rx


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, C = math.cos(angle), math.sin(angle), 1.0-math.cos(angle)
    return np.array([
        [x*x*C+c, x*y*C-z*s, x*z*C+y*s],
        [y*x*C+z*s, y*y*C+c, y*z*C-x*s],
        [z*x*C-y*s, z*y*C+x*s, z*z*C+c],
    ], dtype=float)


def transform_matrix(xyz: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3,:3] = rotation
    result[:3,3] = xyz
    return result


@dataclass
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    lower: Optional[float]
    upper: Optional[float]
    mimic: Optional[str]
    multiplier: float
    offset: float


class URDFModel:
    def __init__(self, path: Path):
        root = ET.fromstring(path.read_text(encoding="utf-8"))
        self.joints: Dict[str, Joint] = {}
        self.by_child: Dict[str, Joint] = {}
        for node in root.findall("joint"):
            origin, axis, limit, mimic = node.find("origin"), node.find("axis"), node.find("limit"), node.find("mimic")
            joint = Joint(
                name=node.attrib["name"], kind=node.attrib.get("type", "fixed"),
                parent=node.find("parent").attrib["link"], child=node.find("child").attrib["link"],
                xyz=vector(origin.attrib.get("xyz") if origin is not None else None),
                rpy=vector(origin.attrib.get("rpy") if origin is not None else None),
                axis=vector(axis.attrib.get("xyz") if axis is not None else None, (1,0,0)),
                lower=float(limit.attrib["lower"]) if limit is not None and "lower" in limit.attrib else None,
                upper=float(limit.attrib["upper"]) if limit is not None and "upper" in limit.attrib else None,
                mimic=mimic.attrib.get("joint") if mimic is not None else None,
                multiplier=float(mimic.attrib.get("multiplier", "1")) if mimic is not None else 1.0,
                offset=float(mimic.attrib.get("offset", "0")) if mimic is not None else 0.0,
            )
            self.joints[joint.name] = joint
            self.by_child[joint.child] = joint

    def chain(self, base: str, tip: str) -> List[Joint]:
        result: List[Joint] = []
        link = tip
        visited: Set[str] = set()
        while link != base:
            if link in visited:
                raise RuntimeError("URDF chain cycle at %s" % link)
            visited.add(link)
            joint = self.by_child.get(link)
            if joint is None:
                raise KeyError("No chain from %s to %s at %s" % (base, tip, link))
            result.append(joint)
            link = joint.parent
        return list(reversed(result))

    def resolve(self, name: str, values: Dict[str, float], stack: Optional[Set[str]]=None) -> float:
        if name in values:
            return float(values[name])
        joint = self.joints.get(name)
        if joint is None or not joint.mimic:
            return 0.0
        stack = set() if stack is None else set(stack)
        if name in stack:
            raise RuntimeError("Mimic cycle at %s" % name)
        stack.add(name)
        return joint.multiplier*self.resolve(joint.mimic, values, stack) + joint.offset

    def progressive_fk(self, base: str, tip: str, values: Dict[str, float]) -> List[Dict[str, Any]]:
        transform = np.eye(4)
        outputs = []
        for joint in self.chain(base, tip):
            transform = transform @ transform_matrix(joint.xyz, rpy_matrix(joint.rpy))
            value = self.resolve(joint.name, values)
            if joint.kind in ("revolute", "continuous"):
                transform = transform @ transform_matrix(np.zeros(3), axis_angle(joint.axis, value))
            elif joint.kind == "prismatic":
                transform = transform @ transform_matrix(joint.axis*value, np.eye(3))
            outputs.append({"joint": joint, "value": value, "transform": transform.copy()})
        return outputs

    def dependents(self, source: str, recursive: bool=True) -> List[Joint]:
        result = []
        frontier = [source]
        visited = set()
        while frontier:
            parent = frontier.pop(0)
            for joint in self.joints.values():
                if joint.mimic == parent and joint.name not in visited:
                    visited.add(joint.name)
                    result.append(joint)
                    if recursive:
                        frontier.append(joint.name)
        return result


def find_urdf() -> Path:
    for name in URDF_NAMES:
        path = BASE_DIR/name
        if path.is_file():
            return path
    raise FileNotFoundError("Latest G2 O10 URDF is required beside this program")


def unwrap_tf(value: Any) -> Any:
    return value[0] if isinstance(value, (tuple, list)) else value


def live_position(tf_api: Any, parent: str, child: str) -> np.ndarray:
    value = unwrap_tf(tf_api.lookup_transform_latest(parent, child))
    obj = getattr(value, "transform", value)
    tr = obj.translation
    return np.array([float(tr.x), float(tr.y), float(tr.z)], dtype=float)


def read_positions(end: Dict[str, Any], active_names: List[str]) -> Dict[str, float]:
    result = {str(name).strip(): float(state["position"]) for name, state in zip(end["names"], end["end_states"])}
    missing = [name for name in active_names if name not in result]
    if missing:
        raise KeyError("Missing active end-state names: %s" % missing)
    return result


def build_hypothesis(base_values: Dict[str, float], mode: str, offsets: Dict[str, float]) -> Dict[str, float]:
    result = {}
    for name, value in base_values.items():
        sign = -1.0 if mode.startswith("NEG") else 1.0
        offset = offsets.get(name, 0.0) if mode.endswith("OFFSET") else 0.0
        result[name] = sign*value + offset
    return result


def analyze_finger(model: URDFModel, tf_api: Any, side: str, finger: str, tip: str,
                   mapped_values: Dict[str, float]) -> Dict[str, Any]:
    base = BASE_LINK[side]
    steps = model.progressive_fk(base, tip, mapped_values)
    rows = []
    first_material = None
    for index, step in enumerate(steps):
        joint: Joint = step["joint"]
        fk = step["transform"][:3,3]
        live = live_position(tf_api, base, joint.child)
        error = float(np.linalg.norm(fk-live))
        if first_material is None and error > MATERIAL_DIVERGENCE_M:
            first_material = {"index": index, "joint": joint.name, "child_link": joint.child, "error_m": error}
        rows.append({
            "index": index, "joint": joint.name, "joint_type": joint.kind,
            "parent_link": joint.parent, "child_link": joint.child,
            "joint_value_used": float(step["value"]),
            "origin_xyz": joint.xyz.tolist(), "origin_rpy": joint.rpy.tolist(), "axis": joint.axis.tolist(),
            "mimic": joint.mimic, "mimic_multiplier": joint.multiplier, "mimic_offset": joint.offset,
            "fk_position_m": fk.tolist(), "live_tf_position_m": live.tolist(),
            "position_error_m": error, "pass_3mm": error <= PASS_TOLERANCE_M,
        })
    final_error = rows[-1]["position_error_m"]
    return {
        "finger": finger, "tip_link": tip, "chain_length": len(rows),
        "steps": rows, "first_material_divergence": first_material,
        "tip_error_m": final_error, "tip_pass": final_error <= PASS_TOLERANCE_M,
    }


def main() -> int:
    ensure_env()
    print("="*108)
    print("G2 O10 End-State to URDF Mapping Diagnostic Probe")
    print("Version             :", VERSION)
    print("READ_ONLY_PROBE     :", READ_ONLY_PROBE)
    print("ENABLE_REAL_MOTION  :", ENABLE_REAL_MOTION)
    print("Motion API          : NONE")
    print("="*108)
    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION:
        return 90

    urdf = find_urdf()
    model = URDFModel(urdf)
    import agibot_gdk
    initialized = False
    try:
        result = agibot_gdk.gdk_init(); print("[GDK] gdk_init result   :", result)
        if result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("gdk_init failed")
        initialized = True
        robot, tf_api = agibot_gdk.Robot(), agibot_gdk.TF()
        time.sleep(WAIT_SEC)
        raw = robot.get_end_state()
        base_values = {
            "left": read_positions(raw["left_end_state"], ACTIVE["left"]),
            "right": read_positions(raw["right_end_state"], ACTIVE["right"]),
        }

        print(); print("[ACTIVE JOINT / URDF DEPENDENCY SUMMARY]")
        active_summary = {}
        for side in ("left", "right"):
            active_summary[side] = []
            print(" ", side.upper())
            for name in ACTIVE[side]:
                joint = model.joints[name]
                deps = model.dependents(name, recursive=True)
                row = {
                    "name": name, "state_position": base_values[side][name], "type": joint.kind,
                    "axis": joint.axis.tolist(), "origin_xyz": joint.xyz.tolist(), "origin_rpy": joint.rpy.tolist(),
                    "lower": joint.lower, "upper": joint.upper,
                    "recursive_mimic_dependents": [
                        {"name": d.name, "multiplier": d.multiplier, "offset": d.offset, "child": d.child}
                        for d in deps
                    ],
                }
                active_summary[side].append(row)
                print("   %-38s q=% .6f deps=%d" % (name, row["state_position"], len(deps)))

        # Offsets remain zero in v6. The four hypotheses intentionally isolate sign behavior first.
        offsets = {side: {name: 0.0 for name in ACTIVE[side]} for side in ("left", "right")}
        modes = ("POS", "NEG", "POS_OFFSET", "NEG_OFFSET")
        hypotheses = {}
        best_mode = {}
        for side in ("left", "right"):
            hypotheses[side] = {}
            print(); print("[%s MAPPING HYPOTHESES]" % side.upper())
            side_scores = []
            for mode in modes:
                mapped = build_hypothesis(base_values[side], mode, offsets[side])
                fingers = {}
                errors = []
                for finger, tip in TIP_LINK[side].items():
                    analysis = analyze_finger(model, tf_api, side, finger, tip, mapped)
                    fingers[finger] = analysis
                    errors.append(analysis["tip_error_m"])
                score = float(np.sqrt(np.mean(np.square(errors))))
                maximum = float(max(errors))
                hypotheses[side][mode] = {"rms_tip_error_m": score, "max_tip_error_m": maximum, "fingers": fingers}
                side_scores.append((score, mode))
                print("  %-10s rms=%7.3f mm max=%7.3f mm" % (mode, 1000*score, 1000*maximum))
            side_scores.sort()
            best_mode[side] = side_scores[0][1]
            print("  Best tested mode:", best_mode[side])

        print(); print("[BEST-MODE PROGRESSIVE DIVERGENCE]")
        overall_pass = True
        for side in ("left", "right"):
            mode = best_mode[side]
            print(" ", side.upper(), mode)
            for finger, analysis in hypotheses[side][mode]["fingers"].items():
                first = analysis["first_material_divergence"]
                overall_pass = overall_pass and analysis["tip_pass"]
                if first:
                    print("   %-7s tip=%7.3f mm first>%dmm at %-38s child=%s err=%7.3f mm" % (
                        finger, 1000*analysis["tip_error_m"], int(1000*MATERIAL_DIVERGENCE_M),
                        first["joint"], first["child_link"], 1000*first["error_m"]))
                else:
                    print("   %-7s tip=%7.3f mm no material divergence" % (finger, 1000*analysis["tip_error_m"]))

        report = {
            "version": VERSION, "timestamp": datetime.now().astimezone().isoformat(),
            "safety": {"read_only": True, "o10_commanded": False, "motion_executed": False},
            "urdf": str(urdf), "table_top_z_base_m": TABLE_TOP_Z_BASE_M,
            "pass_tolerance_m": PASS_TOLERANCE_M,
            "material_divergence_m": MATERIAL_DIVERGENCE_M,
            "active_joint_summary": active_summary,
            "hypotheses": hypotheses,
            "best_tested_mode": best_mode,
            "overall_best_mode_tip_gate_pass": overall_pass,
            "note": "POS_OFFSET and NEG_OFFSET use zero offsets in v6; v6 isolates sign and chain divergence before any offset calibration.",
        }
        REPORT_DIR.mkdir(parents=True, exist_ok=True)
        report_path = REPORT_DIR/("107_o10_state_to_urdf_mapping_probe_v6_%s.json" % datetime.now().strftime("%Y%m%d_%H%M%S"))
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

        print(); print("="*108)
        print("MAPPING PROBE SUMMARY")
        print("="*108)
        print("Joint/chain introspection       : PASS")
        print("Four mapping hypotheses tested : PASS")
        print("Best left mode                  :", best_mode["left"])
        print("Best right mode                 :", best_mode["right"])
        print("3 mm fingertip gate             :", "PASS" if overall_pass else "BLOCK")
        print("Grasp solver enabled            : NO")
        print("O10 commanded                   : NO")
        print("Motion executed                 : NO")
        print("JSON report                     :", report_path)
        print("="*108)
        return 0 if overall_pass else 2
    finally:
        if initialized:
            print(); print("[GDK] gdk_release result :", agibot_gdk.gdk_release())


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(); print("[FATAL] %s: %s" % (type(exc).__name__, exc)); traceback.print_exc()
        print("[SAFETY] No O10, arm, TCP, wheel or base command was issued.")
        raise SystemExit(1)
