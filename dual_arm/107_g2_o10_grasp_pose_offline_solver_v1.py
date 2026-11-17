#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
107_g2_o10_grasp_pose_offline_solver_v1.py

O10 “拇指在盆沿内 + 四指在外”离线候选手型求解器 v1。

本程序：
- 自动加载 GDK 4.1.5 官方环境。
- 只读获取左右 O10 当前 10+10 DOF 状态。
- 从同目录最新版 G2 URDF 建立 hand_base -> fingertip 前向运动学。
- 用 scipy least_squares 离线计算 OPEN / PREGRASP / CONTACT / GRASP 候选手型。
- 左右手分别使用各自 URDF，并检查镜像一致性、Joint Limit、指尖几何。
- 只生成候选 JSON，不发送 O10、机械臂、TCP 或底盘命令。

v1 假设（必须经过后续 Dry Run 和低风险动作验证）：
- 以 Thumb tip 到四指 tip 平均中心的欧氏距离作为“aperture”代理量。
- OPEN=90 mm，PREGRASP=45 mm，CONTACT=22 mm，GRASP=18 mm。
- CONTACT/GRASP 是离线候选，不代表已验证安全接触力。
- 四指低于盆沿翻边的绝对 Z，由后续 108 手掌/双臂姿态求解负责；107 只求手内相对几何。
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
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

READ_ONLY_PROBE = True
ENABLE_REAL_MOTION = False
PROGRAM_VERSION = "107_v1_o10_offline_grasp_solver"
ENV_SH = Path.home() / ".cache/agibot/app/env.sh"
BASE_DIR = Path(__file__).resolve().parent
REPORT_DIR = BASE_DIR / "reports"
URDF_CANDIDATES = ("G2_t2_crs_o10_t2.urdf", "G2_t2_crs_o10_t2.urdf.txt")
WAIT_SEC = 2.0

LEFT_ACTIVE = [
    "idx31_hand_l_thumb_roll_joint", "idx32_hand_l_thumb_abad_joint",
    "idx33_hand_l_thumb_mcp_joint", "idx36_hand_l_index_abad_joint",
    "idx37_hand_l_index_pip_joint", "idx39_hand_l_middle_pip_joint",
    "idx41_hand_l_ring_abad_joint", "idx42_hand_l_ring_pip_joint",
    "idx44_hand_l_pinky_abad_joint", "idx45_hand_l_pinky_pip_joint",
]
RIGHT_ACTIVE = [
    "idx71_hand_r_thumb_roll_joint", "idx72_hand_r_thumb_abad_joint",
    "idx73_hand_r_thumb_mcp_joint", "idx76_hand_r_index_abad_joint",
    "idx77_hand_r_index_pip_joint", "idx79_hand_r_middle_pip_joint",
    "idx81_hand_r_ring_abad_joint", "idx82_hand_r_ring_pip_joint",
    "idx84_hand_r_pinky_abad_joint", "idx85_hand_r_pinky_pip_joint",
]
TIP_LINKS = {
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
BASE_LINKS = {"left": "hand_l_base_link", "right": "hand_r_base_link"}
MIRROR_SIGN = np.array([-1., -1., -1., -1., 1., 1., -1., 1., -1., 1.])

# Candidate targets. These are engineering starting points, not approved motion targets.
STAGES = {
    "OPEN": {"aperture_m": 0.090, "regularization": 0.08},
    "PREGRASP": {"aperture_m": 0.045, "regularization": 0.04},
    "CONTACT": {"aperture_m": 0.022, "regularization": 0.025},
    "GRASP": {"aperture_m": 0.018, "regularization": 0.020},
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


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]], dtype=float)
    ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]], dtype=float)
    rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]], dtype=float)
    return rz @ ry @ rx


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, C = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    return np.array([
        [x*x*C+c, x*y*C-z*s, x*z*C+y*s],
        [y*x*C+z*s, y*y*C+c, y*z*C-x*s],
        [z*x*C-y*s, z*y*C+x*s, z*z*C+c],
    ], dtype=float)


def tf_matrix(xyz: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = xyz
    return transform


def parse_vec(text: Optional[str], default: Tuple[float,float,float]) -> np.ndarray:
    if not text:
        return np.array(default, dtype=float)
    return np.array([float(value) for value in text.split()], dtype=float)


@dataclass
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    lower: Optional[float]
    upper: Optional[float]
    mimic_joint: Optional[str]
    mimic_multiplier: float
    mimic_offset: float


class URDFModel:
    def __init__(self, path: Path):
        root = ET.fromstring(path.read_text(encoding="utf-8"))
        self.joints: Dict[str, Joint] = {}
        self.child_joint: Dict[str, Joint] = {}
        for node in root.findall("joint"):
            name = node.attrib["name"]
            parent = node.find("parent").attrib["link"]
            child = node.find("child").attrib["link"]
            origin = node.find("origin")
            axis = node.find("axis")
            limit = node.find("limit")
            mimic = node.find("mimic")
            joint = Joint(
                name=name, joint_type=node.attrib.get("type", "fixed"), parent=parent, child=child,
                origin_xyz=parse_vec(origin.attrib.get("xyz") if origin is not None else None, (0,0,0)),
                origin_rpy=parse_vec(origin.attrib.get("rpy") if origin is not None else None, (0,0,0)),
                axis=parse_vec(axis.attrib.get("xyz") if axis is not None else None, (1,0,0)),
                lower=float(limit.attrib["lower"]) if limit is not None and "lower" in limit.attrib else None,
                upper=float(limit.attrib["upper"]) if limit is not None and "upper" in limit.attrib else None,
                mimic_joint=mimic.attrib.get("joint") if mimic is not None else None,
                mimic_multiplier=float(mimic.attrib.get("multiplier", "1")) if mimic is not None else 1.0,
                mimic_offset=float(mimic.attrib.get("offset", "0")) if mimic is not None else 0.0,
            )
            self.joints[name] = joint
            self.child_joint[child] = joint

    def chain(self, base: str, tip: str) -> List[Joint]:
        chain: List[Joint] = []
        link = tip
        visited = set()
        while link != base:
            if link in visited:
                raise RuntimeError("URDF cycle while resolving %s -> %s" % (base, tip))
            visited.add(link)
            joint = self.child_joint.get(link)
            if joint is None:
                raise KeyError("No parent joint for link %s" % link)
            chain.append(joint)
            link = joint.parent
        chain.reverse()
        return chain

    def joint_value(self, joint: Joint, values: Dict[str, float]) -> float:
        if joint.mimic_joint:
            return joint.mimic_multiplier * values.get(joint.mimic_joint, 0.0) + joint.mimic_offset
        return values.get(joint.name, 0.0)

    def fk(self, base: str, tip: str, values: Dict[str, float]) -> np.ndarray:
        transform = np.eye(4)
        for joint in self.chain(base, tip):
            transform = transform @ tf_matrix(joint.origin_xyz, rpy_matrix(joint.origin_rpy))
            q = self.joint_value(joint, values)
            if joint.joint_type in ("revolute", "continuous"):
                transform = transform @ tf_matrix(np.zeros(3), axis_angle(joint.axis, q))
            elif joint.joint_type == "prismatic":
                transform = transform @ tf_matrix(joint.axis * q, np.eye(3))
        return transform


def find_urdf() -> Path:
    for filename in URDF_CANDIDATES:
        path = BASE_DIR / filename
        if path.is_file():
            return path
    raise FileNotFoundError("Place G2_t2_crs_o10_t2.urdf in the same directory as this program")


def parse_end_state(end: Dict[str, Any], active_names: List[str], side: str) -> np.ndarray:
    names = [str(name).strip() for name in end["names"]]
    states = end["end_states"]
    by_name = {name: float(state["position"]) for name, state in zip(names, states)}
    missing = [name for name in active_names if name not in by_name]
    if missing:
        raise KeyError("%s missing end-state joints: %s" % (side, missing))
    return np.array([by_name[name] for name in active_names], dtype=float)


def bounds_for(model: URDFModel, active_names: List[str]) -> Tuple[np.ndarray, np.ndarray]:
    lower, upper = [], []
    for name in active_names:
        joint = model.joints[name]
        lower.append(joint.lower if joint.lower is not None else -math.pi)
        upper.append(joint.upper if joint.upper is not None else math.pi)
    return np.array(lower), np.array(upper)


def geometry(model: URDFModel, side: str, active_names: List[str], q: np.ndarray) -> Dict[str, Any]:
    values = dict(zip(active_names, [float(value) for value in q]))
    points = {}
    for finger, link in TIP_LINKS[side].items():
        points[finger] = model.fk(BASE_LINKS[side], link, values)[:3, 3]
    four = np.mean(np.vstack([points[name] for name in ("index", "middle", "ring", "pinky")]), axis=0)
    primary = np.mean(np.vstack([points[name] for name in ("index", "middle")]), axis=0)
    thumb = points["thumb"]
    return {
        "points": points,
        "thumb": thumb,
        "four_center": four,
        "primary_center": primary,
        "virtual_center": 0.5 * (thumb + four),
        "aperture": float(np.linalg.norm(thumb - four)),
        "four_spread": float(max(np.linalg.norm(points[a]-points[b]) for a in ("index","middle","ring","pinky") for b in ("index","middle","ring","pinky"))),
    }


def solve_stage(model: URDFModel, side: str, active_names: List[str], q_seed: np.ndarray,
                target_aperture: float, regularization: float,
                lower: np.ndarray, upper: np.ndarray) -> Tuple[np.ndarray, Dict[str, Any]]:
    try:
        from scipy.optimize import least_squares
    except ImportError as exc:
        raise RuntimeError("scipy is required for 107 offline solver") from exc

    scale_position = 100.0  # meters -> solver scale
    seed = np.clip(q_seed, lower + 1e-6, upper - 1e-6)

    def residual(q: np.ndarray) -> np.ndarray:
        geo = geometry(model, side, active_names, q)
        # Aperture goal plus regularization to avoid large/unnecessary joint changes.
        aperture_error = scale_position * (geo["aperture"] - target_aperture)
        # Encourage four fingers to remain grouped, while avoiding an over-determined absolute pose.
        four_z = np.array([geo["points"][name][2] for name in ("index","middle","ring","pinky")])
        grouping = 25.0 * (four_z - np.mean(four_z))
        joint_regularization = regularization * (q - seed)
        return np.concatenate(([aperture_error], grouping, joint_regularization))

    result = least_squares(
        residual, seed, bounds=(lower, upper), max_nfev=1500,
        ftol=1e-10, xtol=1e-10, gtol=1e-10,
    )
    q = result.x
    geo = geometry(model, side, active_names, q)
    meta = {
        "success": bool(result.success), "status": int(result.status),
        "message": str(result.message), "cost": float(result.cost),
        "nfev": int(result.nfev), "target_aperture_m": target_aperture,
        "achieved_aperture_m": geo["aperture"],
        "aperture_error_m": geo["aperture"] - target_aperture,
        "max_joint_delta_rad": float(np.max(np.abs(q-seed))),
        "joint_limit_margin_rad": float(np.min(np.minimum(q-lower, upper-q))),
        "geometry": {
            "tip_positions_m": {key: value.tolist() for key, value in geo["points"].items()},
            "thumb_m": geo["thumb"].tolist(),
            "four_center_m": geo["four_center"].tolist(),
            "primary_center_m": geo["primary_center"].tolist(),
            "virtual_center_m": geo["virtual_center"].tolist(),
            "four_spread_m": geo["four_spread"],
        },
    }
    return q, meta


def main() -> int:
    ensure_env()
    print("=" * 104)
    print("G2 O10 Offline Grasp Pose Solver")
    print("Version             :", PROGRAM_VERSION)
    print("READ_ONLY_PROBE     :", READ_ONLY_PROBE)
    print("ENABLE_REAL_MOTION  :", ENABLE_REAL_MOTION)
    print("Motion API          : NONE")
    print("Candidate status    : NOT APPROVED FOR MOTION")
    print("=" * 104)
    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION:
        return 90

    urdf_path = find_urdf()
    model = URDFModel(urdf_path)
    print("[URDF]", urdf_path)

    import agibot_gdk
    initialized = False
    try:
        init_result = agibot_gdk.gdk_init()
        print("[GDK] gdk_init result   :", init_result)
        if init_result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("gdk_init failed: %s" % init_result)
        initialized = True
        robot = agibot_gdk.Robot()
        time.sleep(WAIT_SEC)
        state = robot.get_end_state()
        current_left = parse_end_state(state["left_end_state"], LEFT_ACTIVE, "left")
        current_right = parse_end_state(state["right_end_state"], RIGHT_ACTIVE, "right")

        left_bounds = bounds_for(model, LEFT_ACTIVE)
        right_bounds = bounds_for(model, RIGHT_ACTIVE)
        report: Dict[str, Any] = {
            "version": PROGRAM_VERSION,
            "timestamp": datetime.now().astimezone().isoformat(),
            "safety": {"o10_commanded": False, "motion_executed": False, "approved_for_motion": False},
            "urdf": str(urdf_path),
            "assumptions": {
                "aperture_proxy": "Euclidean distance from thumb tip to mean of four fingertips",
                "absolute_flange_contact_z_is_deferred_to_108": True,
                "stage_targets_m": {name: cfg["aperture_m"] for name, cfg in STAGES.items()},
            },
            "current": {"left": current_left.tolist(), "right": current_right.tolist()},
            "stages": {},
        }

        left_seed = current_left.copy()
        right_seed = current_right.copy()
        all_success = True
        for stage_name, stage_cfg in STAGES.items():
            left_q, left_meta = solve_stage(model, "left", LEFT_ACTIVE, left_seed,
                                             stage_cfg["aperture_m"], stage_cfg["regularization"], *left_bounds)
            right_q, right_meta = solve_stage(model, "right", RIGHT_ACTIVE, right_seed,
                                               stage_cfg["aperture_m"], stage_cfg["regularization"], *right_bounds)
            normalized_right = MIRROR_SIGN * right_q
            mirror_delta = left_q - normalized_right
            stage_ok = (
                left_meta["success"] and right_meta["success"] and
                abs(left_meta["aperture_error_m"]) <= 0.002 and
                abs(right_meta["aperture_error_m"]) <= 0.002 and
                left_meta["joint_limit_margin_rad"] > 0.01 and
                right_meta["joint_limit_margin_rad"] > 0.01
            )
            all_success = all_success and stage_ok
            report["stages"][stage_name] = {
                "status": "CANDIDATE_ONLY_NOT_APPROVED",
                "stage_ok": stage_ok,
                "left_joint_names": LEFT_ACTIVE,
                "right_joint_names": RIGHT_ACTIVE,
                "left_positions_rad": left_q.tolist(),
                "right_positions_rad": right_q.tolist(),
                "right_positions_in_left_convention_rad": normalized_right.tolist(),
                "mirror_delta_rad": mirror_delta.tolist(),
                "max_mirror_delta_rad": float(np.max(np.abs(mirror_delta))),
                "left_solver": left_meta,
                "right_solver": right_meta,
            }
            print()
            print("[%s CANDIDATE]" % stage_name)
            print("  Target aperture [mm] : %.1f" % (1000 * stage_cfg["aperture_m"]))
            print("  Left achieved [mm]   : %.3f" % (1000 * left_meta["achieved_aperture_m"]))
            print("  Right achieved [mm]  : %.3f" % (1000 * right_meta["achieved_aperture_m"]))
            print("  Left limit margin    : %.4f rad" % left_meta["joint_limit_margin_rad"])
            print("  Right limit margin   : %.4f rad" % right_meta["joint_limit_margin_rad"])
            print("  Max mirror delta     : %.4f rad" % float(np.max(np.abs(mirror_delta))))
            print("  Stage status         :", "PASS_CANDIDATE" if stage_ok else "CHECK_REQUIRED")
            left_seed, right_seed = left_q, right_q

        REPORT_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = REPORT_DIR / ("107_o10_offline_grasp_solver_v1_%s.json" % timestamp)
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

        print()
        print("=" * 104)
        print("OFFLINE SOLVER SUMMARY")
        print("=" * 104)
        print("URDF parsed                   : PASS")
        print("Current O10 state read        : PASS")
        print("OPEN/PREGRASP/CONTACT/GRASP   :", "CANDIDATES GENERATED" if all_success else "CHECK REQUIRED")
        print("Absolute tray contact Z       : DEFERRED TO 108")
        print("Approved for real motion      : NO")
        print("O10 commanded                 : NO")
        print("Motion executed               : NO")
        print("JSON report                   :", report_path)
        print("=" * 104)
        return 0 if all_success else 2
    finally:
        if initialized:
            release_result = agibot_gdk.gdk_release()
            print()
            print("[GDK] gdk_release result :", release_result)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print()
        print("[FATAL] %s: %s" % (type(exc).__name__, exc))
        traceback.print_exc()
        print("[SAFETY] No O10 or robot motion command was issued.")
        raise SystemExit(1)
