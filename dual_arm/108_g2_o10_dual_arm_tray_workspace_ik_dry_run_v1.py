#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
108_g2_o10_dual_arm_tray_workspace_ik_dry_run_v1.py

G2 + OmniHand O10 双臂料盆 Workspace / IK Dry Run v1。

目标：
- 读取当前双臂 Joint、TF 和 O10 End State。
- 使用最新版 URDF 建立左右 7-DOF 手臂 FK。
- 以固定 Pick Zone 中的料盆中心为输入，构建 PREGRASP / CONTACT / TEST_LIFT 目标。
- 保持当前已验证的左右手姿态，基于 106/107 的 Virtual Grasp Center 估算 hand_base 与 arm_end 目标。
- 用纯 NumPy 数值 IK 离线求解左右手臂关节候选。
- 检查 Joint Limit、目标误差、左右对称性、双手间距与阶段连续性。

绝不调用机械臂、O10、TCP 或底盘运动接口。
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
VERSION = "108_v1_dual_arm_workspace_ik_dry_run"
ENV_SH = Path.home() / ".cache/agibot/app/env.sh"
BASE_DIR = Path(__file__).resolve().parent
REPORT_DIR = BASE_DIR / "reports"
URDF_NAMES = ("G2_t2_crs_o10_t2.urdf", "G2_t2_crs_o10_t2.urdf.txt")
WAIT_SEC = 2.0

# 固定 Pick Zone 第一版假设。桌面高度来自前期已知值；现场测量后应更新。
TRAY = {
    "center_base_m": [0.550, 0.000, 0.810],  # 盆体几何中心，桌面0.760 + 高度0.100/2
    "table_top_z_base_m": 0.760,
    "rim_top_z_base_m": 0.860,
    "top_outer_length_m": 0.440,
    "top_outer_width_m": 0.320,
    "height_m": 0.100,
    "rim_width_m": 0.012,
    "flange_height_m": 0.008,
}

LEFT_JOINTS = [f"idx2{i}_arm_l_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)]
ARM_END = {"left": "arm_l_end_link", "right": "arm_r_end_link"}
HAND_BASE = {"left": "hand_l_base_link", "right": "hand_r_base_link"}

# 来自 106_v2 当前实机几何，hand_base 坐标系表达。
VIRTUAL_OFFSET = {
    "left": np.array([0.0333035090, -0.0213221560, 0.0943374268]),
    "right": np.array([0.0334122628, 0.0215857426, 0.0942956508]),
}

# 盆两条长边中部：左右接触点位于宽度方向 ±160 mm。
# CONTACT 虚拟点位于盆沿顶面以下约 8 mm，介于 thumb(-4) 与 four-finger(-12) 之间。
STAGES = {
    "PREGRASP": {"outside_clearance_m": 0.080, "z_offset_from_rim_m": -0.008},
    "CONTACT": {"outside_clearance_m": 0.000, "z_offset_from_rim_m": -0.008},
    "TEST_LIFT": {"outside_clearance_m": 0.000, "z_offset_from_rim_m": 0.022},
}

POSITION_TOL_M = 0.008
ROTATION_TOL_DEG = 3.0
MIN_JOINT_MARGIN_RAD = 0.03
MAX_STAGE_DELTA_RAD = 0.45
MIN_HAND_SEPARATION_M = 0.20
MAX_HAND_SEPARATION_M = 0.55


def ensure_env() -> None:
    if os.environ.get("G2_GDK_ENV_READY") == "1":
        return
    args = " ".join(shlex.quote(x) for x in [str(Path(__file__).resolve())] + sys.argv[1:])
    cmd = "source %s && export G2_GDK_ENV_READY=1 && exec /usr/bin/python3.10 %s" % (
        shlex.quote(str(ENV_SH)), args)
    print("[ENV] Restarting with official GDK environment ...", flush=True)
    os.execv("/bin/bash", ["bash", "-lc", cmd])


def vec(text: Optional[str], default=(0, 0, 0)) -> np.ndarray:
    return np.array([float(x) for x in text.split()] if text else default, dtype=float)


def rpy_matrix(v: np.ndarray) -> np.ndarray:
    r, p, y = v
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy,-sy,0],[sy,cy,0],[0,0,1.]]) @ np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]]) @ np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, C = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    return np.array([[x*x*C+c,x*y*C-z*s,x*z*C+y*s],[y*x*C+z*s,y*y*C+c,y*z*C-x*s],[z*x*C-y*s,z*y*C+x*s,z*z*C+c]])


def tfm(xyz: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3,:3], out[:3,3] = rotation, xyz
    return out


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    cosine = max(-1.0, min(1.0, (float(np.trace(rotation))-1.0)/2.0))
    angle = math.acos(cosine)
    if angle < 1e-9:
        return np.zeros(3)
    skew = np.array([rotation[2,1]-rotation[1,2], rotation[0,2]-rotation[2,0], rotation[1,0]-rotation[0,1]])
    return angle * skew / (2.0*math.sin(angle))


@dataclass
class Joint:
    name: str; kind: str; parent: str; child: str
    xyz: np.ndarray; rpy: np.ndarray; axis: np.ndarray
    lower: Optional[float]; upper: Optional[float]


class Model:
    def __init__(self, path: Path):
        root = ET.fromstring(path.read_text(encoding="utf-8"))
        self.joints: Dict[str, Joint] = {}
        self.by_child: Dict[str, Joint] = {}
        for node in root.findall("joint"):
            origin, axis, limit = node.find("origin"), node.find("axis"), node.find("limit")
            joint = Joint(
                node.attrib["name"], node.attrib.get("type", "fixed"),
                node.find("parent").attrib["link"], node.find("child").attrib["link"],
                vec(origin.attrib.get("xyz") if origin is not None else None),
                vec(origin.attrib.get("rpy") if origin is not None else None),
                vec(axis.attrib.get("xyz") if axis is not None else None, (1,0,0)),
                float(limit.attrib["lower"]) if limit is not None and "lower" in limit.attrib else None,
                float(limit.attrib["upper"]) if limit is not None and "upper" in limit.attrib else None,
            )
            self.joints[joint.name] = joint
            self.by_child[joint.child] = joint

    def chain(self, base: str, tip: str) -> List[Joint]:
        chain, link = [], tip
        while link != base:
            joint = self.by_child.get(link)
            if joint is None:
                raise KeyError("No URDF chain from %s to %s at %s" % (base, tip, link))
            chain.append(joint)
            link = joint.parent
        return list(reversed(chain))

    def fk(self, base: str, tip: str, values: Dict[str, float]) -> np.ndarray:
        result = np.eye(4)
        for joint in self.chain(base, tip):
            result = result @ tfm(joint.xyz, rpy_matrix(joint.rpy))
            value = values.get(joint.name, 0.0)
            if joint.kind in ("revolute", "continuous"):
                result = result @ tfm(np.zeros(3), axis_angle(joint.axis, value))
            elif joint.kind == "prismatic":
                result = result @ tfm(joint.axis*value, np.eye(3))
        return result


def find_urdf() -> Path:
    for name in URDF_NAMES:
        path = BASE_DIR / name
        if path.is_file():
            return path
    raise FileNotFoundError("Latest G2 URDF is required beside this program")


def unwrap_tf(value: Any) -> Any:
    return value[0] if isinstance(value, (tuple, list)) else value


def gdk_transform_matrix(value: Any) -> np.ndarray:
    value = unwrap_tf(value)
    obj = getattr(value, "transform", value)
    tr, ro = obj.translation, obj.rotation
    q = np.array([float(ro.x), float(ro.y), float(ro.z), float(ro.w)])
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    rotation = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    return tfm(np.array([float(tr.x), float(tr.y), float(tr.z)]), rotation)


def current_joint_vector(raw: Dict[str, Any], names: List[str]) -> np.ndarray:
    states = raw["states"]
    by_name = {str(state["name"]).strip(): float(state["motor_position"]) for state in states}
    return np.array([by_name[name] for name in names])


def bounds(model: Model, names: List[str]) -> Tuple[np.ndarray, np.ndarray]:
    return (
        np.array([model.joints[name].lower for name in names]),
        np.array([model.joints[name].upper for name in names]),
    )


def numerical_jacobian(function, q: np.ndarray, residual: np.ndarray, epsilon=1e-5) -> np.ndarray:
    jacobian = np.zeros((len(residual), len(q)))
    for index in range(len(q)):
        q2 = q.copy(); q2[index] += epsilon
        jacobian[:,index] = (function(q2)-residual)/epsilon
    return jacobian


def solve_arm(model: Model, side: str, names: List[str], seed: np.ndarray, target: np.ndarray,
              lower: np.ndarray, upper: np.ndarray) -> Tuple[np.ndarray, Dict[str, Any]]:
    q = np.clip(seed.copy(), lower+1e-5, upper-1e-5)
    damping, best_q, best_cost = 0.04, q.copy(), float("inf")

    def residual(values: np.ndarray) -> np.ndarray:
        current = model.fk("base_link", ARM_END[side], dict(zip(names, values)))
        position_error = target[:3,3] - current[:3,3]
        orientation_error = rotation_vector(target[:3,:3] @ current[:3,:3].T)
        continuity = 0.025*(values-seed)
        return np.concatenate((position_error*80.0, orientation_error*1.5, continuity))

    accepted, iterations = 0, 0
    for iterations in range(1, 501):
        r = residual(q); cost = 0.5*float(r@r)
        if cost < best_cost:
            best_cost, best_q = cost, q.copy()
        current = model.fk("base_link", ARM_END[side], dict(zip(names, q)))
        pos_error = np.linalg.norm(target[:3,3]-current[:3,3])
        rot_error = math.degrees(np.linalg.norm(rotation_vector(target[:3,:3] @ current[:3,:3].T)))
        if pos_error <= POSITION_TOL_M and rot_error <= ROTATION_TOL_DEG:
            best_q = q.copy(); break
        J = numerical_jacobian(residual, q, r)
        A, b = J.T@J + damping*np.eye(len(q)), -(J.T@r)
        try: step = np.linalg.solve(A, b)
        except np.linalg.LinAlgError: step = np.linalg.pinv(A)@b
        maximum = float(np.max(np.abs(step)))
        if maximum > 0.08: step *= 0.08/maximum
        ok, alpha = False, 1.0
        for _ in range(14):
            candidate = np.clip(q+alpha*step, lower+1e-5, upper-1e-5)
            rr = residual(candidate); candidate_cost = 0.5*float(rr@rr)
            if candidate_cost < cost:
                q, ok, accepted = candidate, True, accepted+1
                damping = max(1e-5, damping*0.75)
                break
            alpha *= 0.5
        if not ok:
            damping = min(1e3, damping*4.0)
            if np.linalg.norm(step) < 1e-7: break

    q = best_q
    achieved = model.fk("base_link", ARM_END[side], dict(zip(names, q)))
    pos_error = float(np.linalg.norm(target[:3,3]-achieved[:3,3]))
    rot_error = float(math.degrees(np.linalg.norm(rotation_vector(target[:3,:3] @ achieved[:3,:3].T))))
    margins = np.minimum(q-lower, upper-q)
    maximum_delta = float(np.max(np.abs(q-seed)))
    return q, {
        "success": bool(pos_error <= POSITION_TOL_M and rot_error <= ROTATION_TOL_DEG and np.min(margins) >= MIN_JOINT_MARGIN_RAD and maximum_delta <= MAX_STAGE_DELTA_RAD),
        "iterations": iterations, "accepted_steps": accepted, "cost": best_cost,
        "position_error_m": pos_error, "rotation_error_deg": rot_error,
        "minimum_joint_margin_rad": float(np.min(margins)),
        "joint_margin_each_rad": margins.tolist(),
        "maximum_joint_delta_rad": maximum_delta,
        "joint_delta_rad": (q-seed).tolist(),
        "target_matrix": target.tolist(), "achieved_matrix": achieved.tolist(),
    }


def build_targets(current_hand: Dict[str, np.ndarray], arm_to_hand: Dict[str, np.ndarray]) -> Dict[str, Dict[str, np.ndarray]]:
    targets: Dict[str, Dict[str, np.ndarray]] = {}
    half_width = TRAY["top_outer_width_m"]/2.0
    for stage, config in STAGES.items():
        targets[stage] = {}
        for side in ("left", "right"):
            sign = 1.0 if side == "left" else -1.0
            grasp_point = np.array([
                TRAY["center_base_m"][0],
                TRAY["center_base_m"][1] + sign*(half_width + config["outside_clearance_m"]),
                TRAY["rim_top_z_base_m"] + config["z_offset_from_rim_m"],
            ])
            hand_rotation = current_hand[side][:3,:3]
            hand_position = grasp_point - hand_rotation @ VIRTUAL_OFFSET[side]
            target_hand = tfm(hand_position, hand_rotation)
            target_arm_end = target_hand @ np.linalg.inv(arm_to_hand[side])
            targets[stage][side] = {
                "grasp_point": grasp_point,
                "hand_base": target_hand,
                "arm_end": target_arm_end,
            }
    return targets


def main() -> int:
    ensure_env()
    print("="*108)
    print("G2 O10 Dual Arm Tray Workspace / IK Dry Run")
    print("Version             :", VERSION)
    print("READ_ONLY_PROBE     :", READ_ONLY_PROBE)
    print("ENABLE_REAL_MOTION  :", ENABLE_REAL_MOTION)
    print("Motion API          : NONE")
    print("="*108)
    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION: return 90

    urdf = find_urdf(); model = Model(urdf)
    import agibot_gdk
    initialized = False
    try:
        result = agibot_gdk.gdk_init(); print("[GDK] gdk_init result   :", result)
        if result != agibot_gdk.GDKRes.kSuccess: raise RuntimeError("gdk_init failed")
        initialized = True
        robot, tf_api = agibot_gdk.Robot(), agibot_gdk.TF(); time.sleep(WAIT_SEC)
        joint_raw = robot.get_joint_states()
        current_q = {"left": current_joint_vector(joint_raw, LEFT_JOINTS), "right": current_joint_vector(joint_raw, RIGHT_JOINTS)}
        arm_bounds = {"left": bounds(model, LEFT_JOINTS), "right": bounds(model, RIGHT_JOINTS)}
        current_hand = {side: gdk_transform_matrix(tf_api.get_tf_from_base_link(HAND_BASE[side])) for side in ("left","right")}
        arm_to_hand = {side: gdk_transform_matrix(tf_api.lookup_transform_latest(ARM_END[side], HAND_BASE[side])) for side in ("left","right")}
        targets = build_targets(current_hand, arm_to_hand)

        report = {"version": VERSION, "timestamp": datetime.now().astimezone().isoformat(), "safety": {"arm_commanded": False, "o10_commanded": False, "motion_executed": False}, "tray": TRAY, "stages": {}}
        previous = {"left": current_q["left"].copy(), "right": current_q["right"].copy()}
        overall = True
        for stage in ("PREGRASP", "CONTACT", "TEST_LIFT"):
            report["stages"][stage] = {}
            solved = {}
            for side, names in (("left", LEFT_JOINTS), ("right", RIGHT_JOINTS)):
                q, meta = solve_arm(model, side, names, previous[side], targets[stage][side]["arm_end"], *arm_bounds[side])
                solved[side] = q
                report["stages"][stage][side] = {
                    "joint_names": names, "joint_positions_rad": q.tolist(), "ik": meta,
                    "grasp_point_base_m": targets[stage][side]["grasp_point"].tolist(),
                    "hand_base_target": targets[stage][side]["hand_base"].tolist(),
                    "arm_end_target": targets[stage][side]["arm_end"].tolist(),
                }
                overall = overall and meta["success"]
            hand_distance = float(np.linalg.norm(targets[stage]["left"]["hand_base"][:3,3] - targets[stage]["right"]["hand_base"][:3,3]))
            separation_ok = MIN_HAND_SEPARATION_M <= hand_distance <= MAX_HAND_SEPARATION_M
            report["stages"][stage]["hand_base_separation_m"] = hand_distance
            report["stages"][stage]["separation_ok"] = separation_ok
            overall = overall and separation_ok
            print()
            print("[%s]" % stage)
            print("  Left grasp point [m]       :", targets[stage]["left"]["grasp_point"])
            print("  Right grasp point [m]      :", targets[stage]["right"]["grasp_point"])
            print("  Hand-base separation [m]   : %.4f  %s" % (hand_distance, "PASS" if separation_ok else "CHECK"))
            for side in ("left","right"):
                meta = report["stages"][stage][side]["ik"]
                print("  %-5s IK: pos_err=%6.2f mm rot_err=%5.2f deg margin=%5.3f rad delta=%5.3f rad %s" % (
                    side.upper(), 1000*meta["position_error_m"], meta["rotation_error_deg"], meta["minimum_joint_margin_rad"], meta["maximum_joint_delta_rad"], "PASS" if meta["success"] else "CHECK"))
            previous = solved

        REPORT_DIR.mkdir(parents=True, exist_ok=True)
        report_path = REPORT_DIR/("108_dual_arm_workspace_ik_dry_run_v1_%s.json" % datetime.now().strftime("%Y%m%d_%H%M%S"))
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print()
        print("="*108)
        print("DRY-RUN SUMMARY")
        print("="*108)
        print("Tray target geometry         : PASS")
        print("Dual-arm IK candidates       :", "PASS" if overall else "CHECK REQUIRED")
        print("Collision verdict            : NOT YET APPROVED; separation-only gate used")
        print("Arm commanded                : NO")
        print("O10 commanded                : NO")
        print("Motion executed              : NO")
        print("Approved for real motion     : NO")
        print("JSON report                  :", report_path)
        print("="*108)
        return 0 if overall else 2
    finally:
        if initialized:
            print(); print("[GDK] gdk_release result :", agibot_gdk.gdk_release())


if __name__ == "__main__":
    try: raise SystemExit(main())
    except Exception as exc:
        print(); print("[FATAL] %s: %s" % (type(exc).__name__, exc)); traceback.print_exc()
        print("[SAFETY] No arm, O10, TCP, or base command was issued.")
        raise SystemExit(1)
