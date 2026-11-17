#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
106_g2_o10_thumb_inside_four_fingers_outside_dry_run_v2.py

G2 + OmniHand O10 抓盆几何 Dry Run v2
策略：拇指在盆沿内，食指/中指/无名指/小指在盆沿外并低于 8 mm 下翻边。

重要：本程序只读取状态与 TF，并计算候选接触几何。
不控制 O10，不控制双臂，不修改 TCP，不执行真实运动。
"""
from __future__ import annotations

import json
import math
import os
import shlex
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

import numpy as np

READ_ONLY_PROBE = True
ENABLE_REAL_MOTION = False
PROGRAM_VERSION = "106_v2_tray_geometry_contact_dry_run"
ENV_SH = Path.home() / ".cache/agibot/app/env.sh"
REPORT_DIR = Path(__file__).resolve().parent / "reports"
WAIT_SEC = 2.0

# -----------------------------------------------------------------------------
# 已确认的真实料盆数据，单位统一为 SI。
# -----------------------------------------------------------------------------
TRAY = {
    "top_outer_length_m": 0.440,
    "top_outer_width_m": 0.320,
    "bottom_contact_length_m": 0.377,
    "bottom_contact_width_m": 0.263,
    "overall_height_m": 0.100,
    "rim_width_m": 0.012,
    "rim_material_thickness_m": 0.001,
    "rim_downward_flange_height_m": 0.008,
    "wall_thickness_m": 0.001,
    "rim_corner_radius_m": 0.050,
    "empty_mass_kg": 0.050,
    "maximum_total_mass_kg": 1.000,
}

# 以上沿最高平面为 tray_rim_z = 0，向下为负。
# 四指接触中心必须低于翻边底沿 z=-8 mm。
# 由于尚未取得指腹有效半径，v2 使用 4 mm 几何裕量，名义中心为 -12 mm。
CONTACT = {
    "tray_rim_top_z_m": 0.0,
    "flange_bottom_z_m": -TRAY["rim_downward_flange_height_m"],
    "four_finger_below_flange_clearance_m": 0.004,
    "four_finger_contact_center_z_m": -(TRAY["rim_downward_flange_height_m"] + 0.004),
    "four_finger_contact_center_z_range_m": [-0.015, -0.010],
    "thumb_inside_contact_center_z_m": -0.004,
    "thumb_inside_contact_center_z_range_m": [-0.006, -0.002],
    "recommended_primary_support_fingers": ["index", "middle"],
    "recommended_secondary_stabilizers": ["ring", "pinky"],
}

LEFT_TIPS = {
    "thumb": "hand_l_thumb_tip", "index": "hand_l_index_tip",
    "middle": "hand_l_middle_tip", "ring": "hand_l_ring_tip",
    "pinky": "hand_l_pinky_tip",
}
RIGHT_TIPS = {
    "thumb": "hand_r_thumb_tip", "index": "hand_r_index_tip",
    "middle": "hand_r_middle_tip", "ring": "hand_r_ring_tip",
    "pinky": "hand_r_pinky_tip",
}
MIRROR_SIGN = [-1.0, -1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0]


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


def unwrap_transform(result: Any) -> Any:
    if isinstance(result, (tuple, list)):
        if not result or result[0] is None:
            raise ValueError("Invalid transform result")
        return result[0]
    return result


def lookup_position(tf_api: Any, parent: str, child: str) -> np.ndarray:
    if not tf_api.can_transform(parent, child):
        raise RuntimeError("can_transform=False: %s -> %s" % (parent, child))
    obj = unwrap_transform(tf_api.lookup_transform_latest(parent, child))
    tf_obj = getattr(obj, "transform", obj)
    tr = getattr(tf_obj, "translation")
    return np.array([float(tr.x), float(tr.y), float(tr.z)], dtype=float)


def parse_end_state(end: Dict[str, Any], side: str) -> Dict[str, Any]:
    names = [str(name).strip() for name in end["names"]]
    raw_names = [str(name) for name in end["names"]]
    states = list(end["end_states"])
    if len(names) != 10 or len(states) != 10:
        raise ValueError("%s O10 state count is not 10" % side)
    joints = []
    for index, (name, raw_name, state) in enumerate(zip(names, raw_names, states)):
        joints.append({
            "index": index, "name": name, "raw_name": raw_name,
            "position": float(state["position"]),
            "enable": bool(state["enable"]),
            "status": state["status"], "err_code": state["err_code"],
        })
    return {
        "side": side, "controlled": bool(end["controlled"]),
        "type": end["type"], "joints": joints,
        "positions": [joint["position"] for joint in joints],
    }


def current_geometry(tf_api: Any, base: str, tips: Dict[str, str]) -> Dict[str, Any]:
    points = {name: lookup_position(tf_api, base, frame) for name, frame in tips.items()}
    four_center = np.mean(np.vstack([points[n] for n in ("index", "middle", "ring", "pinky")]), axis=0)
    primary_center = np.mean(np.vstack([points[n] for n in ("index", "middle")]), axis=0)
    thumb = points["thumb"]
    virtual_center = 0.5 * (thumb + four_center)
    return {
        "tip_positions_m": {k: v.tolist() for k, v in points.items()},
        "four_finger_center_m": four_center.tolist(),
        "primary_index_middle_center_m": primary_center.tolist(),
        "thumb_center_m": thumb.tolist(),
        "virtual_grasp_center_m": virtual_center.tolist(),
        "thumb_to_four_center_distance_m": float(np.linalg.norm(thumb - four_center)),
    }


def compute_tray_geometry() -> Dict[str, float]:
    length_inset = (TRAY["top_outer_length_m"] - TRAY["bottom_contact_length_m"]) / 2.0
    width_inset = (TRAY["top_outer_width_m"] - TRAY["bottom_contact_width_m"]) / 2.0
    return {
        "length_direction_inset_per_side_m": length_inset,
        "width_direction_inset_per_side_m": width_inset,
        "length_wall_angle_from_vertical_deg": math.degrees(math.atan2(length_inset, TRAY["overall_height_m"])),
        "width_wall_angle_from_vertical_deg": math.degrees(math.atan2(width_inset, TRAY["overall_height_m"])),
        "long_side_straight_section_m": TRAY["top_outer_length_m"] - 2.0 * TRAY["rim_corner_radius_m"],
        "short_side_straight_section_m": TRAY["top_outer_width_m"] - 2.0 * TRAY["rim_corner_radius_m"],
    }


def make_stages() -> Dict[str, Any]:
    return {
        "OPEN": {
            "purpose": "clear the 12 mm rim and pose uncertainty",
            "contact_required": False,
            "joint_targets": None,
            "status": "GEOMETRY_DEFINED_JOINT_TARGETS_PENDING",
        },
        "PREGRASP": {
            "thumb_target_z_range_m": [-0.006, -0.002],
            "four_finger_target_z_range_m": [-0.015, -0.010],
            "contact_required": False,
            "joint_targets": None,
            "status": "GEOMETRY_DEFINED_JOINT_TARGETS_PENDING",
        },
        "CONTACT": {
            "thumb_target_z_m": CONTACT["thumb_inside_contact_center_z_m"],
            "four_finger_target_z_m": CONTACT["four_finger_contact_center_z_m"],
            "four_finger_below_flange": True,
            "contact_required": True,
            "joint_targets": None,
            "status": "GEOMETRY_DEFINED_JOINT_TARGETS_PENDING",
        },
        "GRASP": {
            "support_mode": "mechanical hook under downward flange plus light normal preload",
            "primary_support": ["index", "middle"],
            "secondary_stabilization": ["ring", "pinky"],
            "joint_targets": None,
            "status": "BLOCKED_UNTIL_CONTACT_POSE_OR_OFFLINE_KINEMATIC_SEARCH",
        },
    }


def main() -> int:
    ensure_env()
    print("=" * 104)
    print("G2 O10 Tray Grasp Geometry Dry Run v2")
    print("Version             :", PROGRAM_VERSION)
    print("Strategy            : thumb inside + four fingers outside/below flange")
    print("READ_ONLY_PROBE     :", READ_ONLY_PROBE)
    print("ENABLE_REAL_MOTION  :", ENABLE_REAL_MOTION)
    print("Motion API          : NONE")
    print("=" * 104)
    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION:
        return 90

    import agibot_gdk
    initialized = False
    try:
        init_result = agibot_gdk.gdk_init()
        print("[GDK] gdk_init result   :", init_result)
        if init_result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("gdk_init failed: %s" % init_result)
        initialized = True
        robot = agibot_gdk.Robot()
        tf_api = agibot_gdk.TF()
        time.sleep(WAIT_SEC)

        raw_end = robot.get_end_state()
        left = parse_end_state(raw_end["left_end_state"], "left")
        right = parse_end_state(raw_end["right_end_state"], "right")
        left_geometry = current_geometry(tf_api, "hand_l_base_link", LEFT_TIPS)
        right_geometry = current_geometry(tf_api, "hand_r_base_link", RIGHT_TIPS)
        tray_geometry = compute_tray_geometry()
        stages = make_stages()

        right_normalized = [sign * position for sign, position in zip(MIRROR_SIGN, right["positions"])]
        mirror_delta = [l - r for l, r in zip(left["positions"], right_normalized)]

        print()
        print("[TRAY GEOMETRY]")
        print("  Top                  : 440 x 320 mm")
        print("  Bottom contact       : 377 x 263 mm")
        print("  Height               : 100 mm")
        print("  Rim width/thickness  : 12 / 1 mm")
        print("  Downward flange      : 8 mm")
        print("  Long-direction wall angle from vertical : %.3f deg" % tray_geometry["length_wall_angle_from_vertical_deg"])
        print("  Short-direction wall angle from vertical: %.3f deg" % tray_geometry["width_wall_angle_from_vertical_deg"])

        print()
        print("[CONTACT Z DEFINITION, tray rim top = 0]")
        print("  Flange bottom                  : %.1f mm" % (1000 * CONTACT["flange_bottom_z_m"]))
        print("  Four-finger nominal center     : %.1f mm" % (1000 * CONTACT["four_finger_contact_center_z_m"]))
        print("  Four-finger allowed range      : %.1f to %.1f mm" % (
            1000 * CONTACT["four_finger_contact_center_z_range_m"][0],
            1000 * CONTACT["four_finger_contact_center_z_range_m"][1]))
        print("  Thumb nominal center           : %.1f mm" % (1000 * CONTACT["thumb_inside_contact_center_z_m"]))
        print("  NOTE: four-finger center is below -8 mm by 4 mm nominal clearance.")

        for side, geometry in (("LEFT", left_geometry), ("RIGHT", right_geometry)):
            print()
            print("[%s CURRENT HAND GEOMETRY]" % side)
            print("  thumb center          :", geometry["thumb_center_m"])
            print("  index+middle center   :", geometry["primary_index_middle_center_m"])
            print("  four-finger center    :", geometry["four_finger_center_m"])
            print("  virtual grasp center  :", geometry["virtual_grasp_center_m"])
            print("  current aperture [mm] : %.3f" % (1000 * geometry["thumb_to_four_center_distance_m"]))

        report = {
            "version": PROGRAM_VERSION,
            "timestamp": datetime.now().astimezone().isoformat(),
            "safety": {"read_only": True, "o10_commanded": False, "motion_executed": False},
            "tray": TRAY,
            "derived_tray_geometry": tray_geometry,
            "contact_definition": CONTACT,
            "stages": stages,
            "left_end_state": left,
            "right_end_state": right,
            "mirror_sign": MIRROR_SIGN,
            "mirror_delta_rad": mirror_delta,
            "left_current_geometry": left_geometry,
            "right_current_geometry": right_geometry,
        }
        REPORT_DIR.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = REPORT_DIR / ("106_tray_geometry_contact_dry_run_v2_%s.json" % stamp)
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

        print()
        print("=" * 104)
        print("DRY-RUN SUMMARY")
        print("=" * 104)
        print("Tray dimensions complete       : PASS")
        print("Wall angles calculated         : PASS")
        print("Flange-underhook Z defined     : PASS")
        print("OPEN/PREGRASP/CONTACT geometry : PASS")
        print("O10 joint targets              : PENDING, NOT GUESSED")
        print("O10 commanded                  : NO")
        print("Motion executed                : NO")
        print("JSON report                    :", report_path)
        print("=" * 104)
        return 0
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
