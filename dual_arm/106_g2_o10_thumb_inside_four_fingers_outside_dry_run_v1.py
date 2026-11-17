#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
106_g2_o10_thumb_inside_four_fingers_outside_dry_run_v1.py

O10 双手“拇指在盆沿内、四指在盆沿外”抓盆手型 Dry Run v1。

本版只做：
- 读取左右 O10 当前 10+10 DOF End State。
- 读取左右拇指/四指 fingertip TF。
- 归一化左右镜像关节符号，检查当前双手镜像一致性。
- 计算当前手型下的候选虚拟抓取几何中心。
- 生成 HOLD / OPEN / PREGRASP / GRASP 候选配置框架。
- 未提供并确认盆沿截面参数前，OPEN/PREGRASP/GRASP 不生成非零动作目标。

本版绝不调用末端控制、move_ee_pos、set_tcp_transform 或任何运动 API。
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
from typing import Any, Dict, List

import numpy as np

READ_ONLY_PROBE = True
ENABLE_REAL_MOTION = False
PROGRAM_VERSION = "106_v1_thumb_inside_four_outside_dry_run"
ENV_SH = Path.home() / ".cache/agibot/app/env.sh"
REPORT_DIR = Path(__file__).resolve().parent / "reports"
WAIT_SEC = 2.0

# 必须用真实测量值替换 None 后，后续版本才可产生抓盆目标。
TRAY_RIM_CONFIG = {
    "outer_width_m": None,
    "inner_width_m": None,
    "rim_thickness_m": None,
    "rim_height_m": None,
    "rim_corner_radius_m": None,
    "material": None,
    "empty_mass_kg": None,
    "loaded_mass_kg": None,
}

LEFT_TIPS = {
    "thumb": "hand_l_thumb_tip",
    "index": "hand_l_index_tip",
    "middle": "hand_l_middle_tip",
    "ring": "hand_l_ring_tip",
    "pinky": "hand_l_pinky_tip",
}
RIGHT_TIPS = {
    "thumb": "hand_r_thumb_tip",
    "index": "hand_r_index_tip",
    "middle": "hand_r_middle_tip",
    "ring": "hand_r_ring_tip",
    "pinky": "hand_r_pinky_tip",
}

# 按 get_end_state() 已验证顺序：左右各 10 DOF。
# mirror_sign 表示把右手转换到左手规范空间时的符号。
MIRROR_SIGN = [-1.0, -1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0]


def ensure_env() -> None:
    if os.environ.get("G2_GDK_ENV_READY") == "1":
        return
    if not ENV_SH.is_file():
        raise FileNotFoundError(str(ENV_SH))
    args = " ".join(shlex.quote(x) for x in [str(Path(__file__).resolve())] + sys.argv[1:])
    command = (
        "source %s && export G2_GDK_ENV_READY=1 && "
        "exec /usr/bin/python3.10 %s"
    ) % (shlex.quote(str(ENV_SH)), args)
    print("[ENV] Restarting with official GDK environment ...", flush=True)
    os.execv("/bin/bash", ["bash", "-lc", command])


def unwrap_transform(result: Any) -> Any:
    if isinstance(result, (tuple, list)):
        if not result or result[0] is None:
            raise ValueError("Invalid transform tuple/list")
        return result[0]
    return result


def translation(result: Any) -> np.ndarray:
    obj = unwrap_transform(result)
    tf_obj = getattr(obj, "transform", obj)
    tr = getattr(tf_obj, "translation")
    return np.array([float(tr.x), float(tr.y), float(tr.z)], dtype=float)


def lookup_position(tf_api: Any, parent: str, child: str) -> np.ndarray:
    if not tf_api.can_transform(parent, child):
        raise RuntimeError("can_transform=False: %s -> %s" % (parent, child))
    return translation(tf_api.lookup_transform_latest(parent, child))


def normalize_name(name: Any) -> str:
    return str(name).strip()


def parse_end(end: Dict[str, Any], side: str) -> Dict[str, Any]:
    raw_names = list(end.get("names", []))
    states = list(end.get("end_states", []))
    if len(raw_names) != 10 or len(states) != 10:
        raise ValueError("%s O10 expected 10 names/states, got %d/%d" % (side, len(raw_names), len(states)))
    joints = []
    for index, (raw_name, state) in enumerate(zip(raw_names, states)):
        joints.append({
            "index": index,
            "raw_name": str(raw_name),
            "name": normalize_name(raw_name),
            "id": state.get("id"),
            "enable": state.get("enable"),
            "position": float(state.get("position")),
            "velocity": float(state.get("velocity", 0.0)),
            "effort": float(state.get("effort", 0.0)),
            "current": float(state.get("current", 0.0)),
            "status": state.get("status"),
            "err_code": state.get("err_code"),
        })
    return {
        "side": side,
        "controlled": bool(end.get("controlled")),
        "type": end.get("type"),
        "joints": joints,
        "positions": [j["position"] for j in joints],
    }


def hand_geometry(tf_api: Any, base: str, tips: Dict[str, str]) -> Dict[str, Any]:
    points = {finger: lookup_position(tf_api, base, frame) for finger, frame in tips.items()}
    four = np.vstack([points[name] for name in ("index", "middle", "ring", "pinky")])
    outside_center = np.mean(four, axis=0)
    thumb = points["thumb"]
    virtual_center = 0.5 * (thumb + outside_center)
    aperture = float(np.linalg.norm(thumb - outside_center))
    return {
        "base_frame": base,
        "tip_positions_m": {k: v.tolist() for k, v in points.items()},
        "four_finger_outside_center_m": outside_center.tolist(),
        "thumb_inside_point_m": thumb.tolist(),
        "virtual_grasp_center_m": virtual_center.tolist(),
        "thumb_to_four_finger_center_aperture_m": aperture,
    }


def main() -> int:
    ensure_env()
    print("=" * 100)
    print("G2 O10 Thumb-Inside + Four-Fingers-Outside Dry Run")
    print("Version             :", PROGRAM_VERSION)
    print("READ_ONLY_PROBE     :", READ_ONLY_PROBE)
    print("ENABLE_REAL_MOTION  :", ENABLE_REAL_MOTION)
    print("Motion API          : NONE")
    print("=" * 100)
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

        raw = robot.get_end_state()
        left = parse_end(raw["left_end_state"], "left")
        right = parse_end(raw["right_end_state"], "right")

        left_geom = hand_geometry(tf_api, "hand_l_base_link", LEFT_TIPS)
        right_geom = hand_geometry(tf_api, "hand_r_base_link", RIGHT_TIPS)

        right_in_left_convention = [s * p for s, p in zip(MIRROR_SIGN, right["positions"])]
        mirror_delta = [l - r for l, r in zip(left["positions"], right_in_left_convention)]
        max_mirror_error = max(abs(v) for v in mirror_delta)

        tray_complete = all(value is not None for value in TRAY_RIM_CONFIG.values())
        current_pose = {
            "left": left["positions"],
            "right": right["positions"],
        }
        profiles = {
            "HOLD_CURRENT": {
                "status": "READY_DRY_RUN",
                "left_positions": current_pose["left"],
                "right_positions": current_pose["right"],
                "command_allowed": False,
            },
            "OPEN": {
                "status": "BLOCKED_NEEDS_MEASURED_RIM_AND_APPROVED_TARGETS",
                "left_positions": None,
                "right_positions": None,
                "command_allowed": False,
            },
            "PREGRASP": {
                "status": "BLOCKED_NEEDS_MEASURED_RIM_AND_APPROVED_TARGETS",
                "left_positions": None,
                "right_positions": None,
                "command_allowed": False,
            },
            "GRASP": {
                "status": "BLOCKED_NEEDS_MEASURED_RIM_AND_APPROVED_TARGETS",
                "left_positions": None,
                "right_positions": None,
                "command_allowed": False,
            },
        }

        print()
        print("[GRASP STRATEGY]")
        print("  Thumb              : inside tray rim")
        print("  Index/Middle/Ring/Pinky: outside tray rim")
        print("  Commands sent      : NO")
        print()
        print("[CURRENT MIRROR CHECK]")
        for index, (l, r, rn, delta) in enumerate(zip(left["positions"], right["positions"], right_in_left_convention, mirror_delta)):
            print("  %02d left=% .6f right=% .6f normalized_right=% .6f delta=% .6f" % (index, l, r, rn, delta))
        print("  max mirror error [rad]: %.6f" % max_mirror_error)

        for side, geometry in (("LEFT", left_geom), ("RIGHT", right_geom)):
            print()
            print("[%s CURRENT VIRTUAL GRASP GEOMETRY]" % side)
            print("  thumb inside point [m]      :", geometry["thumb_inside_point_m"])
            print("  four-finger outside center  :", geometry["four_finger_outside_center_m"])
            print("  virtual grasp center [m]    :", geometry["virtual_grasp_center_m"])
            print("  current aperture [m]        : %.6f" % geometry["thumb_to_four_finger_center_aperture_m"])

        report = {
            "version": PROGRAM_VERSION,
            "timestamp": datetime.now().astimezone().isoformat(),
            "safety": {
                "read_only": True,
                "o10_commanded": False,
                "motion_executed": False,
            },
            "strategy": "thumb_inside_four_fingers_outside",
            "tray_rim_config": TRAY_RIM_CONFIG,
            "tray_rim_config_complete": tray_complete,
            "left_end_state": left,
            "right_end_state": right,
            "mirror_sign": MIRROR_SIGN,
            "right_positions_in_left_convention": right_in_left_convention,
            "mirror_delta_rad": mirror_delta,
            "max_mirror_error_rad": max_mirror_error,
            "left_current_geometry": left_geom,
            "right_current_geometry": right_geom,
            "candidate_profiles": profiles,
        }
        REPORT_DIR.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = REPORT_DIR / ("106_thumb_inside_four_outside_dry_run_v1_%s.json" % stamp)
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

        print()
        print("=" * 100)
        print("DRY-RUN SUMMARY")
        print("=" * 100)
        print("End State left/right       : PASS")
        print("Fingertip geometry         : PASS")
        print("Grasp strategy             : thumb inside + four fingers outside")
        print("Tray rim data complete     :", tray_complete)
        print("OPEN/PREGRASP/GRASP target : BLOCKED" if not tray_complete else "OPEN/PREGRASP/GRASP target : READY FOR NEXT VERSION")
        print("O10 commanded              : NO")
        print("Motion executed            : NO")
        print("JSON report                :", report_path)
        print("=" * 100)
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
