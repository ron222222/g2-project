#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
89_g2_pick_three_cycles_fixed_height_full_yolo.py

88版稳定控制逻辑的全程YOLO大屏增强入口。
依赖同目录：
  86_g2_pick_three_cycles.py
  88_g2_pick_three_cycles_fixed_height_shortest_axis.py

本文件不更改88版的Waypoint、固定40mm产品高度、锁定桌面Z、最短长轴方向、
抓取高度、闭环、安全门限、腰部和循环流程，只增强显示：
- 1600x960大窗口，左侧锁定YOLO图像，右侧完整状态面板；
- 从启动到程序退出始终保持窗口并刷新Qt事件；
- 每一轮YOLO重新识别后更新锁定画面和全部参数；
- 显示置信度、样本数、产品XYZ、原始/锁定桌面Z、固定产品高度、
  PreGrasp/Grasp、抓取来源、原始/选定长轴、姿态候选、当前阶段与循环。
"""

from __future__ import annotations

import importlib.util
import math
import sys
import threading
from pathlib import Path
from typing import Any, Dict, List

import cv2
import numpy as np

BASE88 = Path(__file__).with_name("88_g2_pick_three_cycles_fixed_height_shortest_axis.py")
if not BASE88.exists():
    raise RuntimeError(
        f"未找到88版: {BASE88}\n"
        "请将89版、88版和86版放在同一目录。"
    )

spec = importlib.util.spec_from_file_location("g2_v88_visual_base", str(BASE88))
if spec is None or spec.loader is None:
    raise RuntimeError("无法加载88版")
v88 = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = v88
spec.loader.exec_module(v88)
g2 = v88.g2

# 只关闭基础程序自己的小窗口，改用本文件唯一的大窗口。
g2.SHOW_YOLO_WINDOW = False

WINDOW_NAME = "G2 YOLO Full Process - Fixed Height 40mm"
WINDOW_WIDTH = 1600
WINDOW_HEIGHT = 960
PANEL_WIDTH = 520
REFRESH_INTERVAL_S = 0.04
FULL_VIEW_IMAGE_FILE = "g2_v89_full_yolo_view.jpg"

LOCK = threading.RLock()
STATE: Dict[str, Any] = {
    "frame": None,
    "cycle": 0,
    "cycle_count": int(getattr(g2, "CYCLE_COUNT", 3)),
    "stage": "STARTUP",
    "message": "Loading verified 88 control logic",
    "confidence": None,
    "position_samples": 0,
    "angle_samples": 0,
    "raw_product_xyz": None,
    "effective_product_xyz": None,
    "raw_table_z": None,
    "locked_table_z": None,
    "fixed_product_height": float(getattr(v88, "FIXED_PRODUCT_HEIGHT_M", 0.040)),
    "pregrasp_xyz": None,
    "grasp_xyz": None,
    "grasp_source": "-",
    "raw_yaw_deg": None,
    "selected_yaw_deg": None,
    "orientation_change_deg": None,
    "z_range_m": None,
    "waist_target_deg": None,
    "tcp_delta": None,
}


def fmt(value: Any, digits: int = 3, suffix: str = "") -> str:
    if value is None:
        return "-"
    try:
        return f"{float(value):.{digits}f}{suffix}"
    except Exception:
        return str(value)


def put(img: np.ndarray, text: str, x: int, y: int,
        color=(230, 230, 230), scale=0.52, thickness=1) -> None:
    cv2.putText(
        img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
        scale, color, thickness, cv2.LINE_AA,
    )


def stage_name(stage: str) -> str:
    s = str(stage).lower()
    mapping = [
        ("waypoint_1", "WAYPOINT 1"),
        ("waypoint_2", "WAYPOINT 2"),
        ("head", "HEAD +20 DEG"),
        ("yolo", "YOLO DETECTION"),
        ("coarse", "COARSE XY ALIGN"),
        ("vertical", "FIXED-ANCHOR ORIENTATION"),
        ("orientation", "FIXED-ANCHOR ORIENTATION"),
        ("fine", "FINE XY ALIGN"),
        ("pregrasp", "PREGRASP"),
        ("descend_grasp", "DESCEND TO GRASP"),
        ("gripper_open", "GRIPPER OPEN"),
        ("gripper_close", "GRIPPER CLOSE"),
        ("lift", "LIFT 10 CM"),
        ("release", "RELEASE PRODUCT"),
        ("wait", "WAIT NEXT PRODUCT"),
        ("completed", "COMPLETED"),
    ]
    if "腰部右转" in str(stage):
        return "WAIST TURN TO DROP"
    if "腰部反转" in str(stage):
        return "WAIST RETURN"
    for token, label in mapping:
        if token in s:
            return label
    return str(stage).upper()


def xyz_lines(name: str, xyz: Any) -> List[str]:
    values = xyz if isinstance(xyz, (list, tuple)) and len(xyz) >= 3 else [None, None, None]
    return [
        name,
        f"  X  {fmt(values[0], 5, ' m')}",
        f"  Y  {fmt(values[1], 5, ' m')}",
        f"  Z  {fmt(values[2], 5, ' m')}",
    ]


def render() -> None:
    with LOCK:
        state = dict(STATE)
        frame = None if STATE["frame"] is None else STATE["frame"].copy()

    if frame is None:
        frame = np.zeros((720, 960, 3), dtype=np.uint8)
        put(frame, "Waiting for YOLO image...", 45, 75, (0, 220, 255), 0.85, 2)

    image_width = WINDOW_WIDTH - PANEL_WIDTH
    ih, iw = frame.shape[:2]
    scale = min(image_width / max(iw, 1), WINDOW_HEIGHT / max(ih, 1))
    resized = cv2.resize(frame, (max(1, int(iw * scale)), max(1, int(ih * scale))))
    canvas = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8)
    ox = (image_width - resized.shape[1]) // 2
    oy = (WINDOW_HEIGHT - resized.shape[0]) // 2
    canvas[oy:oy + resized.shape[0], ox:ox + resized.shape[1]] = resized

    panel_x = image_width
    canvas[:, panel_x:] = (22, 22, 22)
    cv2.line(canvas, (panel_x, 0), (panel_x, WINDOW_HEIGHT), (90, 90, 90), 2)
    x = panel_x + 22
    y = 35
    put(canvas, "G2 YOLO FULL PROCESS", x, y, (0, 220, 255), 0.70, 2)
    y += 30
    put(canvas, "FIXED TABLE Z + PRODUCT HEIGHT 40 mm", x, y, (120, 255, 140), 0.47, 1)
    y += 30
    put(canvas, f"CYCLE  {state['cycle']}/{state['cycle_count']}", x, y, (100, 255, 100), 0.62, 2)
    y += 28
    put(canvas, f"STAGE  {stage_name(state['stage'])}", x, y, (90, 220, 255), 0.50, 1)
    y += 22
    put(canvas, str(state["message"])[:64], x, y, (190, 220, 255), 0.43, 1)
    y += 18
    cv2.line(canvas, (x, y), (WINDOW_WIDTH - 18, y), (70, 70, 70), 1)

    entries: List[tuple[str, List[str]]] = [
        ("DETECTION", [
            f"CONF              {fmt(state['confidence'], 3)}",
            f"POSITION SAMPLES  {state['position_samples']}/{getattr(g2, 'DETECTION_SAMPLES', 8)}",
            f"ANGLE SAMPLES     {state['angle_samples']}/{getattr(g2, 'MIN_ANGLE_SAMPLES', 3)}",
            f"PRODUCT Z RANGE   {fmt(None if state['z_range_m'] is None else state['z_range_m']*1000, 1, ' mm')}",
        ]),
        ("HEIGHT REFERENCE", [
            f"RAW TABLE Z       {fmt(state['raw_table_z'], 5, ' m')}",
            f"LOCKED TABLE Z    {fmt(state['locked_table_z'], 5, ' m')}",
            f"PRODUCT HEIGHT    {fmt(state['fixed_product_height']*1000, 1, ' mm')}",
            "HEIGHT MODE       FIXED 40 mm",
        ]),
        ("PRODUCT AXIS", [
            f"RAW YAW           {fmt(state['raw_yaw_deg'], 2, ' deg')}",
            f"SELECTED YAW      {fmt(state['selected_yaw_deg'], 2, ' deg')}",
            f"ORIENTATION MOVE  {fmt(state['orientation_change_deg'], 2, ' deg')}",
            "AXIS MODE         180 deg equivalent",
        ]),
    ]

    for title, lines in entries:
        y += 25
        put(canvas, title, x, y, (255, 190, 80), 0.51, 1)
        y += 22
        for line in lines:
            put(canvas, line, x, y, (225, 225, 225), 0.45, 1)
            y += 20

    for line in xyz_lines("RAW PRODUCT BASE XYZ", state["raw_product_xyz"]):
        y += 20
        put(canvas, line, x, y, (255, 190, 80) if "XYZ" in line else (225, 225, 225), 0.45, 1)
    for line in xyz_lines("EFFECTIVE PRODUCT XYZ", state["effective_product_xyz"]):
        y += 20
        put(canvas, line, x, y, (255, 190, 80) if "XYZ" in line else (225, 225, 225), 0.45, 1)

    y += 25
    put(canvas, "TARGETS", x, y, (255, 190, 80), 0.51, 1)
    y += 22
    pre = state["pregrasp_xyz"] or [None, None, None]
    grasp = state["grasp_xyz"] or [None, None, None]
    for line in [
        f"PREGRASP Z        {fmt(pre[2], 5, ' m')}",
        f"GRASP Z           {fmt(grasp[2], 5, ' m')}",
        f"GRASP SOURCE      {state['grasp_source']}",
        f"WAIST TARGET      {fmt(state['waist_target_deg'], 1, ' deg')}",
        f"TCP DELTA         {state['tcp_delta'] or '-'}",
    ]:
        put(canvas, line, x, y, (225, 225, 225), 0.43, 1)
        y += 20

    put(canvas, "Q / ESC closes display only", x, WINDOW_HEIGHT - 20, (130, 130, 130), 0.40, 1)
    try:
        cv2.imwrite(FULL_VIEW_IMAGE_FILE, canvas)
        cv2.imshow(WINDOW_NAME, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            cv2.destroyWindow(WINDOW_NAME)
    except cv2.error as exc:
        print(f"[89 YOLO窗口] {exc}")


def set_stage(stage: str, message: str = "") -> None:
    with LOCK:
        STATE["stage"] = stage
        if message:
            STATE["message"] = message
    render()


# 保留88版已经修正过的真实函数。
orig_show_frame = g2.show_frame
orig_detect_product = g2.detect_product
orig_move_arm_once = g2.move_arm_once
orig_ensure_head = g2.ensure_head
orig_move_tcp = g2.move_tcp_closed_loop
orig_align = g2.align_orientation_target
orig_gripper = g2.set_gripper
orig_waist = g2.move_waist_segmented_keep_arms
orig_turn_release = g2.turn_release_and_return_waist


def show_frame_89(frame, lines):
    conf = None
    pos = None
    angle = None
    for item in lines or []:
        text = str(item).strip()
        try:
            if text.lower().startswith("conf="):
                conf = float(text.split("=", 1)[1])
            elif text.lower().startswith("position="):
                pos = int(text.split("=", 1)[1].split("/", 1)[0])
            elif text.lower().startswith("angle="):
                angle = int(text.split("=", 1)[1].split("/", 1)[0])
        except Exception:
            pass
    with LOCK:
        STATE["frame"] = frame.copy()
        if conf is not None:
            STATE["confidence"] = conf
        if pos is not None:
            STATE["position_samples"] = pos
        if angle is not None:
            STATE["angle_samples"] = angle
        STATE["message"] = "Sampling product, angle, depth and table plane"
    render()


def detect_product_89(model, camera, tf_api, report):
    with LOCK:
        STATE["cycle"] = int(report.get("current_cycle_index", 0))
    set_stage("YOLO_DETECTION", "New YOLO detection for current cycle")
    data = orig_detect_product(model, camera, tf_api, report)
    raw_xyz = list(data.get("base_xyz", [None, None, None]))
    # 88版保留了修正前的Z和raw table Z。
    raw_xyz_display = list(raw_xyz)
    if data.get("raw_product_z_m") is not None:
        raw_xyz_display[2] = data.get("raw_product_z_m")
    fixed_table = data.get("table_z_m")
    fixed_height = float(data.get("fixed_product_height_m", 0.040))
    effective_xyz = list(data.get("base_xyz", raw_xyz))
    top_z = None if fixed_table is None else float(fixed_table) + fixed_height
    table_floor = None if fixed_table is None else float(fixed_table) + float(g2.MIN_TCP_ABOVE_TABLE_M)
    nominal_grasp = None if top_z is None else top_z + float(g2.GRASP_ABOVE_PRODUCT_TOP_M)
    grasp_z = None if nominal_grasp is None else max(nominal_grasp, table_floor)
    pre_z = None if top_z is None else max(top_z + float(g2.PREGRASP_ABOVE_PRODUCT_M), grasp_z + 0.060)
    candidates = data.get("axis_equivalent_candidates", [])
    selected_move = None
    if candidates:
        selected_yaw = data.get("yaw_deg")
        for candidate in candidates:
            if abs(float(candidate.get("yaw_deg", 1e9)) - float(selected_yaw)) < 1e-5:
                selected_move = candidate.get("orientation_change_deg")
                break
    with LOCK:
        STATE.update({
            "position_samples": data.get("position_samples", 0),
            "angle_samples": data.get("angle_samples", 0),
            "raw_product_xyz": raw_xyz_display,
            "effective_product_xyz": effective_xyz,
            "raw_table_z": data.get("raw_table_z_m"),
            "locked_table_z": fixed_table,
            "fixed_product_height": fixed_height,
            "pregrasp_xyz": [effective_xyz[0], effective_xyz[1], pre_z],
            "grasp_xyz": [effective_xyz[0], effective_xyz[1], grasp_z],
            "grasp_source": "product_top+offset" if nominal_grasp is not None and nominal_grasp >= table_floor else "table_safety_floor",
            "raw_yaw_deg": data.get("raw_yaw_deg", data.get("yaw_deg")),
            "selected_yaw_deg": data.get("yaw_deg"),
            "orientation_change_deg": selected_move,
            "z_range_m": data.get("product_z_range_m"),
            "message": "Target locked; frozen image retained for motion stages",
        })
    render()
    return data


def move_arm_once_89(robot, tf_api, label, target, base, report):
    set_stage(label, f"Moving right arm to {label}")
    return orig_move_arm_once(robot, tf_api, label, target, base, report)


def ensure_head_89(robot, report):
    set_stage("HEAD_20", "Head joint3 to +20 deg")
    return orig_ensure_head(robot, report)


def move_tcp_89(robot, tf_api, left, delta, q, base, report, label):
    with LOCK:
        STATE["tcp_delta"] = str([round(float(v), 4) for v in delta])
    set_stage(label, "Closed-loop TCP motion")
    result = orig_move_tcp(robot, tf_api, left, delta, q, base, report, label)
    render()
    return result


def align_89(robot, tf_api, left, target_q, base, report):
    set_stage("VERTICAL_ORIENTATION", "Fixed-anchor vertical orientation alignment")
    result = orig_align(robot, tf_api, left, target_q, base, report)
    render()
    return result


def gripper_89(robot, value):
    set_stage("GRIPPER_OPEN" if float(value) < -0.3 else "GRIPPER_CLOSE",
              f"Right gripper target {float(value):.3f} rad")
    return orig_gripper(robot, value)


def waist_89(robot, report, target_waist, label, left_reference, right_reference):
    target_deg = math.degrees(target_waist[g2.WAIST_YAW_JOINT_INDEX])
    with LOCK:
        STATE["waist_target_deg"] = target_deg
    set_stage(label, f"Waist idx05 target {target_deg:.1f} deg")
    result = orig_waist(robot, report, target_waist, label, left_reference, right_reference)
    render()
    return result


def turn_release_89(robot, report):
    set_stage("WAIST_TURN_RELEASE_RETURN", "Turn to drop, release, then return waist")
    return orig_turn_release(robot, report)


# 运行时增强，不复制、不改写88版控制代码。
g2.show_frame = show_frame_89
g2.detect_product = detect_product_89
g2.move_arm_once = move_arm_once_89
g2.ensure_head = ensure_head_89
g2.move_tcp_closed_loop = move_tcp_89
g2.align_orientation_target = align_89
g2.set_gripper = gripper_89
g2.move_waist_segmented_keep_arms = waist_89
g2.turn_release_and_return_waist = turn_release_89

# 在88/86已有的等待时间中持续刷新冻结YOLO画面，不改变运动等待时长。
REAL_TIME = g2.time


class TimeProxy:
    def __getattr__(self, name):
        return getattr(REAL_TIME, name)

    @staticmethod
    def sleep(seconds):
        seconds = max(0.0, float(seconds))
        if seconds <= 0:
            render()
            return
        end = REAL_TIME.monotonic() + seconds
        while True:
            remaining = end - REAL_TIME.monotonic()
            if remaining <= 0:
                break
            render()
            REAL_TIME.sleep(min(REFRESH_INTERVAL_S, remaining))


g2.time = TimeProxy()


if __name__ == "__main__":
    try:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT)
        cv2.moveWindow(WINDOW_NAME, 10, 10)
    except cv2.error as exc:
        print(f"[89 YOLO窗口创建失败] {exc}")
    set_stage("STARTUP", "Starting verified 88 logic with full-process display")
    try:
        g2.main()
        set_stage("COMPLETED", "All configured cycles completed")
        REAL_TIME.sleep(1.0)
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
