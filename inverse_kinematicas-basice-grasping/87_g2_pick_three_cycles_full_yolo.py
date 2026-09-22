#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
87_g2_pick_three_cycles_full_yolo.py

直接运行入口，基于同目录中已验证的 86_g2_pick_three_cycles.py。
只做以下增强：
1. 三循环之间取消 READY，固定等待 INTER_CYCLE_WAIT_S 后自动继续。
2. 腰部速度提高到 0.60 rad/s，每段最大 40deg，80deg 正反转各2段。
3. YOLO窗口放大并在识别后持续开启到程序结束。
4. 画面显示完整状态：置信度、样本数、产品XYZ、桌面Z、产品高度、
   PreGrasp/Grasp、抓取高度来源、产品方向、循环次数及当前阶段。
5. 不修改86版的Waypoint、抓取高度、视觉算法、固定锚点、安全门限和三循环逻辑。
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys
import threading
from pathlib import Path
from typing import Any, Dict, List

import cv2
import numpy as np

BASE_FILE = Path(__file__).with_name("86_g2_pick_three_cycles.py")
if not BASE_FILE.exists():
    raise RuntimeError(
        f"未找到基础程序: {BASE_FILE}\n"
        "请将87版与已验证的86_g2_pick_three_cycles.py放在同一目录。"
    )

spec = importlib.util.spec_from_file_location("g2_cycle86", str(BASE_FILE))
if spec is None or spec.loader is None:
    raise RuntimeError("无法加载86版程序")
g2 = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = g2
spec.loader.exec_module(g2)

# -----------------------------------------------------------------------------
# 87版配置，只覆盖显示、腰部速度和循环等待；不动抓取参数。
# -----------------------------------------------------------------------------
g2.WAIT_FOR_PRODUCT_CONFIRMATION = False
g2.INTER_CYCLE_WAIT_S = 8.0

g2.WAIST_MAX_STEP_DEG = 40.0
g2.WAIST_SPEED_RAD_S = 0.60

g2.SHOW_YOLO_WINDOW = False  # 禁止86版额外创建第二个OpenCV窗口
UI_WINDOW_NAME = "G2 YOLO Full Process"
UI_WINDOW_WIDTH = 1500
UI_WINDOW_HEIGHT = 900
UI_PANEL_WIDTH = 470
UI_REFRESH_INTERVAL_S = 0.04
UI_IMAGE_FILE = "g2_cycle87_full_yolo_live.jpg"

UI_LOCK = threading.RLock()
UI_STATE: Dict[str, Any] = {
    "frame": None,
    "stage": "INITIALIZING",
    "cycle": 0,
    "cycle_count": int(getattr(g2, "CYCLE_COUNT", 3)),
    "conf": None,
    "position_samples": 0,
    "angle_samples": 0,
    "product_xyz": None,
    "table_z": None,
    "measured_height": None,
    "effective_height": None,
    "height_source": "-",
    "pregrasp_xyz": None,
    "grasp_xyz": None,
    "grasp_source": "-",
    "product_yaw_deg": None,
    "message": "Waiting for camera",
}


def _fmt(value: Any, digits: int = 3, suffix: str = "") -> str:
    if value is None:
        return "-"
    try:
        return f"{float(value):.{digits}f}{suffix}"
    except Exception:
        return str(value)


def _put(canvas: np.ndarray, text: str, x: int, y: int,
         color=(230, 230, 230), scale: float = 0.56,
         thickness: int = 1) -> None:
    cv2.putText(
        canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
        scale, color, thickness, cv2.LINE_AA,
    )


def _stage_group(stage: str) -> str:
    s = stage.lower()
    if "waypoint_1" in s:
        return "WAYPOINT 1"
    if "waypoint_2" in s:
        return "WAYPOINT 2"
    if "detect" in s or "yolo" in s:
        return "YOLO DETECTION"
    if "orientation" in s or "vertical" in s or "angle" in s:
        return "ORIENTATION ALIGN"
    if "coarse" in s:
        return "COARSE XY ALIGN"
    if "fine" in s:
        return "FINE XY ALIGN"
    if "pregrasp" in s:
        return "PREGRASP"
    if "descend_grasp" in s:
        return "DESCEND TO GRASP"
    if "gripper_open" in s:
        return "GRIPPER OPEN"
    if "gripper_close" in s:
        return "GRIPPER CLOSE"
    if "lift" in s:
        return "LIFT 10 CM"
    if "腰部右转" in stage:
        return "WAIST TURN TO DROP"
    if "腰部反转" in stage:
        return "WAIST RETURN"
    if "release" in s:
        return "RELEASE PRODUCT"
    if "wait" in s:
        return "WAIT NEXT PRODUCT"
    return stage.upper()


def _render() -> None:
    with UI_LOCK:
        frame = UI_STATE.get("frame")
        state = dict(UI_STATE)
    if frame is None:
        frame = np.zeros((720, 960, 3), dtype=np.uint8)
        _put(frame, "Waiting for YOLO frame...", 40, 70, (0, 220, 255), 0.8, 2)

    panel_w = UI_PANEL_WIDTH
    image_w = UI_WINDOW_WIDTH - panel_w
    image_h = UI_WINDOW_HEIGHT
    ih, iw = frame.shape[:2]
    scale = min(image_w / max(1, iw), image_h / max(1, ih))
    resized = cv2.resize(
        frame,
        (max(1, int(iw * scale)), max(1, int(ih * scale))),
        interpolation=cv2.INTER_LINEAR,
    )
    canvas = np.zeros((UI_WINDOW_HEIGHT, UI_WINDOW_WIDTH, 3), dtype=np.uint8)
    x0 = (image_w - resized.shape[1]) // 2
    y0 = (image_h - resized.shape[0]) // 2
    canvas[y0:y0 + resized.shape[0], x0:x0 + resized.shape[1]] = resized

    # Right information panel.
    px = image_w
    canvas[:, px:] = (24, 24, 24)
    cv2.line(canvas, (px, 0), (px, UI_WINDOW_HEIGHT), (80, 80, 80), 2)
    x = px + 24
    y = 42
    _put(canvas, "G2 YOLO PICK CYCLE", x, y, (0, 220, 255), 0.72, 2)
    y += 34
    _put(
        canvas,
        f"CYCLE  {state['cycle']}/{state['cycle_count']}",
        x, y, (90, 255, 90), 0.66, 2,
    )
    y += 32
    _put(canvas, f"STAGE  {_stage_group(str(state['stage']))}", x, y,
         (90, 220, 255), 0.55, 1)
    y += 34
    cv2.line(canvas, (x, y), (UI_WINDOW_WIDTH - 24, y), (75, 75, 75), 1)

    sections: List[tuple[str, List[str]]] = [
        ("DETECTION", [
            f"CONF             {_fmt(state['conf'], 3)}",
            f"POSITION SAMPLES {state['position_samples']}/{getattr(g2, 'DETECTION_SAMPLES', 8)}",
            f"ANGLE SAMPLES    {state['angle_samples']}/{getattr(g2, 'MIN_ANGLE_SAMPLES', 3)}",
            f"PRODUCT YAW      {_fmt(state['product_yaw_deg'], 1, ' deg')}",
        ]),
        ("PRODUCT BASE XYZ", [
            f"X  {_fmt((state['product_xyz'] or [None]*3)[0], 5, ' m')}",
            f"Y  {_fmt((state['product_xyz'] or [None]*3)[1], 5, ' m')}",
            f"Z  {_fmt((state['product_xyz'] or [None]*3)[2], 5, ' m')}",
        ]),
        ("HEIGHT MODEL", [
            f"TABLE Z          {_fmt(state['table_z'], 5, ' m')}",
            f"MEASURED HEIGHT  {_fmt(None if state['measured_height'] is None else state['measured_height']*1000, 1, ' mm')}",
            f"EFFECTIVE HEIGHT {_fmt(None if state['effective_height'] is None else state['effective_height']*1000, 1, ' mm')}",
            f"HEIGHT SOURCE    {state['height_source']}",
        ]),
        ("TARGETS", [
            f"PREGRASP Z       {_fmt((state['pregrasp_xyz'] or [None]*3)[2], 5, ' m')}",
            f"GRASP Z          {_fmt((state['grasp_xyz'] or [None]*3)[2], 5, ' m')}",
            f"GRASP SOURCE     {state['grasp_source']}",
        ]),
    ]

    for title, lines in sections:
        y += 30
        _put(canvas, title, x, y, (255, 190, 80), 0.55, 1)
        y += 25
        for line in lines:
            _put(canvas, line, x, y, (225, 225, 225), 0.49, 1)
            y += 23

    y += 18
    cv2.line(canvas, (x, y), (UI_WINDOW_WIDTH - 24, y), (75, 75, 75), 1)
    y += 27
    _put(canvas, "STATUS", x, y, (255, 190, 80), 0.55, 1)
    y += 25
    message = str(state.get("message", ""))[:55]
    _put(canvas, message, x, y, (180, 230, 255), 0.47, 1)
    _put(canvas, "Press Q or ESC: close display only", x, UI_WINDOW_HEIGHT - 28,
         (140, 140, 140), 0.42, 1)

    cv2.imwrite(UI_IMAGE_FILE, canvas)
    try:
        cv2.imshow(UI_WINDOW_NAME, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            cv2.destroyWindow(UI_WINDOW_NAME)
    except cv2.error as exc:
        print(f"[87 YOLO窗口] {exc}")


def _set_stage(stage: str, message: str = "") -> None:
    with UI_LOCK:
        UI_STATE["stage"] = stage
        if message:
            UI_STATE["message"] = message
    _render()


# Preserve original calls before patching.
_original_show_frame = g2.show_frame
_original_detect_product = g2.detect_product
_original_move_arm_once = g2.move_arm_once
_original_ensure_head = g2.ensure_head
_original_move_tcp_closed_loop = g2.move_tcp_closed_loop
_original_align_orientation_target = g2.align_orientation_target
_original_set_gripper = g2.set_gripper
_original_move_waist = g2.move_waist_segmented_keep_arms
_original_turn_release = g2.turn_release_and_return_waist


def show_frame_87(frame, lines):
    conf = None
    pos = None
    angle = None
    for line in lines or []:
        text = str(line).strip()
        lower = text.lower()
        try:
            if lower.startswith("conf="):
                conf = float(text.split("=", 1)[1])
            elif lower.startswith("position="):
                pos = int(text.split("=", 1)[1].split("/", 1)[0])
            elif lower.startswith("angle="):
                angle = int(text.split("=", 1)[1].split("/", 1)[0])
        except Exception:
            pass
    with UI_LOCK:
        UI_STATE["frame"] = frame.copy()
        if conf is not None:
            UI_STATE["conf"] = conf
        if pos is not None:
            UI_STATE["position_samples"] = pos
        if angle is not None:
            UI_STATE["angle_samples"] = angle
        UI_STATE["message"] = "YOLO target sampling"
    _render()


def detect_product_87(model, camera, tf_api, report):
    with UI_LOCK:
        UI_STATE["cycle"] = int(report.get("current_cycle_index", 0))
    _set_stage("YOLO_DETECTION", "Detecting product center, angle and depth")
    result = _original_detect_product(model, camera, tf_api, report)
    product = result.get("base_xyz")
    table_z = result.get("table_z_m")
    measured = None
    effective = None
    height_source = "-"
    pre = None
    grasp = None
    grasp_source = "-"
    if product is not None and table_z is not None:
        measured = float(product[2] - table_z)
        if (
            g2.PRODUCT_HEIGHT_VALID_MIN_M
            <= measured
            <= g2.PRODUCT_HEIGHT_VALID_MAX_M
        ):
            effective = measured
            height_source = "measured"
        else:
            effective = g2.PRODUCT_NOMINAL_HEIGHT_M
            height_source = "nominal_40mm"
        product_top = float(table_z + effective)
        table_floor = float(table_z + g2.MIN_TCP_ABOVE_TABLE_M)
        nominal_grasp = float(product_top + g2.GRASP_ABOVE_PRODUCT_TOP_M)
        grasp_z = max(nominal_grasp, table_floor)
        grasp_source = (
            "product_top+offset" if nominal_grasp >= table_floor
            else "table_safety_floor"
        )
        pre_z = max(
            product_top + g2.PREGRASP_ABOVE_PRODUCT_M,
            grasp_z + 0.060,
        )
        pre = [product[0], product[1], pre_z]
        grasp = [product[0], product[1], grasp_z]
    with UI_LOCK:
        UI_STATE.update({
            "product_xyz": product,
            "table_z": table_z,
            "measured_height": measured,
            "effective_height": effective,
            "height_source": height_source,
            "pregrasp_xyz": pre,
            "grasp_xyz": grasp,
            "grasp_source": grasp_source,
            "product_yaw_deg": result.get("yaw_deg"),
            "position_samples": result.get("position_samples", 0),
            "angle_samples": result.get("angle_samples", 0),
            "message": "Target locked for current pick cycle",
        })
    _render()
    return result


def move_arm_once_87(robot, tf_api, label, target, base, report):
    _set_stage(label, f"Moving right arm to {label}")
    return _original_move_arm_once(robot, tf_api, label, target, base, report)


def ensure_head_87(robot, report):
    _set_stage("HEAD_DOWN_20", "Head joint3 moving to +20 deg")
    return _original_ensure_head(robot, report)


def move_tcp_closed_loop_87(robot, tf_api, left, delta, q, base, report, label):
    _set_stage(label, f"TCP delta: {[round(float(v), 4) for v in delta]}")
    result = _original_move_tcp_closed_loop(
        robot, tf_api, left, delta, q, base, report, label
    )
    _render()
    return result


def align_orientation_target_87(robot, tf_api, left, target_q, base, report):
    _set_stage("VERTICAL_ORIENTATION", "Fixed-anchor vertical gripper alignment")
    result = _original_align_orientation_target(
        robot, tf_api, left, target_q, base, report
    )
    _render()
    return result


def set_gripper_87(robot, value):
    stage = "GRIPPER_OPEN" if float(value) < -0.3 else "GRIPPER_CLOSE"
    _set_stage(stage, f"Right gripper target: {float(value):.3f} rad")
    return _original_set_gripper(robot, value)


def move_waist_87(robot, report, target_waist, label, left_reference, right_reference):
    _set_stage(label, f"Waist target idx05: {math.degrees(target_waist[g2.WAIST_YAW_JOINT_INDEX]):.1f} deg")
    result = _original_move_waist(
        robot, report, target_waist, label, left_reference, right_reference
    )
    _render()
    return result


def turn_release_87(robot, report):
    _set_stage("WAIST_TURN_RELEASE_RETURN", "Turn, release product, then return waist")
    return _original_turn_release(robot, report)


# Apply runtime patches. The verified 86 control flow remains unchanged.
g2.show_frame = show_frame_87
g2.detect_product = detect_product_87
g2.move_arm_once = move_arm_once_87
g2.ensure_head = ensure_head_87
g2.move_tcp_closed_loop = move_tcp_closed_loop_87
g2.align_orientation_target = align_orientation_target_87
g2.set_gripper = set_gripper_87
g2.move_waist_segmented_keep_arms = move_waist_87
g2.turn_release_and_return_waist = turn_release_87


# Keep the UI responsive during the verified control code's sleeps without changing
# requested sleep durations. The proxy only refreshes the frozen/locked YOLO view.
_REAL_TIME = g2.time


class _TimeProxy:
    def __getattr__(self, name):
        return getattr(_REAL_TIME, name)

    @staticmethod
    def sleep(seconds):
        seconds = max(0.0, float(seconds))
        if seconds <= 0.0:
            _render()
            return
        end = _REAL_TIME.monotonic() + seconds
        while True:
            remaining = end - _REAL_TIME.monotonic()
            if remaining <= 0.0:
                break
            _render()
            _REAL_TIME.sleep(min(UI_REFRESH_INTERVAL_S, remaining))


g2.time = _TimeProxy()


if __name__ == "__main__":
    try:
        cv2.namedWindow(UI_WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(UI_WINDOW_NAME, UI_WINDOW_WIDTH, UI_WINDOW_HEIGHT)
        cv2.moveWindow(UI_WINDOW_NAME, 20, 20)
    except cv2.error as exc:
        print(f"[87 YOLO窗口创建失败] {exc}")
    _set_stage("STARTUP", "Loading verified three-cycle program")
    try:
        g2.main()
        _set_stage("COMPLETED", "All three cycles completed")
        _REAL_TIME.sleep(1.0)
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
