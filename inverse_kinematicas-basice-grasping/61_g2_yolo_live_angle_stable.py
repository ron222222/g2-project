#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
61_g2_yolo_live_angle_stable.py

完全独立单文件版本，不import 52/53或其他本地程序。

流程：
1. 识别当前右臂是否在 HOME / WAYPOINT_1 / WAYPOINT_2 附近。
2. HOME -> WAYPOINT_1：一次 move_arm_joint() 到点，不分段。
3. WAYPOINT_1验证通过后，头部 idx13_head_joint3 移动到 +20度并闭环验证。
4. WAYPOINT_1 -> WAYPOINT_2：一次 move_arm_joint() 到点，不分段。
5. 在 WAYPOINT_2 执行 YOLO + 深度 + TF，得到产品中心 Base XYZ。
6. 在 YOLO框内估计长方形产品长轴方向。
7. 可选：高位角度对齐、高位XY对准、下降、夹取、抬升10cm。

安全设计：
- 默认所有真实动作关闭。
- 固定Waypoint每个只调用一次move_arm_joint()。
- 头部只在WAYPOINT_1验证成功后动作。
- YOLO中心定位与角度定位分离：中心有效但角度无效时会明确报告原因。
- 角度对齐采用多段闭环TCP旋转，降低一次大角度旋转的TCP漂移。
- 每个动作后读取关节/TF闭环验证。

运行环境：/usr/bin/python3.10，ROS2 Humble，agibot_gdk。
"""

import os

# OpenCV Qt字体修复。必须在import cv2之前执行。
# OpenCV当前Qt插件会固定查找 ~/.local/lib/python3.10/site-packages/cv2/qt/fonts。
# 本程序在该位置自动创建到系统DejaVu字体的符号链接，不修改PyCharm配置。
_SYSTEM_FONT_DIR = "/usr/share/fonts/truetype/dejavu"
_CV2_QT_FONT_DIR = os.path.expanduser(
    "~/.local/lib/python3.10/site-packages/cv2/qt/fonts"
)

def _prepare_cv2_qt_fonts_before_import() -> dict:
    result = {"system_font_dir": _SYSTEM_FONT_DIR,
              "cv2_qt_font_dir": _CV2_QT_FONT_DIR,
              "created": False, "linked_fonts": 0, "error": None}
    try:
        if os.path.isdir(_SYSTEM_FONT_DIR):
            os.makedirs(_CV2_QT_FONT_DIR, exist_ok=True)
            for name in os.listdir(_SYSTEM_FONT_DIR):
                if not name.lower().endswith((".ttf", ".otf")):
                    continue
                source = os.path.join(_SYSTEM_FONT_DIR, name)
                target = os.path.join(_CV2_QT_FONT_DIR, name)
                if not os.path.lexists(target):
                    os.symlink(source, target)
                result["linked_fonts"] += 1
            result["created"] = True
        os.environ["QT_QPA_FONTDIR"] = _CV2_QT_FONT_DIR
        os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "0")
    except Exception as exc:
        result["error"] = str(exc)
        # 即使字体链接失败，也不影响机器人主流程，仍保存JPEG画面。
        os.environ.setdefault("QT_QPA_FONTDIR", _SYSTEM_FONT_DIR)
    return result

QT_FONT_PREPARE_RESULT = _prepare_cv2_qt_fonts_before_import()

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import agibot_gdk
from ultralytics import YOLO

# =============================================================================
# 功能开关：首次运行请按阶段逐项开启
# =============================================================================
ENABLE_REAL_MOTION = True
ENABLE_ANGLE_ALIGNMENT = True
ENABLE_XY_AND_PREGRASP = True
ENABLE_GRIPPER_AND_PICK = True
REQUIRE_DANGEROUS_STAGE_CONFIRMATION = True

MODEL_PATH = "runs/detect/runs/product_detector/weights/best.pt"
REPORT_FILE = "g2_yolo_live_angle_stable_report.json"
ANGLE_DEBUG_IMAGE = "g2_product_angle_stable_debug.jpg"
LIVE_WINDOW_NAME = "G2 YOLO Live"
LIVE_LATEST_IMAGE = "g2_yolo_angle_stable_latest.jpg"
SHOW_YOLO_WINDOW = True
SAVE_LATEST_FRAME = True
DISPLAY_SCALE = 1.0
NO_DETECTION_SLEEP_S = 0.03
WINDOW_WAIT_KEY_MS = 20
KEEP_FINAL_YOLO_FRAME_OPEN = True
# 产品抓取点比59版降低30mm: +25mm -> -5mm。
GRASP_Z_CORRECTION_M = -0.030

# =============================================================================
# 坐标系与关节名称
# =============================================================================
LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"
TCP_FRAME = "gripper_r_center_link"
HEAD_FRAME = "head_link3"

RIGHT_JOINTS = [
    "idx61_arm_r_joint1",
    "idx62_arm_r_joint2",
    "idx63_arm_r_joint3",
    "idx64_arm_r_joint4",
    "idx65_arm_r_joint5",
    "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]
HEAD_JOINTS = [
    "idx11_head_joint1",
    "idx12_head_joint2",
    "idx13_head_joint3",
]

# =============================================================================
# VR实测关节姿态
# =============================================================================
HOME = [
    -1.57079643,
    -1.57079608,
     1.57079585,
    -1.57079645,
     0.00000024,
     0.00000000,
     0.00000036,
]

WAYPOINT_1 = [
    -1.91304851,
     0.20393955,
     1.83169374,
    -1.84651811,
     0.29203423,
    -0.28742294,
     0.91643847,
]

WAYPOINT_2 = [
    -2.34999915,
     1.00000018,
     2.39123385,
    -1.79048167,
    -1.50846278,
    -0.95681237,
    -0.11304288,
]

RECORDED_TCP = {
    "home": [0.73185, -0.24350, 0.81644],
    "waypoint_1": [0.58639, -0.34289, 0.93608],
    "waypoint_2": [0.65699, -0.16556, 0.98419],
}

# =============================================================================
# 固定Waypoint一次到点参数
# =============================================================================
DIRECT_ARM_SPEED_RAD_S = 0.20
JOINT_TARGET_TOLERANCE_RAD = 0.06
KEYPOINT_TCP_TOLERANCE_M = 0.050
START_POSE_TOLERANCE_RAD = 0.15
SETTLE_SECONDS = 1.0

JOINT_LIMITS = [
    (-3.071796, 3.071796),
    (-2.059505, 2.059505),
    (-3.071796, 3.071796),
    (-2.495838, 1.012308),
    (-3.071796, 3.071796),
    (-1.012308, 1.012308),
    (-1.535907, 1.535907),
]

# =============================================================================
# 头部参数：实测 +20度为正确低头方向
# =============================================================================
HEAD_TARGET_DEG = 20.0
HEAD_SPEED_RAD_S = 0.20
HEAD_TOLERANCE_DEG = 1.5
HEAD_SETTLE_SECONDS = 1.0

# =============================================================================
# YOLO、深度与角度识别参数
# =============================================================================
MIN_CONFIDENCE = 0.60
DETECTION_SAMPLES = 8
MIN_VALID_POSITION_SAMPLES = 4
MIN_VALID_ANGLE_SAMPLES = 3
DETECTION_TIMEOUT_S = 25.0
CAMERA_TIMEOUT_MS = 1000.0
DEPTH_RADIUS = 4
MIN_DEPTH_RAW = 50.0
MAX_DEPTH_RAW = 10000.0

# ROI角度提取
MIN_RECT_ASPECT_RATIO = 1.20
MIN_CONTOUR_AREA_RATIO = 0.025
MAX_CONTOUR_AREA_RATIO = 0.95
MAX_CONTOUR_CENTER_DISTANCE_RATIO = 0.42
ROI_PADDING_RATIO = 0.08
ANGLE_MAX_DEVIATION_DEG = 20.0

# =============================================================================
# 产品抓取与安全窗口
# =============================================================================
PREGRASP_ABOVE_PRODUCT_M = 0.120
GRASP_ABOVE_PRODUCT_M = -0.005
LIFT_AFTER_GRASP_M = 0.100
MAX_XY_FROM_WP2_M = 0.250
MAX_PREGRASP_DESCENT_M = 0.250
MIN_TCP_Z_M = 0.690

# 夹爪扁长指尖的长边对应局部轴；若现场相差约90度，改为"y"。
GRIPPER_LONG_AXIS_LOCAL = "y"
GRIPPER_YAW_OFFSET_DEG = 0.0
MAX_YAW_CORRECTION_DEG = 60.0

# TCP相对End的固定偏移
TCP_OFFSET_END_M = np.array([0.0, 0.0, 0.14308], dtype=float)

# =============================================================================
# 笛卡尔动作参数
# =============================================================================
RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
LIFE_TIME = 0.02
CART_STEP_M = 0.0015
TCP_POSITION_TOLERANCE_M = 0.025
ORIENTATION_TOLERANCE_DEG = 6.0

# 角度对齐：每段最多15度，每段内部再做四元数插值
MAX_YAW_PER_STAGE_DEG = 7.5
ROTATION_INTERPOLATION_STEP_DEG = 0.75
MAX_TCP_DRIFT_PER_ROTATION_STAGE_M = 0.025
ROTATION_FINAL_HOLD_SECONDS = 0.45
ROTATION_HOLD_RATE_HZ = 50.0
ROTATION_STAGE_SETTLE_SECONDS = 1.0

# =============================================================================
# 力矩安全
# =============================================================================
BASELINE_SAMPLES = 30
TORQUE_DELTA_LIMIT_NM = 30.0


@dataclass
class PoseData:
    position: List[float]
    orientation: List[float]


def save_report(report: dict) -> None:
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def normalize_q(q) -> List[float]:
    arr = np.asarray(q, dtype=float)
    norm = float(np.linalg.norm(arr))
    if norm < 1e-12:
        raise RuntimeError("四元数模长接近0")
    return (arr / norm).tolist()


def q_multiply(q1, q2) -> List[float]:
    x1, y1, z1, w1 = normalize_q(q1)
    x2, y2, z2, w2 = normalize_q(q2)
    return normalize_q([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def q_from_base_yaw(yaw_rad: float) -> List[float]:
    return [0.0, 0.0, math.sin(yaw_rad/2.0), math.cos(yaw_rad/2.0)]


def q_slerp(q0, q1, alpha: float) -> List[float]:
    a = np.asarray(normalize_q(q0), dtype=float)
    b = np.asarray(normalize_q(q1), dtype=float)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return normalize_q((1.0-alpha)*a + alpha*b)
    theta0 = math.acos(dot)
    sin_theta0 = math.sin(theta0)
    s0 = math.sin((1.0-alpha)*theta0) / sin_theta0
    s1 = math.sin(alpha*theta0) / sin_theta0
    return normalize_q(s0*a + s1*b)


def quaternion_error_deg(q1, q2) -> float:
    a = normalize_q(q1)
    b = normalize_q(q2)
    dot = abs(sum(x*y for x, y in zip(a, b)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def q_to_rotation(q) -> np.ndarray:
    x, y, z, w = normalize_q(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=float)


def transform_matrix(transform) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = q_to_rotation([
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    ])
    matrix[:3, 3] = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    return matrix


def distance(a, b) -> float:
    return math.sqrt(sum((float(b[i])-float(a[i]))**2 for i in range(3)))


def lerp_xyz(a, b, alpha: float) -> List[float]:
    return [(1.0-alpha)*a[i] + alpha*b[i] for i in range(3)]


def wrap_axis_pi(angle: float) -> float:
    while angle > math.pi/2.0:
        angle -= math.pi
    while angle < -math.pi/2.0:
        angle += math.pi
    return angle


def circular_axis_average(angles: List[float]) -> float:
    values = np.asarray(angles, dtype=float)
    return 0.5 * math.atan2(
        float(np.mean(np.sin(2.0*values))),
        float(np.mean(np.cos(2.0*values))),
    )


def axis_angle_difference_deg(a: float, b: float) -> float:
    return abs(math.degrees(wrap_axis_pi(a-b)))


def read_pose(tf_api, frame: str) -> PoseData:
    transform = tf_api.get_tf_from_base_link(frame)
    return PoseData(
        [
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
        ],
        normalize_q([
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]),
    )


def wait_pose(tf_api, frame: str, timeout_s: float = 10.0) -> PoseData:
    deadline = time.time() + timeout_s
    last_error = None
    while time.time() < deadline:
        try:
            if tf_api.can_transform("base_link", frame):
                return read_pose(tf_api, frame)
        except Exception as exc:
            last_error = exc
        time.sleep(0.2)
    raise RuntimeError(f"等待TF超时: {frame}; {last_error}")


def joint_state_map(robot) -> dict:
    return {state["name"]: state for state in robot.get_joint_states()["states"]}


def get_positions(robot, names: List[str]) -> List[float]:
    states = joint_state_map(robot)
    missing = [name for name in names if name not in states]
    if missing:
        raise RuntimeError(f"缺少关节状态: {missing}")
    return [float(states[name]["motor_position"]) for name in names]


def max_joint_error(a, b) -> float:
    return max(abs(float(x)-float(y)) for x, y in zip(a, b))


def closest_start_pose(current_right) -> Tuple[str, int, float]:
    candidates = [
        (max_joint_error(current_right, HOME), "home", 0),
        (max_joint_error(current_right, WAYPOINT_1), "waypoint_1", 1),
        (max_joint_error(current_right, WAYPOINT_2), "waypoint_2", 2),
    ]
    candidates.sort(key=lambda item: item[0])
    error, name, index = candidates[0]
    if error > START_POSE_TOLERANCE_RAD:
        raise RuntimeError(
            f"当前右臂不在HOME/WAYPOINT_1/WAYPOINT_2附近；"
            f"最近{name}，最大关节差={error:.3f}rad"
        )
    return name, index, error


def get_torques(robot) -> Dict[str, float]:
    return {
        state["name"]: float(state["effort"])
        for state in robot.get_joint_states()["states"]
    }


def establish_torque_baseline(robot) -> Dict[str, float]:
    print("建立力矩基线，请保持机器人静止...")
    samples = []
    for _ in range(BASELINE_SAMPLES):
        samples.append(get_torques(robot))
        time.sleep(0.05)
    names = set().union(*(sample.keys() for sample in samples))
    return {
        name: sum(sample.get(name, 0.0) for sample in samples)/len(samples)
        for name in names
    }


def verify_torque(robot, baseline: Dict[str, float], report: dict) -> None:
    current = get_torques(robot)
    abnormal = []
    for name, value in current.items():
        if name in baseline:
            delta = abs(value-baseline[name])
            if delta > TORQUE_DELTA_LIMIT_NM:
                abnormal.append({"joint": name, "delta": delta})
    if abnormal:
        report["status"] = "STOPPED_TORQUE"
        report["torque_abnormal"] = abnormal[:10]
        save_report(report)
        raise RuntimeError("力矩安全检查触发")


def validate_joint_target(label: str, target: List[float]) -> None:
    if len(target) != 7:
        raise RuntimeError(f"{label}右臂目标不是7个关节")
    for index, (value, limits) in enumerate(zip(target, JOINT_LIMITS)):
        if not limits[0] <= value <= limits[1]:
            raise RuntimeError(
                f"{label} {RIGHT_JOINTS[index]}={value:.6f}超出限位{limits}"
            )


def move_right_arm_once(
    robot,
    tf_api,
    label: str,
    target: List[float],
    baseline: Dict[str, float],
    report: dict,
) -> None:
    validate_joint_target(label, target)
    verify_torque(robot, baseline, report)

    print(f"[{label}] 一次到点，速度={DIRECT_ARM_SPEED_RAD_S:.2f}rad/s")
    result = robot.move_arm_joint(
        list(target),
        [DIRECT_ARM_SPEED_RAD_S]*7,
        1,
    )
    if result != 0:
        raise RuntimeError(f"{label} move_arm_joint失败: {result}")

    time.sleep(SETTLE_SECONDS)
    actual = get_positions(robot, RIGHT_JOINTS)
    joint_error = max_joint_error(actual, target)
    tcp = wait_pose(tf_api, TCP_FRAME)
    tcp_error = distance(tcp.position, RECORDED_TCP[label])

    report["joint_stages"].append({
        "stage": label,
        "mode": "single_move_arm_joint",
        "target": target,
        "actual": actual,
        "max_joint_error_rad": joint_error,
        "actual_tcp": tcp.__dict__,
        "recorded_tcp": RECORDED_TCP[label],
        "tcp_error_m": tcp_error,
    })
    save_report(report)

    print(
        f"[{label}验证] 最大关节误差={joint_error:.5f}rad, "
        f"TCP误差={tcp_error:.5f}m"
    )
    if joint_error > JOINT_TARGET_TOLERANCE_RAD:
        raise RuntimeError(f"{label}关节未到位")
    if tcp_error > KEYPOINT_TCP_TOLERANCE_M:
        raise RuntimeError(f"{label} TCP未复现")


def move_head_after_waypoint1(robot, report: dict) -> None:
    before = get_positions(robot, HEAD_JOINTS)
    target = [before[0], before[1], math.radians(HEAD_TARGET_DEG)]

    print("-"*78)
    print("[WAYPOINT_1已验证] 执行头部 idx13_head_joint3 = +20度")
    print(f"[头部] 当前(deg)={[round(math.degrees(v),3) for v in before]}")
    print(f"[头部] 目标(deg)={[round(math.degrees(v),3) for v in target]}")

    result = robot.move_head_joint(target, [HEAD_SPEED_RAD_S]*3)
    if result != 0:
        raise RuntimeError(f"move_head_joint失败: {result}")
    time.sleep(HEAD_SETTLE_SECONDS)

    after = get_positions(robot, HEAD_JOINTS)
    error_deg = abs(math.degrees(after[2])-HEAD_TARGET_DEG)
    report["head_after_waypoint_1"] = {
        "before_rad": before,
        "before_deg": [math.degrees(v) for v in before],
        "target_rad": target,
        "target_deg": [math.degrees(v) for v in target],
        "after_rad": after,
        "after_deg": [math.degrees(v) for v in after],
        "idx13_error_deg": error_deg,
    }
    save_report(report)

    print(f"[头部验证] 实际(deg)={[round(math.degrees(v),3) for v in after]}")
    print(f"[头部验证] idx13误差={error_deg:.3f}deg")
    if error_deg > HEAD_TOLERANCE_DEG:
        report["status"] = "STOPPED_HEAD_NOT_REACHED"
        save_report(report)
        raise RuntimeError("头部+20度未到位，禁止继续WAYPOINT_2和YOLO")
    print("[头部验证] +20度低头通过。")
    print("-"*78)


def ensure_head_at_positive20(robot, report: dict, source: str) -> None:
    current = get_positions(robot, HEAD_JOINTS)
    error_deg = abs(math.degrees(current[2])-HEAD_TARGET_DEG)
    if error_deg <= HEAD_TOLERANCE_DEG:
        report["head_check"] = {
            "source": source,
            "actual_deg": [math.degrees(v) for v in current],
            "idx13_error_deg": error_deg,
        }
        save_report(report)
        print(f"[头部检查] idx13已在+20度附近，误差={error_deg:.3f}deg")
        return
    move_head_after_waypoint1(robot, report)


def safe_get_image(camera, camera_type):
    try:
        return camera.get_latest_image(camera_type, CAMERA_TIMEOUT_MS)
    except Exception:
        return None


def decode_color(image):
    return cv2.imdecode(np.frombuffer(image.data, np.uint8), cv2.IMREAD_COLOR)


def decode_depth(image):
    try:
        dtype = np.float32 if image.bit_depth == 32 else np.uint16
        return np.frombuffer(image.data, dtype=dtype).reshape(
            (image.height, image.width)
        )
    except Exception:
        return None


def median_depth(depth, u: int, v: int) -> Optional[float]:
    height, width = depth.shape[:2]
    patch = depth[
        max(0, v-DEPTH_RADIUS):min(height, v+DEPTH_RADIUS+1),
        max(0, u-DEPTH_RADIUS):min(width, u+DEPTH_RADIUS+1),
    ].astype(float)
    valid = patch[np.isfinite(patch)]
    valid = valid[(valid > MIN_DEPTH_RAW) & (valid < MAX_DEPTH_RAW)]
    return None if valid.size == 0 else float(np.median(valid))


def color_to_depth_pixel(
    point: Tuple[float, float],
    color_shape,
    depth_shape,
) -> Tuple[int, int]:
    color_h, color_w = color_shape[:2]
    depth_h, depth_w = depth_shape[:2]
    u = int(round(point[0]*depth_w/color_w))
    v = int(round(point[1]*depth_h/color_h))
    return (
        max(0, min(depth_w-1, u)),
        max(0, min(depth_h-1, v)),
    )


def pixel_depth_to_base(
    u: int,
    v: int,
    depth_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    base_to_camera: np.ndarray,
) -> np.ndarray:
    camera_point = np.array([
        (u-cx)*depth_m/fx,
        (v-cy)*depth_m/fy,
        depth_m,
        1.0,
    ])
    return (base_to_camera @ camera_point)[:3]


def expanded_bbox(box_xyxy, image_shape) -> Tuple[int, int, int, int]:
    x1, y1, x2, y2 = map(float, box_xyxy)
    width = x2-x1
    height = y2-y1
    pad_x = width*ROI_PADDING_RATIO
    pad_y = height*ROI_PADDING_RATIO
    image_h, image_w = image_shape[:2]
    return (
        max(0, int(math.floor(x1-pad_x))),
        max(0, int(math.floor(y1-pad_y))),
        min(image_w-1, int(math.ceil(x2+pad_x))),
        min(image_h-1, int(math.ceil(y2+pad_y))),
    )


def contour_angle_candidate(
    contour,
    roi_shape,
) -> Optional[Tuple[float, np.ndarray, float, float]]:
    roi_h, roi_w = roi_shape[:2]
    roi_area = float(roi_h*roi_w)
    area = float(cv2.contourArea(contour))
    if roi_area <= 1.0:
        return None
    area_ratio = area/roi_area
    if not MIN_CONTOUR_AREA_RATIO <= area_ratio <= MAX_CONTOUR_AREA_RATIO:
        return None

    rect = cv2.minAreaRect(contour)
    (_, _), (rw, rh), _ = rect
    if min(rw, rh) < 3.0:
        return None
    aspect = max(rw, rh)/min(rw, rh)
    if aspect < MIN_RECT_ASPECT_RATIO:
        return None

    points = cv2.boxPoints(rect)
    center = np.mean(points, axis=0)
    roi_center = np.array([roi_w/2.0, roi_h/2.0])
    center_distance_ratio = float(
        np.linalg.norm(center-roi_center)/max(roi_w, roi_h)
    )
    if center_distance_ratio > MAX_CONTOUR_CENTER_DISTANCE_RATIO:
        return None

    longest = None
    for index in range(4):
        a = points[index]
        b = points[(index+1) % 4]
        length = float(np.linalg.norm(b-a))
        if longest is None or length > longest[0]:
            longest = (length, a, b)
    _, point_a, point_b = longest

    # 优先面积、长宽比和居中程度。
    score = area_ratio * min(aspect, 6.0) / (1.0+3.0*center_distance_ratio)
    return score, np.vstack([point_a, point_b]), aspect, area_ratio


def estimate_axis_in_roi(
    color: np.ndarray,
    bbox: Tuple[int, int, int, int],
) -> Tuple[Optional[dict], np.ndarray]:
    x1, y1, x2, y2 = bbox
    roi = color[y1:y2+1, x1:x2+1]
    debug_roi = roi.copy()
    if roi.size == 0 or roi.shape[0] < 12 or roi.shape[1] < 12:
        return None, debug_roi

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    masks = []
    edges = cv2.Canny(blur, 30, 110)
    edges = cv2.morphologyEx(
        edges,
        cv2.MORPH_CLOSE,
        np.ones((5, 5), np.uint8),
        iterations=2,
    )
    masks.append(("canny", edges))

    _, otsu = cv2.threshold(
        blur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU
    )
    masks.append(("otsu", otsu))
    masks.append(("otsu_inverse", cv2.bitwise_not(otsu)))

    candidates = []
    for method, mask in masks:
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            candidate = contour_angle_candidate(contour, roi.shape)
            if candidate is None:
                continue
            score, points, aspect, area_ratio = candidate
            candidates.append({
                "score": score,
                "points": points,
                "aspect_ratio": aspect,
                "area_ratio": area_ratio,
                "method": method,
            })

    if not candidates:
        return None, debug_roi
    best = max(candidates, key=lambda item: item["score"])
    points = best["points"].copy()
    points[:, 0] += x1
    points[:, 1] += y1
    best["points_color"] = points
    return best, debug_roi


def get_obb_axis(result) -> Optional[dict]:
    """若模型为Ultralytics OBB，优先直接读取旋转框。"""
    obb = getattr(result, "obb", None)
    if obb is None or len(obb) == 0:
        return None
    try:
        confidences = obb.conf.cpu().numpy()
        index = int(np.argmax(confidences))
        polygon = obb.xyxyxyxy[index].cpu().numpy().reshape(4, 2)
        longest = None
        for i in range(4):
            a = polygon[i]
            b = polygon[(i+1) % 4]
            length = float(np.linalg.norm(b-a))
            if longest is None or length > longest[0]:
                longest = (length, a, b)
        _, a, b = longest
        side_lengths = [
            float(np.linalg.norm(polygon[(i+1)%4]-polygon[i]))
            for i in range(4)
        ]
        aspect = max(side_lengths)/max(1e-6, min(side_lengths))
        return {
            "score": float(confidences[index]),
            "points_color": np.vstack([a, b]),
            "aspect_ratio": aspect,
            "area_ratio": None,
            "method": "yolo_obb",
        }
    except Exception:
        return None


def show_live_frame(frame: np.ndarray, status_lines: List[str]) -> bool:
    """显示并保存YOLO实时画面。按Q或ESC安全终止识别。"""
    canvas = frame.copy()
    y = 28
    for line in status_lines:
        cv2.putText(
            canvas, str(line), (12, y), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (0, 255, 255), 2, cv2.LINE_AA
        )
        y += 27
    if SAVE_LATEST_FRAME:
        cv2.imwrite(LIVE_LATEST_IMAGE, canvas)
    if not SHOW_YOLO_WINDOW:
        return True
    try:
        display = canvas
        if abs(DISPLAY_SCALE - 1.0) > 1e-6:
            display = cv2.resize(
                canvas, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE,
                interpolation=cv2.INTER_AREA
            )
        cv2.imshow(LIVE_WINDOW_NAME, display)
        key = cv2.waitKey(WINDOW_WAIT_KEY_MS) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            return False
    except cv2.error as exc:
        print(f"[YOLO画面] GUI显示失败，继续保存最新画面: {exc}")
        return True
    return True


def close_live_window() -> None:
    if SHOW_YOLO_WINDOW:
        try:
            cv2.destroyWindow(LIVE_WINDOW_NAME)
            cv2.waitKey(1)
        except cv2.error:
            pass


def detect_product_position_and_angle(model, camera, tf_api, report: dict) -> dict:
    if SHOW_YOLO_WINDOW:
        try:
            cv2.namedWindow(LIVE_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(LIVE_WINDOW_NAME, 960, 540)
            cv2.moveWindow(LIVE_WINDOW_NAME, 40, 40)
            cv2.waitKey(100)
            print(f"[YOLO画面] 已创建窗口: {LIVE_WINDOW_NAME}")
        except cv2.error as exc:
            print(f"[YOLO画面] 创建窗口失败，将继续保存画面文件: {exc}")
    intrinsic = camera.get_camera_intrinsic(agibot_gdk.CameraType.kHeadDepth)
    fx, fy, cx, cy = map(float, list(intrinsic.intrinsic)[:4])
    base_to_camera = (
        transform_matrix(tf_api.get_tf_from_base_link(HEAD_FRAME))
        @ transform_matrix(
            tf_api.get_tf_from_sensor(
                agibot_gdk.SensorExtrinsicType.kHeadRGBDToHeadLink3
            )
        )
    )

    position_samples = []
    angle_samples = []
    reject_counts = {
        "no_image": 0,
        "no_detection": 0,
        "no_center_depth": 0,
        "no_angle_contour": 0,
        "invalid_axis": 0,
    }
    last_debug = None
    deadline = time.time()+DETECTION_TIMEOUT_S

    while time.time() < deadline and len(position_samples) < DETECTION_SAMPLES:
        color_object = safe_get_image(camera, agibot_gdk.CameraType.kHeadColor)
        depth_object = safe_get_image(camera, agibot_gdk.CameraType.kHeadDepth)
        if color_object is None or depth_object is None:
            reject_counts["no_image"] += 1
            continue

        color = decode_color(color_object)
        depth = decode_depth(depth_object)
        if color is None or depth is None:
            reject_counts["no_image"] += 1
            continue

        result = model.predict(
            color,
            conf=MIN_CONFIDENCE,
            verbose=False,
        )[0]
        annotated = color.copy()
        if result.boxes is None or len(result.boxes) == 0:
            reject_counts["no_detection"] += 1
            if not show_live_frame(annotated, [
                "YOLO: NO DETECTION",
                f"conf threshold: {MIN_CONFIDENCE:.2f}",
                f"attempts without target: {reject_counts['no_detection']}",
                "Press Q or ESC to stop vision",
            ]):
                raise KeyboardInterrupt("用户在YOLO窗口中终止")
            time.sleep(NO_DETECTION_SLEEP_S)
            continue

        box = max(result.boxes, key=lambda item: float(item.conf[0]))
        confidence = float(box.conf[0])
        raw_bbox = box.xyxy[0].tolist()
        bbox = expanded_bbox(raw_bbox, color.shape)
        x1, y1, x2, y2 = bbox
        center_color = ((x1+x2)/2.0, (y1+y2)/2.0)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(
            annotated,
            (int(round(center_color[0])), int(round(center_color[1]))),
            5, (0, 255, 255), -1
        )
        cv2.putText(
            annotated, f"conf={confidence:.3f}",
            (x1, max(25, y1-8)), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (0, 255, 0), 2, cv2.LINE_AA
        )
        center_u, center_v = color_to_depth_pixel(
            center_color, color.shape, depth.shape
        )
        center_raw_depth = median_depth(depth, center_u, center_v)
        if center_raw_depth is None:
            reject_counts["no_center_depth"] += 1
            if not show_live_frame(annotated, [
                "YOLO: DETECTED",
                "Depth: INVALID AT CENTER",
                f"confidence: {confidence:.3f}",
            ]):
                raise KeyboardInterrupt("用户在YOLO窗口中终止")
            time.sleep(NO_DETECTION_SLEEP_S)
            continue
        center_depth_m = (
            center_raw_depth/1000.0
            if center_raw_depth > 20.0
            else center_raw_depth
        )
        center_base = pixel_depth_to_base(
            center_u,
            center_v,
            center_depth_m,
            fx,
            fy,
            cx,
            cy,
            base_to_camera,
        )
        position_samples.append({
            "confidence": confidence,
            "base_xyz": center_base.tolist(),
            "center_pixel": list(center_color),
            "depth_m": center_depth_m,
            "bbox": [x1, y1, x2, y2],
        })

        angle_candidate = get_obb_axis(result)
        if angle_candidate is None:
            angle_candidate, _ = estimate_axis_in_roi(color, bbox)
        if angle_candidate is None:
            reject_counts["no_angle_contour"] += 1
        else:
            points = angle_candidate["points_color"]
            # 两个长轴端点使用中心深度反投影，避免端点深度落在背景导致角度丢失。
            base_axis_points = []
            for point in points:
                u, v = color_to_depth_pixel(
                    (float(point[0]), float(point[1])),
                    color.shape,
                    depth.shape,
                )
                base_axis_points.append(
                    pixel_depth_to_base(
                        u, v, center_depth_m,
                        fx, fy, cx, cy,
                        base_to_camera,
                    )
                )
            axis = base_axis_points[1]-base_axis_points[0]
            if float(np.linalg.norm(axis[:2])) < 1e-5:
                reject_counts["invalid_axis"] += 1
            else:
                yaw = math.atan2(float(axis[1]), float(axis[0]))
                angle_samples.append({
                    "yaw_rad": yaw,
                    "yaw_deg": math.degrees(yaw),
                    "aspect_ratio": angle_candidate["aspect_ratio"],
                    "method": angle_candidate["method"],
                    "axis_pixels": points.tolist(),
                })

                debug = color.copy()
                cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 255), 2)
                point_a = tuple(np.round(points[0]).astype(int))
                point_b = tuple(np.round(points[1]).astype(int))
                cv2.line(debug, point_a, point_b, (0, 0, 255), 3)
                cv2.circle(
                    debug,
                    (int(round(center_color[0])), int(round(center_color[1]))),
                    5,
                    (0, 255, 0),
                    -1,
                )
                cv2.putText(
                    debug,
                    f"yaw={math.degrees(yaw):.1f} deg {angle_candidate['method']}",
                    (x1, max(25, y1-10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 0, 255),
                    2,
                )
                last_debug = debug
                annotated = debug

        live_lines = [
            "YOLO: DETECTED",
            f"confidence: {confidence:.3f}",
            f"position samples: {len(position_samples)}/{DETECTION_SAMPLES}",
            f"angle samples: {len(angle_samples)}",
            f"depth: {center_depth_m:.3f} m",
            "Press Q or ESC to stop vision",
        ]
        if not show_live_frame(annotated, live_lines):
            raise KeyboardInterrupt("用户在YOLO窗口中终止")
        time.sleep(0.10)

    if last_debug is not None:
        cv2.imwrite(ANGLE_DEBUG_IMAGE, last_debug)
        if KEEP_FINAL_YOLO_FRAME_OPEN:
            show_live_frame(last_debug, [
                "YOLO TARGET LOCKED",
                f"position samples: {len(position_samples)}",
                f"angle samples: {len(angle_samples)}",
                "Target is frozen for this pick cycle",
            ])

    report["vision_diagnostics"] = {
        "position_sample_count": len(position_samples),
        "angle_sample_count": len(angle_samples),
        "reject_counts": reject_counts,
        "angle_debug_image": ANGLE_DEBUG_IMAGE if last_debug is not None else None,
        "live_latest_image": LIVE_LATEST_IMAGE if Path(LIVE_LATEST_IMAGE).exists() else None,
        "show_yolo_window": SHOW_YOLO_WINDOW,
    }
    save_report(report)

    if len(position_samples) < MIN_VALID_POSITION_SAMPLES:
        raise RuntimeError(
            f"有效产品位置样本不足: {len(position_samples)}; "
            f"诊断={reject_counts}"
        )

    product_xyz = np.median(
        np.array([sample["base_xyz"] for sample in position_samples]),
        axis=0,
    ).tolist()
    result_data = {
        "confidence": float(np.median([
            sample["confidence"] for sample in position_samples
        ])),
        "base_xyz": product_xyz,
        "depth_m": float(np.median([
            sample["depth_m"] for sample in position_samples
        ])),
        "position_sample_count": len(position_samples),
        "angle_valid": False,
        "angle_sample_count": len(angle_samples),
        "reject_counts": reject_counts,
        "debug_image": ANGLE_DEBUG_IMAGE if last_debug is not None else None,
    }

    if len(angle_samples) >= MIN_VALID_ANGLE_SAMPLES:
        raw_angles = [sample["yaw_rad"] for sample in angle_samples]
        preliminary = circular_axis_average(raw_angles)
        filtered = [
            sample for sample in angle_samples
            if axis_angle_difference_deg(sample["yaw_rad"], preliminary)
            <= ANGLE_MAX_DEVIATION_DEG
        ]
        if len(filtered) >= MIN_VALID_ANGLE_SAMPLES:
            yaw = circular_axis_average([
                sample["yaw_rad"] for sample in filtered
            ])
            result_data.update({
                "angle_valid": True,
                "product_yaw_rad": yaw,
                "product_yaw_deg": math.degrees(yaw),
                "angle_sample_count": len(filtered),
                "aspect_ratio": float(np.median([
                    sample["aspect_ratio"] for sample in filtered
                ])),
                "angle_methods": sorted(set(
                    sample["method"] for sample in filtered
                )),
            })
    return result_data


def set_both_arm_pose(robot, left_pose: PoseData, right_pose: PoseData) -> None:
    request = agibot_gdk.EndEffectorPose()
    request.life_time = LIFE_TIME
    request.group = agibot_gdk.EndEffectorControlGroup.kBothArms

    lp = request.left_end_effector_pose.position
    lq = request.left_end_effector_pose.orientation
    rp = request.right_end_effector_pose.position
    rq = request.right_end_effector_pose.orientation

    lp.x, lp.y, lp.z = left_pose.position
    lq.x, lq.y, lq.z, lq.w = left_pose.orientation
    rp.x, rp.y, rp.z = right_pose.position
    rq.x, rq.y, rq.z, rq.w = right_pose.orientation

    result = robot.end_effector_pose_control(request)
    if result != 0:
        raise RuntimeError(f"end_effector_pose_control失败: {result}")


def end_from_tcp(tcp_xyz, orientation) -> List[float]:
    offset = q_to_rotation(orientation).dot(TCP_OFFSET_END_M)
    return (np.asarray(tcp_xyz, dtype=float)-offset).tolist()


def gripper_axis_yaw(orientation) -> float:
    rotation = q_to_rotation(orientation)
    axis_index = 0 if GRIPPER_LONG_AXIS_LOCAL.lower() == "x" else 1
    axis = rotation[:, axis_index]
    if float(np.linalg.norm(axis[:2])) < 1e-5:
        raise RuntimeError("夹爪长轴在Base XY投影过小，无法计算偏航角")
    return math.atan2(float(axis[1]), float(axis[0]))


def move_tcp_delta(
    robot,
    tf_api,
    left_hold: PoseData,
    delta_xyz: List[float],
    fixed_orientation: List[float],
    baseline: Dict[str, float],
    report: dict,
    label: str,
) -> None:
    start_end = wait_pose(tf_api, RIGHT_FRAME)
    start_tcp = wait_pose(tf_api, TCP_FRAME)
    target_tcp = [start_tcp.position[i]+delta_xyz[i] for i in range(3)]
    target_end = [start_end.position[i]+delta_xyz[i] for i in range(3)]
    path_length = math.sqrt(sum(value*value for value in delta_xyz))
    steps = max(2, int(math.ceil(path_length/CART_STEP_M)))

    if REQUIRE_DANGEROUS_STAGE_CONFIRMATION:
        command = input(
            f"准备执行{label}，TCP delta="
            f"{[round(v,5) for v in delta_xyz]}，输入 NEXT："
        ).strip()
        if command != "NEXT":
            raise RuntimeError(f"用户取消{label}")

    for index in range(1, steps+1):
        verify_torque(robot, baseline, report)
        alpha = index/steps
        predicted_tcp_z = start_tcp.position[2]+alpha*delta_xyz[2]
        if predicted_tcp_z < MIN_TCP_Z_M:
            raise RuntimeError(f"{label}预测TCP高度低于安全门限")
        command_position = lerp_xyz(start_end.position, target_end, alpha)
        set_both_arm_pose(
            robot,
            left_hold,
            PoseData(command_position, fixed_orientation.copy()),
        )
        if SHOW_YOLO_WINDOW:
            try:
                cv2.waitKey(1)
            except cv2.error:
                pass
        time.sleep(DT)

    time.sleep(SETTLE_SECONDS)
    actual_tcp = wait_pose(tf_api, TCP_FRAME)
    actual_end = wait_pose(tf_api, RIGHT_FRAME)
    tcp_error = distance(actual_tcp.position, target_tcp)
    orientation_error = quaternion_error_deg(
        actual_end.orientation,
        fixed_orientation,
    )
    report["cartesian_stages"].append({
        "stage": label,
        "delta_xyz": delta_xyz,
        "target_tcp": target_tcp,
        "actual_tcp": actual_tcp.__dict__,
        "tcp_error_m": tcp_error,
        "orientation_error_deg": orientation_error,
    })
    save_report(report)

    print(
        f"[{label}验证] TCP误差={tcp_error:.4f}m, "
        f"姿态误差={orientation_error:.2f}deg"
    )
    if tcp_error > TCP_POSITION_TOLERANCE_M:
        raise RuntimeError(f"{label} TCP未到位")
    if orientation_error > ORIENTATION_TOLERANCE_DEG:
        raise RuntimeError(f"{label}姿态未保持")


def rotate_tcp_yaw_staged(
    robot,
    tf_api,
    left_hold: PoseData,
    yaw_delta_rad: float,
    baseline: Dict[str, float],
    report: dict,
) -> Tuple[PoseData, PoseData]:
    total_deg = abs(math.degrees(yaw_delta_rad))
    stage_count = max(1, int(math.ceil(total_deg/MAX_YAW_PER_STAGE_DEG)))

    if REQUIRE_DANGEROUS_STAGE_CONFIRMATION:
        command = input(
            f"准备高位角度对齐，总旋转={math.degrees(yaw_delta_rad):.1f}deg，"
            f"拆为{stage_count}段，输入 NEXT："
        ).strip()
        if command != "NEXT":
            raise RuntimeError("用户取消角度对齐")

    initial_end = wait_pose(tf_api, RIGHT_FRAME)
    initial_tcp = wait_pose(tf_api, TCP_FRAME)
    initial_orientation = initial_end.orientation.copy()
    stages = []

    for stage_index in range(1, stage_count+1):
        verify_torque(robot, baseline, report)
        stage_fraction = stage_index/stage_count
        stage_yaw = yaw_delta_rad*stage_fraction
        target_orientation = q_multiply(
            q_from_base_yaw(stage_yaw),
            initial_orientation,
        )
        stage_start_end = wait_pose(tf_api, RIGHT_FRAME)
        stage_start_tcp = wait_pose(tf_api, TCP_FRAME)
        orientation_change = quaternion_error_deg(
            stage_start_end.orientation,
            target_orientation,
        )
        interpolation_count = max(
            2,
            int(math.ceil(
                orientation_change/ROTATION_INTERPOLATION_STEP_DEG
            )),
        )

        for interpolation_index in range(1, interpolation_count+1):
            verify_torque(robot, baseline, report)
            alpha = interpolation_index/interpolation_count
            command_orientation = q_slerp(
                stage_start_end.orientation,
                target_orientation,
                alpha,
            )
            # 每个插值点固定阶段开始时的真实TCP，反算End目标。
            command_end_position = end_from_tcp(
                stage_start_tcp.position,
                command_orientation,
            )
            set_both_arm_pose(
                robot,
                left_hold,
                PoseData(command_end_position, command_orientation),
            )
            if SHOW_YOLO_WINDOW:
                try:
                    cv2.waitKey(1)
                except cv2.error:
                    pass
            time.sleep(DT)

        # 在每段最终姿态持续发送一小段时间，让姿态误差收敛。
        hold_count = max(1, int(
            ROTATION_FINAL_HOLD_SECONDS * ROTATION_HOLD_RATE_HZ
        ))
        final_end_position = end_from_tcp(
            stage_start_tcp.position, target_orientation
        )
        for _ in range(hold_count):
            verify_torque(robot, baseline, report)
            set_both_arm_pose(
                robot,
                left_hold,
                PoseData(final_end_position, target_orientation),
            )
            if SHOW_YOLO_WINDOW:
                try:
                    cv2.waitKey(1)
                except cv2.error:
                    pass
            time.sleep(1.0/ROTATION_HOLD_RATE_HZ)

        time.sleep(ROTATION_STAGE_SETTLE_SECONDS)
        actual_tcp = wait_pose(tf_api, TCP_FRAME)
        actual_end = wait_pose(tf_api, RIGHT_FRAME)
        tcp_drift = distance(actual_tcp.position, stage_start_tcp.position)
        orientation_error = quaternion_error_deg(
            actual_end.orientation,
            target_orientation,
        )
        stage_record = {
            "stage_index": stage_index,
            "stage_count": stage_count,
            "target_yaw_from_start_deg": math.degrees(stage_yaw),
            "target_orientation": target_orientation,
            "actual_end": actual_end.__dict__,
            "actual_tcp": actual_tcp.__dict__,
            "tcp_drift_m": tcp_drift,
            "orientation_error_deg": orientation_error,
        }
        stages.append(stage_record)
        report["angle_alignment_stages"] = stages
        save_report(report)

        print(
            f"[角度对齐 {stage_index}/{stage_count}] "
            f"TCP漂移={tcp_drift:.4f}m, "
            f"姿态误差={orientation_error:.2f}deg"
        )
        if tcp_drift > MAX_TCP_DRIFT_PER_ROTATION_STAGE_M:
            raise RuntimeError(
                f"角度对齐第{stage_index}段TCP漂移过大"
            )
        if orientation_error > ORIENTATION_TOLERANCE_DEG:
            raise RuntimeError(
                f"角度对齐第{stage_index}段姿态误差过大"
            )

    final_tcp = wait_pose(tf_api, TCP_FRAME)
    final_end = wait_pose(tf_api, RIGHT_FRAME)
    total_tcp_drift = distance(final_tcp.position, initial_tcp.position)
    report["angle_alignment_summary"] = {
        "requested_yaw_delta_deg": math.degrees(yaw_delta_rad),
        "stage_count": stage_count,
        "initial_tcp": initial_tcp.__dict__,
        "final_tcp": final_tcp.__dict__,
        "total_tcp_drift_m": total_tcp_drift,
        "final_end": final_end.__dict__,
    }
    save_report(report)
    return final_tcp, final_end


def set_right_gripper(robot, position: float) -> None:
    request = agibot_gdk.JointStates()
    request.group = "right_tool"
    request.target_type = "omnipicker"
    joint = agibot_gdk.JointState()
    joint.position = position
    request.states = [joint]
    request.nums = 1
    result = robot.move_ee_pos(request)
    if result != 0:
        raise RuntimeError(f"右夹爪控制失败: {result}")
    time.sleep(0.7)


def main() -> None:
    report = {
        "program": "61_g2_yolo_live_angle_stable.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "enable_angle_alignment": ENABLE_ANGLE_ALIGNMENT,
        "enable_xy_and_pregrasp": ENABLE_XY_AND_PREGRASP,
        "enable_gripper_and_pick": ENABLE_GRIPPER_AND_PICK,
        "calibration_changes": {
            "gripper_long_axis_local": GRIPPER_LONG_AXIS_LOCAL,
            "grasp_above_product_m": GRASP_ABOVE_PRODUCT_M,
            "grasp_z_correction_from_previous_m": GRASP_Z_CORRECTION_M,
            "qt_font_dir": os.environ.get("QT_QPA_FONTDIR"),
            "qt_font_prepare_result": QT_FONT_PREPARE_RESULT,
            "max_yaw_per_stage_deg": MAX_YAW_PER_STAGE_DEG,
            "rotation_final_hold_seconds": ROTATION_FINAL_HOLD_SECONDS,
        },
        "joint_stages": [],
        "cartesian_stages": [],
        "status": "INITIALIZED",
    }
    initialized = False
    camera = None

    try:
        if not Path(MODEL_PATH).exists():
            raise RuntimeError(f"YOLO模型不存在: {MODEL_PATH}")
        if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("GDK初始化失败")
        initialized = True

        robot = agibot_gdk.Robot()
        tf_api = agibot_gdk.TF()
        camera = agibot_gdk.Camera()
        model = YOLO(MODEL_PATH)
        time.sleep(3.0)

        current_right = get_positions(robot, RIGHT_JOINTS)
        start_name, start_index, start_error = closest_start_pose(current_right)
        left_hold = wait_pose(tf_api, LEFT_FRAME)
        report.update({
            "recognized_start_pose": start_name,
            "start_pose_index": start_index,
            "start_pose_max_joint_error_rad": start_error,
            "initial_right_joints": current_right,
            "initial_tcp": wait_pose(tf_api, TCP_FRAME).__dict__,
        })
        save_report(report)

        print("="*78)
        print("61_g2_yolo_live_angle_stable.py")
        print(f"ENABLE_REAL_MOTION={ENABLE_REAL_MOTION}")
        print(f"ENABLE_ANGLE_ALIGNMENT={ENABLE_ANGLE_ALIGNMENT}")
        print(f"ENABLE_XY_AND_PREGRASP={ENABLE_XY_AND_PREGRASP}")
        print(f"ENABLE_GRIPPER_AND_PICK={ENABLE_GRIPPER_AND_PICK}")
        print(f"夹爪长轴局部轴={GRIPPER_LONG_AXIS_LOCAL.upper()}")
        print(f"抓取TCP相对产品表面Z={GRASP_ABOVE_PRODUCT_M:.3f}m（比59版下降30mm）")
        print(f"Qt字体目录={os.environ.get('QT_QPA_FONTDIR', '未设置')}")
        print(f"Qt字体准备={QT_FONT_PREPARE_RESULT}")
        print(f"角度每段最大变化={MAX_YAW_PER_STAGE_DEG:.1f}deg")
        print(f"角度末端保持={ROTATION_FINAL_HOLD_SECONDS:.2f}s")
        print(f"起始姿态={start_name}, 最大关节差={start_error:.4f}rad")
        print("固定路径：HOME -> WAYPOINT_1（一次到点）")
        print("          -> 头部idx13 +20度")
        print("          -> WAYPOINT_2（一次到点）")
        print("          -> YOLO位置+产品长轴")
        print("本程序为独立单文件；夹爪长轴使用局部Y轴；抓取点比59版下降30mm。")
        print("="*78)

        if not ENABLE_REAL_MOTION:
            report["status"] = "DRY_RUN_PASS"
            save_report(report)
            print("DRY RUN通过，未发送运动或夹爪命令。")
            return

        if input("确认急停可用、路径无障碍后输入 PICK：").strip() != "PICK":
            report["status"] = "CANCELLED_BY_USER"
            save_report(report)
            return

        baseline = establish_torque_baseline(robot)

        if start_index == 0:
            move_right_arm_once(
                robot, tf_api, "waypoint_1", WAYPOINT_1, baseline, report
            )
            move_head_after_waypoint1(robot, report)
            move_right_arm_once(
                robot, tf_api, "waypoint_2", WAYPOINT_2, baseline, report
            )
        elif start_index == 1:
            ensure_head_at_positive20(robot, report, "start_at_waypoint_1")
            move_right_arm_once(
                robot, tf_api, "waypoint_2", WAYPOINT_2, baseline, report
            )
        elif start_index == 2:
            ensure_head_at_positive20(robot, report, "start_at_waypoint_2")

        wp2_tcp = wait_pose(tf_api, TCP_FRAME)
        wp2_end = wait_pose(tf_api, RIGHT_FRAME)
        wp2_tcp_error = distance(wp2_tcp.position, RECORDED_TCP["waypoint_2"])
        if wp2_tcp_error > KEYPOINT_TCP_TOLERANCE_M:
            raise RuntimeError("WAYPOINT_2最终TCP验证失败")

        print("WAYPOINT_2和头部验证通过，开始YOLO位置与角度识别...")
        detection = detect_product_position_and_angle(
            model, camera, tf_api, report
        )
        product = detection["base_xyz"]
        pregrasp_tcp = [
            product[0],
            product[1],
            product[2]+PREGRASP_ABOVE_PRODUCT_M,
        ]
        grasp_tcp = [
            product[0],
            product[1],
            product[2]+GRASP_ABOVE_PRODUCT_M,
        ]
        lift_tcp = [
            product[0],
            product[1],
            product[2]+GRASP_ABOVE_PRODUCT_M+LIFT_AFTER_GRASP_M,
        ]
        dx = pregrasp_tcp[0]-wp2_tcp.position[0]
        dy = pregrasp_tcp[1]-wp2_tcp.position[1]
        dz = pregrasp_tcp[2]-wp2_tcp.position[2]
        xy_distance = math.hypot(dx, dy)
        descent = -dz

        report.update({
            "detection": detection,
            "product_base_xyz": product,
            "dynamic_targets": {
                "pregrasp_tcp": pregrasp_tcp,
                "grasp_tcp": grasp_tcp,
                "lift_tcp": lift_tcp,
            },
            "wp2_to_pregrasp_delta": [dx, dy, dz],
        })
        save_report(report)

        print(f"产品Base XYZ={np.round(product,5).tolist()}")
        print(f"动态PreGrasp={np.round(pregrasp_tcp,5).tolist()}")
        print(f"WP2->PreGrasp delta={[round(dx,5),round(dy,5),round(dz,5)]}")
        print(
            f"位置样本={detection['position_sample_count']}, "
            f"角度样本={detection['angle_sample_count']}"
        )

        if xy_distance > MAX_XY_FROM_WP2_M:
            raise RuntimeError(
                f"产品XY距离WAYPOINT_2过大: {xy_distance:.3f}m"
            )
        if descent < 0.0 or descent > MAX_PREGRASP_DESCENT_M:
            raise RuntimeError(
                f"动态PreGrasp下降量异常: {descent:.3f}m"
            )
        if grasp_tcp[2] < MIN_TCP_Z_M:
            raise RuntimeError("动态Grasp TCP低于绝对安全高度")

        # 角度计划：没有可靠角度时，中心定位仍保留，但禁止角度动作和抓取。
        if not detection.get("angle_valid", False):
            report["status"] = "POSITION_READY_ANGLE_UNAVAILABLE"
            save_report(report)
            diagnostics = detection.get("reject_counts", {})
            raise RuntimeError(
                "YOLO产品中心定位成功，但产品长轴角度无可靠结果；"
                f"诊断={diagnostics}；"
                f"请检查调试图像 {ANGLE_DEBUG_IMAGE}"
            )

        product_yaw = float(detection["product_yaw_rad"])
        current_gripper_yaw = gripper_axis_yaw(wp2_end.orientation)
        yaw_delta = wrap_axis_pi(
            product_yaw-current_gripper_yaw
            + math.radians(GRIPPER_YAW_OFFSET_DEG)
        )
        report.update({
            "product_yaw_deg": math.degrees(product_yaw),
            "gripper_axis_yaw_deg": math.degrees(current_gripper_yaw),
            "yaw_correction_deg": math.degrees(yaw_delta),
        })
        save_report(report)

        print(f"产品长轴={math.degrees(product_yaw):.1f}deg")
        print(f"夹爪长轴={math.degrees(current_gripper_yaw):.1f}deg")
        print(f"需要旋转夹爪={math.degrees(yaw_delta):.1f}deg")
        print(f"角度调试图像={ANGLE_DEBUG_IMAGE}")

        if abs(math.degrees(yaw_delta)) > MAX_YAW_CORRECTION_DEG:
            raise RuntimeError(
                f"需要的夹爪角度修正过大: "
                f"{math.degrees(yaw_delta):.1f}deg"
            )

        if not ENABLE_ANGLE_ALIGNMENT:
            report["status"] = "ANGLE_PLAN_READY_ALIGNMENT_DISABLED"
            save_report(report)
            print("位置和角度规划完成。角度动作未启用，安全停止。")
            return

        aligned_tcp, aligned_end = rotate_tcp_yaw_staged(
            robot,
            tf_api,
            left_hold,
            yaw_delta,
            baseline,
            report,
        )
        aligned_orientation = aligned_end.orientation.copy()

        if not ENABLE_XY_AND_PREGRASP:
            report["status"] = "ANGLE_ALIGNED_XY_DISABLED"
            save_report(report)
            print("夹爪角度已对齐。XY和下降未启用，安全停止。")
            return

        move_tcp_delta(
            robot,
            tf_api,
            left_hold,
            [
                pregrasp_tcp[0]-aligned_tcp.position[0],
                pregrasp_tcp[1]-aligned_tcp.position[1],
                0.0,
            ],
            aligned_orientation,
            baseline,
            report,
            "align_xy_at_wp2_height",
        )
        current_tcp = wait_pose(tf_api, TCP_FRAME)
        move_tcp_delta(
            robot,
            tf_api,
            left_hold,
            [0.0, 0.0, pregrasp_tcp[2]-current_tcp.position[2]],
            aligned_orientation,
            baseline,
            report,
            "descend_to_pregrasp",
        )

        if not ENABLE_GRIPPER_AND_PICK:
            report["status"] = "PREGRASP_REACHED_PICK_DISABLED"
            save_report(report)
            print("动态PreGrasp已到达。抓取未启用，安全停止。")
            return

        print("打开右夹爪...")
        set_right_gripper(robot, -0.785)
        report["gripper_opened"] = True
        save_report(report)

        current_tcp = wait_pose(tf_api, TCP_FRAME)
        move_tcp_delta(
            robot,
            tf_api,
            left_hold,
            [0.0, 0.0, grasp_tcp[2]-current_tcp.position[2]],
            aligned_orientation,
            baseline,
            report,
            "descend_to_grasp",
        )

        print("关闭右夹爪...")
        set_right_gripper(robot, 0.0)
        report["gripper_closed"] = True
        save_report(report)

        move_tcp_delta(
            robot,
            tf_api,
            left_hold,
            [0.0, 0.0, LIFT_AFTER_GRASP_M],
            aligned_orientation,
            baseline,
            report,
            "lift_10cm",
        )

        report["final_tcp"] = wait_pose(tf_api, TCP_FRAME).__dict__
        report["status"] = "PICK_AND_LIFT_COMPLETED"
        save_report(report)
        print("产品角度对齐抓取并抬升10cm完成。")

    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED"
        report["error"] = "KeyboardInterrupt"
        save_report(report)
        print("用户中断，停止后续动作。")
    except Exception as exc:
        if report.get("status") == "INITIALIZED":
            report["status"] = "ERROR"
        report["error"] = str(exc)
        save_report(report)
        raise
    finally:
        close_live_window()
        if camera is not None:
            try:
                camera.close_camera()
            except Exception:
                pass
        if initialized:
            agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
