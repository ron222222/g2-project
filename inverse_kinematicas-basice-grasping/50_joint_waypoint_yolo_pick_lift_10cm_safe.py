#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
50_joint_waypoint_yolo_pick_lift_10cm_safe.py

基于VR示教关节路径 + 实时YOLO局部定位的右臂抓取程序。

流程：
1. 检查当前右臂是否位于已知安全姿态（HOME/WP1/WP2/WP3/PREGRASP）。
2. 使用 move_arm_joint() 按示教关节路径到 PREGRASP。
3. 到达 PREGRASP 后，实时YOLO + 深度 + TF计算产品 Base XYZ。
4. 不使用固定产品坐标；以实时产品坐标生成 TCP PreGrasp。
5. 仅允许小范围笛卡尔修正，超出局部窗口立即停止。
6. 打开右夹爪，沿 base_link -Z 下探。
7. 关闭右夹爪，沿 base_link +Z 抬升10cm。

默认安全设置：
- ENABLE_REAL_MOTION = False
- ENABLE_GRIPPER_ACTION = False
- ENABLE_DESCEND_AND_LIFT = False
- REQUIRE_CONFIRM_BEFORE_EACH_JOINT_STAGE = True

建议验证顺序：
A. 只开 ENABLE_REAL_MOTION，验证示教路径与YOLO规划，不下探。
B. 再开 ENABLE_GRIPPER_ACTION，仅验证夹爪打开。
C. 最后开 ENABLE_DESCEND_AND_LIFT，执行下探、闭合和抬升。
"""

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np
import agibot_gdk
from ultralytics import YOLO

# ==================== 总开关 ====================
ENABLE_REAL_MOTION = True
ENABLE_GRIPPER_ACTION = False
ENABLE_DESCEND_AND_LIFT = False
REQUIRE_CONFIRM_BEFORE_EACH_JOINT_STAGE = True

MODEL_PATH = "runs/detect/runs/product_detector/weights/best.pt"
REPORT_FILE = "joint_waypoint_yolo_pick_lift_10cm_report.json"

# ==================== 坐标系 ====================
LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"
TCP_FRAME = "gripper_r_center_link"
HEAD_FRAME = "head_link3"

LEFT_JOINTS = [
    "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
    "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
]
RIGHT_JOINTS = [
    "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
    "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]

# ==================== VR示教关节路径 ====================
HOME = [-1.57079643, -1.57079608, 1.57079585, -1.57079645,
        0.00000024, 0.00000000, 0.00000036]
WAYPOINT_1 = [-1.91304851, 0.20393955, 1.83169374, -1.84651811,
              0.29203423, -0.28742294, 0.91643847]
WAYPOINT_2 = [-2.34999969, 0.99999998, 2.39123433, -1.79048179,
              -1.50846242, -0.95681261, -0.11304288]
WAYPOINT_3 = [-2.35000014, 0.99999983, 2.27358603, -1.75903363,
              0.97554659, 0.01765348, 0.59585902]
PREGRASP = [-0.26450533, -1.08659669, 0.16999971, -1.47974414,
            -0.31172467, 0.44504713, 1.37577128]

POSE_CHAIN = [
    ("home", HOME),
    ("waypoint_1", WAYPOINT_1),
    ("waypoint_2", WAYPOINT_2),
    ("waypoint_3", WAYPOINT_3),
    ("pregrasp", PREGRASP),
]

RECORDED_TCP = {
    "home": [0.73185, -0.24350, 0.81644],
    "waypoint_1": [0.58639, -0.34289, 0.93608],
    "waypoint_2": [0.65699, -0.16556, 0.98419],
    "waypoint_3": [0.63205, -0.07514, 0.97981],
    "pregrasp": [0.66409, -0.09406, 0.82812],
}
RECORDED_PREGRASP_Q = [0.492028, 0.861327, 0.092413, 0.086507]

# ==================== 关节控制安全参数 ====================
ARM_SPEED_RAD_S = 0.08
START_POSE_MATCH_TOLERANCE_RAD = 0.12
JOINT_TARGET_TOLERANCE_RAD = 0.06
TCP_WAYPOINT_TOLERANCE_M = 0.050
PREGRASP_Q_TOLERANCE_DEG = 8.0
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

# ==================== YOLO / 深度参数 ====================
MIN_CONFIDENCE = 0.60
DETECTION_SAMPLES = 8
DETECTION_TIMEOUT_S = 20.0
CAMERA_TIMEOUT_MS = 1000.0
DEPTH_RADIUS = 3
MIN_DEPTH_RAW = 50.0
MAX_DEPTH_RAW = 10000.0

# ==================== 动态抓取参数 ====================
# 这些高度偏移相对实时检测到的产品Base Z，不依赖固定产品XYZ。
PREGRASP_ABOVE_PRODUCT_M = 0.100
GRASP_ABOVE_PRODUCT_M = 0.015
LIFT_AFTER_GRASP_M = 0.100

# 示教PREGRASP只允许进行小范围视觉修正，防止离开已验证构型。
MAX_LOCAL_DX_M = 0.050
MAX_LOCAL_DY_M = 0.060
MAX_LOCAL_DZ_M = 0.050
MAX_LOCAL_TRANSLATION_M = 0.085

CART_STEP_M = 0.001
RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
LIFE_TIME = 0.02
CART_POSITION_TOLERANCE_M = 0.020
CART_ORIENTATION_TOLERANCE_DEG = 5.0
TARGET_TCP_TOLERANCE_M = 0.025
MIN_TCP_Z_M = 0.700

# ==================== 力矩安全参数 ====================
BASELINE_SAMPLES = 30
TORQUE_DELTA_LIMIT_NM = 30.0


@dataclass
class PoseData:
    position: List[float]
    orientation: List[float]


def save_report(report):
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def normalize_q(q):
    norm = math.sqrt(sum(float(value) ** 2 for value in q))
    if norm < 1e-12:
        raise RuntimeError("四元数模长接近0")
    return [float(value) / norm for value in q]


def quaternion_error_deg(q1, q2):
    q1 = normalize_q(q1)
    q2 = normalize_q(q2)
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def distance(a, b):
    return math.sqrt(sum((float(b[index]) - float(a[index])) ** 2 for index in range(3)))


def lerp(a, b, alpha):
    return [(1.0-alpha)*a[index] + alpha*b[index] for index in range(3)]


def quaternion_to_rotation_matrix(q):
    x, y, z, w = normalize_q(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=float)


def transform_matrix(transform):
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_to_rotation_matrix([
        transform.rotation.x, transform.rotation.y,
        transform.rotation.z, transform.rotation.w,
    ])
    matrix[:3, 3] = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    return matrix


def read_pose(tf_api, frame):
    transform = tf_api.get_tf_from_base_link(frame)
    return PoseData(
        [float(transform.translation.x), float(transform.translation.y),
         float(transform.translation.z)],
        normalize_q([transform.rotation.x, transform.rotation.y,
                     transform.rotation.z, transform.rotation.w]),
    )


def wait_pose(tf_api, frame, timeout_s=10.0):
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


def joint_state_map(robot):
    return {state["name"]: state for state in robot.get_joint_states()["states"]}


def joint_positions(robot, names):
    states = joint_state_map(robot)
    missing = [name for name in names if name not in states]
    if missing:
        raise RuntimeError(f"缺少关节状态: {missing}")
    return [float(states[name]["motor_position"]) for name in names]


def max_joint_distance(a, b):
    return max(abs(float(x)-float(y)) for x, y in zip(a, b))


def identify_start_pose(current_right):
    candidates = []
    for index, (name, target) in enumerate(POSE_CHAIN):
        candidates.append((max_joint_distance(current_right, target), index, name))
    candidates.sort(key=lambda item: item[0])
    error, index, name = candidates[0]
    if error > START_POSE_MATCH_TOLERANCE_RAD:
        raise RuntimeError(
            f"当前右臂不在已知安全姿态附近；最近{name}，最大关节差={error:.3f}rad"
        )
    return index, name, error


def get_torques(robot) -> Dict[str, float]:
    return {
        state["name"]: float(state["effort"])
        for state in robot.get_joint_states()["states"]
    }


def establish_torque_baseline(robot):
    samples = []
    print("建立力矩基线，请保持机器人静止...")
    for _ in range(BASELINE_SAMPLES):
        samples.append(get_torques(robot))
        time.sleep(0.05)
    names = set().union(*(sample.keys() for sample in samples))
    return {
        name: sum(sample.get(name, 0.0) for sample in samples) / len(samples)
        for name in names
    }


def verify_torque(robot, baseline, report):
    current = get_torques(robot)
    abnormal = []
    for name, value in current.items():
        if name in baseline:
            delta = abs(value - baseline[name])
            if delta > TORQUE_DELTA_LIMIT_NM:
                abnormal.append({"joint": name, "delta": delta})
    if abnormal:
        report["status"] = "STOPPED_TORQUE"
        report["torque_abnormal"] = abnormal[:10]
        save_report(report)
        raise RuntimeError("力矩安全检查触发")


def validate_joint_target(label, target):
    if len(target) != 7:
        raise RuntimeError(f"{label}右臂目标长度不是7")
    for index, (value, limits) in enumerate(zip(target, JOINT_LIMITS)):
        if not limits[0] <= value <= limits[1]:
            raise RuntimeError(
                f"{label} {RIGHT_JOINTS[index]}={value:.6f}超出限位{limits}"
            )


def move_right_joint_stage(robot, tf_api, label, target, baseline, report):
    validate_joint_target(label, target)
    verify_torque(robot, baseline, report)

    if REQUIRE_CONFIRM_BEFORE_EACH_JOINT_STAGE:
        if input(f"准备执行 {label}，确认路径无障碍后输入 NEXT：").strip() != "NEXT":
            raise RuntimeError(f"用户取消{label}")

    # control_group=1时只发送右臂7个位置和7个速度。
    positions = list(target)
    velocities = [ARM_SPEED_RAD_S] * 7
    result = robot.move_arm_joint(positions, velocities, 1)
    if result != 0:
        raise RuntimeError(f"{label} move_arm_joint失败: {result}")

    time.sleep(SETTLE_SECONDS)
    actual = joint_positions(robot, RIGHT_JOINTS)
    errors = [abs(a-b) for a, b in zip(actual, target)]
    max_error = max(errors)
    tcp = wait_pose(tf_api, TCP_FRAME)
    tcp_error = distance(tcp.position, RECORDED_TCP[label])

    report["joint_stages"].append({
        "stage": label,
        "target": target,
        "actual": actual,
        "max_joint_error_rad": max_error,
        "actual_tcp": tcp.__dict__,
        "recorded_tcp": RECORDED_TCP[label],
        "tcp_error_m": tcp_error,
    })
    save_report(report)

    print(f"[{label}验证] 最大关节误差={max_error:.5f}rad, TCP误差={tcp_error:.5f}m")
    if max_error > JOINT_TARGET_TOLERANCE_RAD:
        raise RuntimeError(f"{label}关节未到位")
    if tcp_error > TCP_WAYPOINT_TOLERANCE_M:
        raise RuntimeError(f"{label} TCP未复现")


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
        return np.frombuffer(image.data, dtype=dtype).reshape((image.height, image.width))
    except Exception:
        return None


def median_depth(depth, u, v):
    height, width = depth.shape[:2]
    patch = depth[
        max(0, v-DEPTH_RADIUS):min(height, v+DEPTH_RADIUS+1),
        max(0, u-DEPTH_RADIUS):min(width, u+DEPTH_RADIUS+1),
    ].astype(float)
    valid = patch[np.isfinite(patch)]
    valid = valid[(valid > MIN_DEPTH_RAW) & (valid < MAX_DEPTH_RAW)]
    return None if valid.size == 0 else float(np.median(valid))


def detect_product(model, camera, tf_api):
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

    samples = []
    deadline = time.time() + DETECTION_TIMEOUT_S
    while time.time() < deadline and len(samples) < DETECTION_SAMPLES:
        color_object = safe_get_image(camera, agibot_gdk.CameraType.kHeadColor)
        depth_object = safe_get_image(camera, agibot_gdk.CameraType.kHeadDepth)
        if color_object is None or depth_object is None:
            continue
        color = decode_color(color_object)
        depth = decode_depth(depth_object)
        if color is None or depth is None:
            continue

        results = model.predict(color, conf=MIN_CONFIDENCE, verbose=False)
        if not len(results) or results[0].boxes is None or not len(results[0].boxes):
            continue
        box = max(results[0].boxes, key=lambda item: float(item.conf[0]))
        confidence = float(box.conf[0])
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        pixel_x, pixel_y = int((x1+x2)/2), int((y1+y2)/2)

        depth_h, depth_w = depth.shape[:2]
        depth_u = max(0, min(depth_w-1, int(round(pixel_x*depth_w/color.shape[1]))))
        depth_v = max(0, min(depth_h-1, int(round(pixel_y*depth_h/color.shape[0]))))
        raw_depth = median_depth(depth, depth_u, depth_v)
        if raw_depth is None:
            continue

        depth_m = raw_depth/1000.0 if raw_depth > 20.0 else raw_depth
        camera_point = np.array([
            (depth_u-cx)*depth_m/fx,
            (depth_v-cy)*depth_m/fy,
            depth_m,
            1.0,
        ])
        base_point = (base_to_camera @ camera_point)[:3]
        samples.append((confidence, base_point.tolist(), [pixel_x, pixel_y], depth_m))
        time.sleep(0.10)

    if len(samples) < max(3, DETECTION_SAMPLES//2):
        raise RuntimeError(f"有效YOLO三维样本不足: {len(samples)}")

    return {
        "confidence": float(np.median([sample[0] for sample in samples])),
        "base_xyz": np.median(
            np.array([sample[1] for sample in samples]), axis=0
        ).tolist(),
        "center_pixel": samples[-1][2],
        "depth_m": float(np.median([sample[3] for sample in samples])),
        "sample_count": len(samples),
    }


def set_both_arm_pose(robot, left_pose, right_pose):
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


def move_tcp_to_target(robot, tf_api, left_hold, target_tcp, fixed_q,
                       baseline, report, label):
    start_end = wait_pose(tf_api, RIGHT_FRAME)
    start_tcp = wait_pose(tf_api, TCP_FRAME)
    delta = [target_tcp[index]-start_tcp.position[index] for index in range(3)]
    target_end = [start_end.position[index]+delta[index] for index in range(3)]
    move_distance = math.sqrt(sum(value*value for value in delta))
    steps = max(2, int(math.ceil(move_distance/CART_STEP_M)))

    print(f"[{label}] TCP delta={[round(v,5) for v in delta]}, steps={steps}")
    for step in range(1, steps+1):
        verify_torque(robot, baseline, report)
        alpha = step/steps
        command_position = lerp(start_end.position, target_end, alpha)
        predicted_tcp_z = start_tcp.position[2] + alpha*delta[2]
        if predicted_tcp_z < MIN_TCP_Z_M:
            raise RuntimeError(f"{label}预测TCP高度低于安全门限")
        set_both_arm_pose(
            robot, left_hold, PoseData(command_position, fixed_q.copy())
        )
        time.sleep(DT)

    time.sleep(SETTLE_SECONDS)
    actual_end = wait_pose(tf_api, RIGHT_FRAME)
    actual_tcp = wait_pose(tf_api, TCP_FRAME)
    tcp_error = distance(actual_tcp.position, target_tcp)
    orientation_error = quaternion_error_deg(actual_end.orientation, fixed_q)
    report["cartesian_stages"].append({
        "stage": label,
        "target_tcp": target_tcp,
        "actual_tcp": actual_tcp.__dict__,
        "tcp_error_m": tcp_error,
        "orientation_error_deg": orientation_error,
    })
    save_report(report)
    print(f"[{label}验证] TCP误差={tcp_error:.4f}m, 姿态误差={orientation_error:.2f}deg")
    if tcp_error > TARGET_TCP_TOLERANCE_M:
        raise RuntimeError(f"{label} TCP未到位")
    if orientation_error > CART_ORIENTATION_TOLERANCE_DEG:
        raise RuntimeError(f"{label} 姿态未保持")


def set_right_gripper(robot, position):
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


def main():
    report = {
        "program": "50_joint_waypoint_yolo_pick_lift_10cm_safe.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "enable_gripper_action": ENABLE_GRIPPER_ACTION,
        "enable_descend_and_lift": ENABLE_DESCEND_AND_LIFT,
        "joint_stages": [],
        "cartesian_stages": [],
        "status": "INITIALIZED",
    }
    initialized = False
    camera = None

    try:
        if not Path(MODEL_PATH).exists():
            raise RuntimeError(f"YOLO模型不存在: {MODEL_PATH}")
        result = agibot_gdk.gdk_init()
        if result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError(f"GDK初始化失败: {result}")
        initialized = True

        robot = agibot_gdk.Robot()
        tf_api = agibot_gdk.TF()
        camera = agibot_gdk.Camera()
        model = YOLO(MODEL_PATH)
        time.sleep(3.0)

        current_right = joint_positions(robot, RIGHT_JOINTS)
        start_index, start_name, start_error = identify_start_pose(current_right)
        current_tcp = wait_pose(tf_api, TCP_FRAME)
        left_hold = wait_pose(tf_api, LEFT_FRAME)

        # 从当前已知姿态之后继续；从HOME开始时依次走WP1/WP2/WP3/PREGRASP。
        remaining_chain = POSE_CHAIN[start_index+1:]
        report.update({
            "recognized_start_pose": start_name,
            "start_pose_max_joint_error_rad": start_error,
            "initial_right_joints": current_right,
            "initial_tcp": current_tcp.__dict__,
            "remaining_joint_chain": [name for name, _ in remaining_chain],
        })
        save_report(report)

        print("=" * 78)
        print("50_joint_waypoint_yolo_pick_lift_10cm_safe.py")
        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print(f"ENABLE_GRIPPER_ACTION = {ENABLE_GRIPPER_ACTION}")
        print(f"ENABLE_DESCEND_AND_LIFT = {ENABLE_DESCEND_AND_LIFT}")
        print(f"识别起始姿态 = {start_name}, 最大关节差={start_error:.4f}rad")
        print(f"剩余关节路径 = {[name for name, _ in remaining_chain]}")
        print("产品坐标将在到达PREGRASP后由实时YOLO生成，不使用固定产品XYZ。")
        print("=" * 78)

        if not ENABLE_REAL_MOTION:
            report["status"] = "DRY_RUN_PASS"
            save_report(report)
            print("DRY RUN通过，未发送运动或夹爪命令。")
            return

        if input("确认急停可用、示教路径无障碍后输入 PICK：").strip() != "PICK":
            report["status"] = "CANCELLED_BY_USER"
            save_report(report)
            return

        baseline = establish_torque_baseline(robot)

        for label, target in remaining_chain:
            move_right_joint_stage(robot, tf_api, label, target, baseline, report)

        # 如果起始已经是PREGRASP，仍进行TF验证。
        pregrasp_tcp = wait_pose(tf_api, TCP_FRAME)
        pregrasp_end = wait_pose(tf_api, RIGHT_FRAME)
        pregrasp_tcp_error = distance(pregrasp_tcp.position, RECORDED_TCP["pregrasp"])
        pregrasp_q_error = quaternion_error_deg(
            pregrasp_end.orientation, RECORDED_PREGRASP_Q
        )
        report["pregrasp_validation"] = {
            "actual_tcp": pregrasp_tcp.__dict__,
            "actual_end": pregrasp_end.__dict__,
            "tcp_error_m": pregrasp_tcp_error,
            "orientation_error_deg": pregrasp_q_error,
        }
        save_report(report)
        if pregrasp_tcp_error > TCP_WAYPOINT_TOLERANCE_M:
            raise RuntimeError("PREGRASP TCP未复现")
        if pregrasp_q_error > PREGRASP_Q_TOLERANCE_DEG:
            raise RuntimeError("PREGRASP姿态未复现")

        print("PREGRASP关节路径验证通过，开始实时YOLO定位...")
        detection = detect_product(model, camera, tf_api)
        product = detection["base_xyz"]
        target_pregrasp_tcp = [
            product[0], product[1], product[2] + PREGRASP_ABOVE_PRODUCT_M
        ]
        correction = [
            target_pregrasp_tcp[index] - pregrasp_tcp.position[index]
            for index in range(3)
        ]
        correction_norm = math.sqrt(sum(value*value for value in correction))

        report.update({
            "detection": detection,
            "dynamic_product_base_xyz": product,
            "dynamic_pregrasp_tcp": target_pregrasp_tcp,
            "local_correction_xyz": correction,
            "local_correction_norm_m": correction_norm,
        })
        save_report(report)

        print(f"YOLO产品Base XYZ = {[round(v,5) for v in product]}")
        print(f"动态TCP PreGrasp = {[round(v,5) for v in target_pregrasp_tcp]}")
        print(f"局部修正 = {[round(v,5) for v in correction]}, norm={correction_norm:.4f}m")

        if (
            abs(correction[0]) > MAX_LOCAL_DX_M
            or abs(correction[1]) > MAX_LOCAL_DY_M
            or abs(correction[2]) > MAX_LOCAL_DZ_M
            or correction_norm > MAX_LOCAL_TRANSLATION_M
        ):
            report["status"] = "STOPPED_PRODUCT_OUTSIDE_LOCAL_WINDOW"
            save_report(report)
            raise RuntimeError("产品超出示教PREGRASP的局部修正窗口，请调整产品或机器人位置")

        # 实时产品位置已经获得；若抓取动作未启用，在这里安全停止。
        if not ENABLE_GRIPPER_ACTION:
            report["status"] = "YOLO_PLAN_READY_GRIPPER_DISABLED"
            save_report(report)
            print("YOLO动态抓取目标已生成。夹爪动作未启用，安全停止。")
            return

        # 保持示教PREGRASP姿态，局部修正TCP。
        move_tcp_to_target(
            robot, tf_api, left_hold, target_pregrasp_tcp,
            pregrasp_end.orientation, baseline, report,
            "vision_correct_pregrasp",
        )

        print("打开右夹爪...")
        set_right_gripper(robot, -0.785)
        report["gripper_opened"] = True
        save_report(report)

        if not ENABLE_DESCEND_AND_LIFT:
            report["status"] = "VISION_PREGRASP_REACHED_DESCEND_DISABLED"
            save_report(report)
            print("动态PREGRASP已到达，右夹爪已打开。下探功能未启用，安全停止。")
            return

        # 使用实时产品坐标，沿base_link Z方向下探。
        target_grasp_tcp = [
            product[0], product[1], product[2] + GRASP_ABOVE_PRODUCT_M
        ]
        report["dynamic_grasp_tcp"] = target_grasp_tcp
        save_report(report)
        move_tcp_to_target(
            robot, tf_api, left_hold, target_grasp_tcp,
            pregrasp_end.orientation, baseline, report,
            "descend_to_grasp",
        )

        print("关闭右夹爪...")
        close_torque_before = get_torques(robot)
        set_right_gripper(robot, 0.0)
        close_torque_after = get_torques(robot)
        report["gripper_closed"] = True
        report["right_gripper_close_torque_snapshot"] = {
            "before": close_torque_before,
            "after": close_torque_after,
        }
        save_report(report)

        target_lift_tcp = [
            target_grasp_tcp[0],
            target_grasp_tcp[1],
            target_grasp_tcp[2] + LIFT_AFTER_GRASP_M,
        ]
        report["dynamic_lift_tcp"] = target_lift_tcp
        save_report(report)
        move_tcp_to_target(
            robot, tf_api, left_hold, target_lift_tcp,
            pregrasp_end.orientation, baseline, report,
            "lift_10cm",
        )

        report["final_tcp"] = wait_pose(tf_api, TCP_FRAME).__dict__
        report["status"] = "PICK_AND_LIFT_COMPLETED"
        save_report(report)
        print("抓取并抬升10cm流程完成。")

    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED"
        report["error"] = "KeyboardInterrupt"
        save_report(report)
        print("用户中断，已停止后续动作。")
    except Exception as exc:
        if report.get("status") == "INITIALIZED":
            report["status"] = "ERROR"
        report["error"] = str(exc)
        save_report(report)
        raise
    finally:
        if camera is not None:
            try:
                camera.close_camera()
            except Exception:
                pass
        if initialized:
            agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
