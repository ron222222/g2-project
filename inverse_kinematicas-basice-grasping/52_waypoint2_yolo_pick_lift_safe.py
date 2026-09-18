#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
52_waypoint2_yolo_pick_lift_safe.py

右臂抓取流程：
HOME/已知姿态 -> 分段关节运动到 WAYPOINT_1 -> WAYPOINT_2
-> 在 WAYPOINT_2 运行 YOLO + 深度 + TF
-> 保持 WAYPOINT_2 实测姿态，先高位对准产品XY
-> 垂直下降到动态 PREGRASP
-> 打开右夹爪
-> 垂直下降到动态 GRASP
-> 关闭右夹爪
-> 垂直抬升10cm

不使用 WAYPOINT_3，不使用旧固定 PREGRASP。
默认只做 Dry Run。请按阶段逐步开放功能。
"""

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import cv2
import numpy as np
import agibot_gdk
from ultralytics import YOLO

# ==================== 分阶段安全开关 ====================
ENABLE_REAL_MOTION = True
ENABLE_VISION_ALIGNMENT = True
ENABLE_GRIPPER_OPEN = True
ENABLE_DESCEND_GRIP_LIFT = True
REQUIRE_STAGE_CONFIRMATION = True

MODEL_PATH = "runs/detect/runs/product_detector/weights/best.pt"
REPORT_FILE = "waypoint2_yolo_pick_lift_safe_report.json"

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

HOME = [-1.57079643, -1.57079608, 1.57079585, -1.57079645,
        0.00000024, 0.00000000, 0.00000036]
WAYPOINT_1 = [-1.91304851, 0.20393955, 1.83169374, -1.84651811,
              0.29203423, -0.28742294, 0.91643847]
WAYPOINT_2 = [-2.34999915, 1.00000018, 2.39123385, -1.79048167,
              -1.50846278, -0.95681237, -0.11304288]

KEYPOINTS = [("waypoint_1", WAYPOINT_1), ("waypoint_2", WAYPOINT_2)]
RECORDED_TCP = {
    "home": [0.73185, -0.24350, 0.81644],
    "waypoint_1": [0.58639, -0.34289, 0.93608],
    "waypoint_2": [0.65699, -0.16556, 0.98419],
}
RECORDED_WP2_Q = [-0.521650, 0.822528, -0.115051, 0.195174]

# 关节路径参数
ARM_SPEED_RAD_S = 0.12
MAX_JOINT_DELTA_PER_SUBTARGET_RAD = 0.30
JOINT_TOLERANCE_RAD = 0.07
KEYPOINT_TCP_TOLERANCE_M = 0.050
START_POSE_TOLERANCE_RAD = 0.15

# YOLO / 深度参数
MIN_CONFIDENCE = 0.60
DETECTION_SAMPLES = 8
DETECTION_TIMEOUT_S = 20.0
CAMERA_TIMEOUT_MS = 1000.0
DEPTH_RADIUS = 4
MIN_DEPTH_RAW = 50.0
MAX_DEPTH_RAW = 10000.0

# 动态抓取高度，均相对实时产品表面点
PREGRASP_ABOVE_PRODUCT_M = 0.120
GRASP_ABOVE_PRODUCT_M = 0.025
LIFT_AFTER_GRASP_M = 0.100

# 从WP2开始只允许有限局部移动
MAX_XY_FROM_WP2_M = 0.25
MAX_PREGRASP_DESCENT_FROM_WP2_M = 0.220
MIN_TCP_Z_M = 0.700

# 笛卡尔运动参数
CART_STEP_M = 0.001
RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
LIFE_TIME = 0.02
SETTLE_S = 0.8
TCP_TOLERANCE_M = 0.025
ORIENTATION_TOLERANCE_DEG = 6.0

# 力矩安全参数
BASELINE_SAMPLES = 30
TORQUE_DELTA_LIMIT_NM = 30.0

JOINT_LIMITS = [
    (-3.071796, 3.071796), (-2.059505, 2.059505),
    (-3.071796, 3.071796), (-2.495838, 1.012308),
    (-3.071796, 3.071796), (-1.012308, 1.012308),
    (-1.535907, 1.535907),
]


@dataclass
class PoseData:
    position: List[float]
    orientation: List[float]


def save(report):
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def normalize_q(q):
    n = math.sqrt(sum(float(v) ** 2 for v in q))
    if n < 1e-12:
        raise RuntimeError("四元数模长接近0")
    return [float(v) / n for v in q]


def quaternion_error_deg(q1, q2):
    q1, q2 = normalize_q(q1), normalize_q(q2)
    dot = abs(sum(a*b for a, b in zip(q1, q2)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def distance(a, b):
    return math.sqrt(sum((float(b[i])-float(a[i]))**2 for i in range(3)))


def lerp(a, b, alpha):
    return [(1-alpha)*a[i] + alpha*b[i] for i in range(3)]


def q_to_r(q):
    x, y, z, w = normalize_q(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=float)


def transform_matrix(t):
    m = np.eye(4)
    m[:3, :3] = q_to_r([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
    m[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return m


def read_pose(tf_api, frame):
    t = tf_api.get_tf_from_base_link(frame)
    return PoseData(
        [float(t.translation.x), float(t.translation.y), float(t.translation.z)],
        normalize_q([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]),
    )


def wait_pose(tf_api, frame, timeout=10.0):
    end = time.time() + timeout
    last_error = None
    while time.time() < end:
        try:
            if tf_api.can_transform("base_link", frame):
                return read_pose(tf_api, frame)
        except Exception as exc:
            last_error = exc
        time.sleep(0.2)
    raise RuntimeError(f"等待TF超时: {frame}; {last_error}")


def state_map(robot):
    return {s["name"]: s for s in robot.get_joint_states()["states"]}


def positions(robot, names):
    states = state_map(robot)
    return [float(states[name]["motor_position"]) for name in names]


def max_joint_error(a, b):
    return max(abs(x-y) for x, y in zip(a, b))


def closest_known_pose(current):
    candidates = [
        (max_joint_error(current, HOME), "home", -1),
        (max_joint_error(current, WAYPOINT_1), "waypoint_1", 0),
        (max_joint_error(current, WAYPOINT_2), "waypoint_2", 1),
    ]
    candidates.sort(key=lambda item: item[0])
    error, name, index = candidates[0]
    if error > START_POSE_TOLERANCE_RAD:
        raise RuntimeError(
            f"当前右臂不在HOME/WP1/WP2附近；最近{name}，最大关节差={error:.3f}rad"
        )
    return name, index, error


def get_torques(robot) -> Dict[str, float]:
    return {s["name"]: float(s["effort"])
            for s in robot.get_joint_states()["states"]}


def make_torque_baseline(robot):
    samples = []
    print("建立力矩基线，请保持机器人静止...")
    for _ in range(BASELINE_SAMPLES):
        samples.append(get_torques(robot))
        time.sleep(0.05)
    names = set().union(*(s.keys() for s in samples))
    return {name: sum(s.get(name, 0.0) for s in samples)/len(samples)
            for name in names}


def verify_torque(robot, baseline, report):
    current = get_torques(robot)
    bad = []
    for name, value in current.items():
        if name in baseline:
            delta = abs(value-baseline[name])
            if delta > TORQUE_DELTA_LIMIT_NM:
                bad.append({"joint": name, "delta": delta})
    if bad:
        report["status"] = "STOPPED_TORQUE"
        report["torque_abnormal"] = bad[:10]
        save(report)
        raise RuntimeError("力矩安全检查触发")


def validate_joint_target(label, target):
    if len(target) != 7:
        raise RuntimeError(f"{label}关节数不是7")
    for i, (value, limit) in enumerate(zip(target, JOINT_LIMITS)):
        if not limit[0] <= value <= limit[1]:
            raise RuntimeError(f"{label} {RIGHT_JOINTS[i]}超出限位")


def segmented_joint_move(robot, tf_api, label, target, baseline, report):
    validate_joint_target(label, target)
    start = positions(robot, RIGHT_JOINTS)
    max_delta = max_joint_error(start, target)
    count = max(1, int(math.ceil(max_delta/MAX_JOINT_DELTA_PER_SUBTARGET_RAD)))

    if REQUIRE_STAGE_CONFIRMATION:
        if input(f"准备执行{label}，共{count}个短目标，输入 NEXT：").strip() != "NEXT":
            raise RuntimeError(f"用户取消{label}")

    stage = {"stage": label, "segment_count": count, "subtargets": []}
    for i in range(1, count+1):
        verify_torque(robot, baseline, report)
        alpha = i/count
        target_i = [(1-alpha)*a + alpha*b for a, b in zip(start, target)]
        try:
            result = robot.move_arm_joint(target_i, [ARM_SPEED_RAD_S]*7, 1)
        except Exception as exc:
            stage["subtargets"].append({
                "index": i, "target": target_i,
                "actual_after_error": positions(robot, RIGHT_JOINTS),
                "error": str(exc),
            })
            report["joint_stages"].append(stage)
            report["status"] = f"STOPPED_{label.upper()}_SEGMENT_{i}"
            save(report)
            raise
        if result != 0:
            raise RuntimeError(f"{label}短目标{i}返回失败")
        actual = positions(robot, RIGHT_JOINTS)
        error = max_joint_error(actual, target_i)
        stage["subtargets"].append({
            "index": i, "target": target_i,
            "actual": actual, "max_joint_error_rad": error,
        })
        save(report)
        print(f"[{label}] {i}/{count}, 最大关节误差={error:.5f}rad")
        if error > JOINT_TOLERANCE_RAD:
            raise RuntimeError(f"{label}短目标{i}未到位")

    time.sleep(SETTLE_S)
    tcp = wait_pose(tf_api, TCP_FRAME)
    tcp_error = distance(tcp.position, RECORDED_TCP[label])
    stage["actual_tcp"] = tcp.__dict__
    stage["tcp_error_m"] = tcp_error
    report["joint_stages"].append(stage)
    save(report)
    print(f"[{label}验证] TCP误差={tcp_error:.5f}m")
    if tcp_error > KEYPOINT_TCP_TOLERANCE_M:
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
    h, w = depth.shape[:2]
    patch = depth[max(0, v-DEPTH_RADIUS):min(h, v+DEPTH_RADIUS+1),
                  max(0, u-DEPTH_RADIUS):min(w, u+DEPTH_RADIUS+1)].astype(float)
    valid = patch[np.isfinite(patch)]
    valid = valid[(valid > MIN_DEPTH_RAW) & (valid < MAX_DEPTH_RAW)]
    return None if valid.size == 0 else float(np.median(valid))


def detect_product(model, camera, tf_api):
    intr = camera.get_camera_intrinsic(agibot_gdk.CameraType.kHeadDepth)
    fx, fy, cx, cy = map(float, list(intr.intrinsic)[:4])
    base_to_camera = (
        transform_matrix(tf_api.get_tf_from_base_link(HEAD_FRAME))
        @ transform_matrix(tf_api.get_tf_from_sensor(
            agibot_gdk.SensorExtrinsicType.kHeadRGBDToHeadLink3))
    )

    samples = []
    deadline = time.time()+DETECTION_TIMEOUT_S
    while time.time() < deadline and len(samples) < DETECTION_SAMPLES:
        co = safe_get_image(camera, agibot_gdk.CameraType.kHeadColor)
        do = safe_get_image(camera, agibot_gdk.CameraType.kHeadDepth)
        if co is None or do is None:
            continue
        color, depth = decode_color(co), decode_depth(do)
        if color is None or depth is None:
            continue
        results = model.predict(color, conf=MIN_CONFIDENCE, verbose=False)
        if not len(results) or results[0].boxes is None or not len(results[0].boxes):
            continue
        box = max(results[0].boxes, key=lambda b: float(b.conf[0]))
        conf = float(box.conf[0])
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        px, py = int((x1+x2)/2), int((y1+y2)/2)
        dh, dw = depth.shape[:2]
        u = max(0, min(dw-1, int(round(px*dw/color.shape[1]))))
        v = max(0, min(dh-1, int(round(py*dh/color.shape[0]))))
        raw = median_depth(depth, u, v)
        if raw is None:
            continue
        z = raw/1000.0 if raw > 20 else raw
        camera_point = np.array([(u-cx)*z/fx, (v-cy)*z/fy, z, 1.0])
        base = (base_to_camera @ camera_point)[:3]
        samples.append((conf, base.tolist(), [px, py], z))
        time.sleep(0.1)

    if len(samples) < max(3, DETECTION_SAMPLES//2):
        raise RuntimeError(f"有效YOLO三维样本不足: {len(samples)}")
    xyz = np.median(np.array([s[1] for s in samples]), axis=0).tolist()
    return {
        "confidence": float(np.median([s[0] for s in samples])),
        "base_xyz": xyz,
        "center_pixel": samples[-1][2],
        "depth_m": float(np.median([s[3] for s in samples])),
        "sample_count": len(samples),
    }


def set_both_arm_pose(robot, left_pose, right_pose):
    req = agibot_gdk.EndEffectorPose()
    req.life_time = LIFE_TIME
    req.group = agibot_gdk.EndEffectorControlGroup.kBothArms
    lp, lq = req.left_end_effector_pose.position, req.left_end_effector_pose.orientation
    rp, rq = req.right_end_effector_pose.position, req.right_end_effector_pose.orientation
    lp.x, lp.y, lp.z = left_pose.position
    lq.x, lq.y, lq.z, lq.w = left_pose.orientation
    rp.x, rp.y, rp.z = right_pose.position
    rq.x, rq.y, rq.z, rq.w = right_pose.orientation
    result = robot.end_effector_pose_control(req)
    if result != 0:
        raise RuntimeError(f"end_effector_pose_control失败:{result}")


def move_tcp_delta(robot, tf_api, left_hold, delta_xyz, fixed_q,
                   baseline, report, label):
    start_end = wait_pose(tf_api, RIGHT_FRAME)
    start_tcp = wait_pose(tf_api, TCP_FRAME)
    target_tcp = [start_tcp.position[i]+delta_xyz[i] for i in range(3)]
    target_end = [start_end.position[i]+delta_xyz[i] for i in range(3)]
    length = math.sqrt(sum(v*v for v in delta_xyz))
    steps = max(2, int(math.ceil(length/CART_STEP_M)))

    if REQUIRE_STAGE_CONFIRMATION:
        if input(f"准备执行{label}，TCP delta={delta_xyz}，输入 NEXT：").strip() != "NEXT":
            raise RuntimeError(f"用户取消{label}")

    for i in range(1, steps+1):
        verify_torque(robot, baseline, report)
        alpha = i/steps
        tcp_z = start_tcp.position[2]+alpha*delta_xyz[2]
        if tcp_z < MIN_TCP_Z_M:
            raise RuntimeError(f"{label}预测TCP高度低于安全门限")
        command = PoseData(lerp(start_end.position, target_end, alpha), fixed_q.copy())
        set_both_arm_pose(robot, left_hold, command)
        time.sleep(DT)

    time.sleep(SETTLE_S)
    actual_tcp = wait_pose(tf_api, TCP_FRAME)
    actual_end = wait_pose(tf_api, RIGHT_FRAME)
    tcp_error = distance(actual_tcp.position, target_tcp)
    q_error = quaternion_error_deg(actual_end.orientation, fixed_q)
    report["cartesian_stages"].append({
        "stage": label, "delta_xyz": delta_xyz,
        "target_tcp": target_tcp, "actual_tcp": actual_tcp.__dict__,
        "tcp_error_m": tcp_error, "orientation_error_deg": q_error,
    })
    save(report)
    print(f"[{label}验证] TCP误差={tcp_error:.4f}m, 姿态误差={q_error:.2f}deg")
    if tcp_error > TCP_TOLERANCE_M or q_error > ORIENTATION_TOLERANCE_DEG:
        raise RuntimeError(f"{label}未到位")


def set_right_gripper(robot, position):
    req = agibot_gdk.JointStates()
    req.group = "right_tool"
    req.target_type = "omnipicker"
    joint = agibot_gdk.JointState()
    joint.position = position
    req.states = [joint]
    req.nums = 1
    result = robot.move_ee_pos(req)
    if result != 0:
        raise RuntimeError(f"右夹爪控制失败:{result}")
    time.sleep(0.7)


def main():
    report = {
        "program": "52_waypoint2_yolo_pick_lift_safe.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "enable_vision_alignment": ENABLE_VISION_ALIGNMENT,
        "enable_gripper_open": ENABLE_GRIPPER_OPEN,
        "enable_descend_grip_lift": ENABLE_DESCEND_GRIP_LIFT,
        "joint_stages": [], "cartesian_stages": [],
        "status": "INITIALIZED",
    }
    initialized = False
    camera = None

    try:
        if not Path(MODEL_PATH).exists():
            raise RuntimeError(f"YOLO模型不存在:{MODEL_PATH}")
        if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("GDK初始化失败")
        initialized = True
        robot = agibot_gdk.Robot()
        tf_api = agibot_gdk.TF()
        camera = agibot_gdk.Camera()
        model = YOLO(MODEL_PATH)
        time.sleep(3.0)

        current_right = positions(robot, RIGHT_JOINTS)
        start_name, start_index, start_error = closest_known_pose(current_right)
        left_hold = wait_pose(tf_api, LEFT_FRAME)
        report.update({
            "recognized_start_pose": start_name,
            "start_pose_error_rad": start_error,
            "initial_right_joints": current_right,
            "initial_tcp": wait_pose(tf_api, TCP_FRAME).__dict__,
        })
        save(report)

        remaining = KEYPOINTS[start_index+1:] if start_index >= 0 else KEYPOINTS
        print("="*78)
        print("52_waypoint2_yolo_pick_lift_safe.py")
        print(f"ENABLE_REAL_MOTION={ENABLE_REAL_MOTION}")
        print(f"ENABLE_VISION_ALIGNMENT={ENABLE_VISION_ALIGNMENT}")
        print(f"ENABLE_GRIPPER_OPEN={ENABLE_GRIPPER_OPEN}")
        print(f"ENABLE_DESCEND_GRIP_LIFT={ENABLE_DESCEND_GRIP_LIFT}")
        print(f"起始姿态={start_name}, 剩余关节路径={[n for n,_ in remaining]}")
        print("到达WAYPOINT_2后才执行YOLO；不使用WAYPOINT_3或旧PREGRASP。")
        print("="*78)

        if not ENABLE_REAL_MOTION:
            report["status"] = "DRY_RUN_PASS"
            save(report)
            print("DRY RUN通过，未发送运动或夹爪命令。")
            return

        if input("确认急停可用、路径无障碍后输入 PICK：").strip() != "PICK":
            report["status"] = "CANCELLED_BY_USER"
            save(report)
            return

        torque_base = make_torque_baseline(robot)
        for label, target in remaining:
            segmented_joint_move(robot, tf_api, label, target, torque_base, report)

        wp2_tcp = wait_pose(tf_api, TCP_FRAME)
        wp2_end = wait_pose(tf_api, RIGHT_FRAME)
        q_error = quaternion_error_deg(wp2_end.orientation, RECORDED_WP2_Q)
        if distance(wp2_tcp.position, RECORDED_TCP["waypoint_2"]) > KEYPOINT_TCP_TOLERANCE_M:
            raise RuntimeError("WAYPOINT_2 TCP未复现")
        if q_error > ORIENTATION_TOLERANCE_DEG:
            raise RuntimeError("WAYPOINT_2姿态未复现")

        print("WAYPOINT_2验证通过，开始YOLO定位产品...")
        detection = detect_product(model, camera, tf_api)
        product = detection["base_xyz"]
        pregrasp_tcp = [product[0], product[1], product[2]+PREGRASP_ABOVE_PRODUCT_M]
        grasp_tcp = [product[0], product[1], product[2]+GRASP_ABOVE_PRODUCT_M]
        lift_tcp = [product[0], product[1], product[2]+GRASP_ABOVE_PRODUCT_M+LIFT_AFTER_GRASP_M]

        dx = pregrasp_tcp[0]-wp2_tcp.position[0]
        dy = pregrasp_tcp[1]-wp2_tcp.position[1]
        dz = pregrasp_tcp[2]-wp2_tcp.position[2]
        xy = math.hypot(dx, dy)
        descent = -dz

        report.update({
            "detection": detection, "product_base_xyz": product,
            "dynamic_targets": {
                "pregrasp_tcp": pregrasp_tcp,
                "grasp_tcp": grasp_tcp,
                "lift_tcp": lift_tcp,
            },
            "wp2_to_pregrasp_delta": [dx, dy, dz],
        })
        save(report)

        print(f"产品Base XYZ={np.round(product,5).tolist()}")
        print(f"动态PreGrasp={np.round(pregrasp_tcp,5).tolist()}")
        print(f"WP2->PreGrasp delta={[round(dx,5),round(dy,5),round(dz,5)]}")

        if xy > MAX_XY_FROM_WP2_M:
            report["status"] = "STOPPED_PRODUCT_XY_OUTSIDE_WINDOW"
            save(report)
            raise RuntimeError(f"产品XY距离WAYPOINT_2过大:{xy:.3f}m")
        if descent < 0 or descent > MAX_PREGRASP_DESCENT_FROM_WP2_M:
            report["status"] = "STOPPED_PREGRASP_Z_OUTSIDE_WINDOW"
            save(report)
            raise RuntimeError(f"动态PreGrasp下降量异常:{descent:.3f}m")
        if grasp_tcp[2] < MIN_TCP_Z_M:
            report["status"] = "STOPPED_GRASP_TCP_TOO_LOW"
            save(report)
            raise RuntimeError("动态抓取TCP低于绝对安全门限")

        if not ENABLE_VISION_ALIGNMENT:
            report["status"] = "YOLO_TARGET_READY_ALIGNMENT_DISABLED"
            save(report)
            print("YOLO目标已生成。视觉对准未启用，安全停止。")
            return

        # 先在WP2高度水平对准产品XY，再垂直下降到PreGrasp。
        move_tcp_delta(robot, tf_api, left_hold, [dx, dy, 0.0],
                       wp2_end.orientation, torque_base, report, "align_xy_at_wp2_height")
        after_xy = wait_pose(tf_api, TCP_FRAME)
        pregrasp_dz = pregrasp_tcp[2]-after_xy.position[2]
        move_tcp_delta(robot, tf_api, left_hold, [0.0, 0.0, pregrasp_dz],
                       wp2_end.orientation, torque_base, report, "descend_to_pregrasp")

        if not ENABLE_GRIPPER_OPEN:
            report["status"] = "PREGRASP_REACHED_GRIPPER_DISABLED"
            save(report)
            print("动态PreGrasp已到达。夹爪动作未启用，安全停止。")
            return

        print("打开右夹爪...")
        set_right_gripper(robot, -0.785)
        report["gripper_opened"] = True
        save(report)

        if not ENABLE_DESCEND_GRIP_LIFT:
            report["status"] = "PREGRASP_REACHED_DESCEND_DISABLED"
            save(report)
            print("右夹爪已打开。下探抓取未启用，安全停止。")
            return

        current_tcp = wait_pose(tf_api, TCP_FRAME)
        grasp_dz = grasp_tcp[2]-current_tcp.position[2]
        move_tcp_delta(robot, tf_api, left_hold, [0.0, 0.0, grasp_dz],
                       wp2_end.orientation, torque_base, report, "descend_to_grasp")

        print("关闭右夹爪...")
        set_right_gripper(robot, 0.0)
        report["gripper_closed"] = True
        save(report)

        move_tcp_delta(robot, tf_api, left_hold, [0.0, 0.0, LIFT_AFTER_GRASP_M],
                       wp2_end.orientation, torque_base, report, "lift_10cm")

        report["final_tcp"] = wait_pose(tf_api, TCP_FRAME).__dict__
        report["status"] = "PICK_AND_LIFT_COMPLETED"
        save(report)
        print("抓取并抬升10cm完成。")

    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED"
        report["error"] = "KeyboardInterrupt"
        save(report)
        print("用户中断，已停止后续动作。")
    except Exception as exc:
        if report.get("status") == "INITIALIZED":
            report["status"] = "ERROR"
        report["error"] = str(exc)
        save(report)
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
