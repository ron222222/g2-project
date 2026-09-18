#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
46_yolo_tcp_pick_lift_10cm_start_raise_safe.py

安全垂直抓取流程：
YOLO识别 -> 深度/TF定位 -> 第一动作垂直抬高25cm ->
高位移动到产品XY上方 -> 高位围绕TCP旋转至朝下 ->
打开夹爪 -> 下探 -> 关闭夹爪 -> 抬升10cm。

默认 ENABLE_REAL_MOTION=False，只检测和规划，不运动。
真实运行必须手动改为True，并在终端输入 PICK。

重要：任何阶段未达到位置/姿态容差，立即停止，不执行后续动作。
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

# ==================== 用户配置 ====================
ENABLE_REAL_MOTION = True
MODEL_PATH = "runs/detect/runs/product_detector/weights/best.pt"
REPORT_FILE = "yolo_tcp_pick_lift_10cm_start_raise_report.json"

MIN_CONFIDENCE = 0.60
DETECTION_SAMPLES = 8
DETECTION_TIMEOUT_S = 20.0
CAMERA_TIMEOUT_MS = 1000.0
DEPTH_RADIUS = 3
MIN_DEPTH_RAW = 50.0
MAX_DEPTH_RAW = 10000.0

LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"
TCP_FRAME = "gripper_r_center_link"
HEAD_FRAME = "head_link3"
TCP_OFFSET_END_M = [0.0, 0.0, 0.14308]
DOWN_QUATERNION_XYZW = [1.0, 0.0, 0.0, 0.0]

START_RAISE_M = 0.150
MIN_START_RAISE_ACHIEVED_M = 0.200
HIGH_STAGING_ABOVE_PRODUCT_M = 0.300
PREGRASP_ABOVE_PRODUCT_M = 0.100
GRASP_ABOVE_PRODUCT_M = 0.015
LIFT_AFTER_GRASP_M = 0.100

RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
LIFE_TIME = 0.02
TRANSLATION_STEP_M = 0.001
ORIENTATION_STEP_DEG = 0.5
HOLD_SECONDS = 0.30
TF_SETTLE_SECONDS = 0.60

POSITION_TOLERANCE_M = 0.020
TCP_TOLERANCE_M = 0.020
ORIENTATION_TOLERANCE_DEG = 5.0
MAX_TOTAL_MOVE_M = 0.60
MAX_ORIENTATION_CHANGE_DEG = 100.0

BASELINE_SAMPLES = 30
TORQUE_DELTA_LIMIT_NM = 30.0

END_LIMITS = ((0.20, 1.20), (-0.80, 0.80), (0.20, 1.50))
TCP_LIMITS = ((0.20, 1.20), (-0.80, 0.80), (0.20, 1.50))


@dataclass
class PoseData:
    position: List[float]
    orientation: List[float]


def save_report(report):
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def normalize_q(q):
    n = math.sqrt(sum(float(v) ** 2 for v in q))
    if n < 1e-12:
        raise RuntimeError("四元数模长接近0")
    return [float(v) / n for v in q]


def q_error_deg(q1, q2):
    q1, q2 = normalize_q(q1), normalize_q(q2)
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def slerp(q0, q1, alpha):
    q0, q1 = normalize_q(q0), normalize_q(q1)
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0:
        q1, dot = [-v for v in q1], -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return normalize_q([(1-alpha)*a + alpha*b for a, b in zip(q0, q1)])
    theta0 = math.acos(dot)
    theta = theta0 * alpha
    sin0 = math.sin(theta0)
    s0 = math.cos(theta) - dot * math.sin(theta) / sin0
    s1 = math.sin(theta) / sin0
    return normalize_q([s0*a + s1*b for a, b in zip(q0, q1)])


def q_to_r(q):
    x, y, z, w = normalize_q(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=float)


def transform_matrix(t):
    matrix = np.eye(4)
    matrix[:3, :3] = q_to_r([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
    matrix[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return matrix


def distance(a, b):
    return math.sqrt(sum((float(b[i]) - float(a[i])) ** 2 for i in range(3)))


def lerp(a, b, alpha):
    return [(1-alpha)*a[i] + alpha*b[i] for i in range(3)]


def workspace_ok(xyz, limits):
    return all(limits[i][0] < xyz[i] < limits[i][1] for i in range(3))


def end_from_tcp(tcp_xyz, quaternion):
    offset = q_to_r(quaternion).dot(np.array(TCP_OFFSET_END_M, dtype=float))
    return (np.array(tcp_xyz, dtype=float) - offset).tolist()


def tcp_from_end(end_xyz, quaternion):
    offset = q_to_r(quaternion).dot(np.array(TCP_OFFSET_END_M, dtype=float))
    return (np.array(end_xyz, dtype=float) + offset).tolist()


def read_pose(tf_api, frame):
    transform = tf_api.get_tf_from_base_link(frame)
    return PoseData(
        [float(transform.translation.x), float(transform.translation.y), float(transform.translation.z)],
        normalize_q([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]),
    )


def wait_pose(tf_api, frame, timeout=10.0):
    deadline = time.time() + timeout
    last_error = None
    while time.time() < deadline:
        try:
            if tf_api.can_transform("base_link", frame):
                return read_pose(tf_api, frame)
        except Exception as exc:
            last_error = exc
        time.sleep(0.2)
    raise RuntimeError(f"等待TF超时: {frame}; {last_error}")


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
    patch = depth[
        max(0, v-DEPTH_RADIUS):min(h, v+DEPTH_RADIUS+1),
        max(0, u-DEPTH_RADIUS):min(w, u+DEPTH_RADIUS+1),
    ].astype(float)
    valid = patch[np.isfinite(patch)]
    valid = valid[(valid > MIN_DEPTH_RAW) & (valid < MAX_DEPTH_RAW)]
    return None if valid.size == 0 else float(np.median(valid))


def detect_product(model, camera, tf_api):
    intrinsic = camera.get_camera_intrinsic(agibot_gdk.CameraType.kHeadDepth)
    fx, fy, cx0, cy0 = map(float, list(intrinsic.intrinsic)[:4])
    t_base_head = transform_matrix(tf_api.get_tf_from_base_link(HEAD_FRAME))
    t_head_camera = transform_matrix(
        tf_api.get_tf_from_sensor(agibot_gdk.SensorExtrinsicType.kHeadRGBDToHeadLink3)
    )
    t_base_camera = t_base_head @ t_head_camera

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
        px, py = int((x1+x2)/2), int((y1+y2)/2)
        dh, dw = depth.shape[:2]
        du = max(0, min(dw-1, int(round(px*dw/color.shape[1]))))
        dv = max(0, min(dh-1, int(round(py*dh/color.shape[0]))))
        raw = median_depth(depth, du, dv)
        if raw is None:
            continue
        z = raw/1000.0 if raw > 20 else raw
        camera_point = np.array([(du-cx0)*z/fx, (dv-cy0)*z/fy, z, 1.0])
        base_point = (t_base_camera @ camera_point)[:3]
        samples.append((confidence, base_point.tolist(), [px, py], z))
        time.sleep(0.1)

    if len(samples) < max(3, DETECTION_SAMPLES//2):
        raise RuntimeError(f"有效YOLO三维样本不足: {len(samples)}")

    xyz = np.median(np.array([sample[1] for sample in samples]), axis=0).tolist()
    return {
        "confidence": float(np.median([sample[0] for sample in samples])),
        "base_xyz": xyz,
        "center_pixel": samples[-1][2],
        "depth_m": float(np.median([sample[3] for sample in samples])),
        "sample_count": len(samples),
    }


def get_torques(robot) -> Dict[str, float]:
    return {state["name"]: float(state["effort"]) for state in robot.get_joint_states()["states"]}


def establish_baseline(robot):
    samples = []
    print("建立力矩基线，请保持机器人静止...")
    for _ in range(BASELINE_SAMPLES):
        samples.append(get_torques(robot))
        time.sleep(0.05)
    names = set().union(*(sample.keys() for sample in samples))
    return {name: sum(sample.get(name, 0.0) for sample in samples)/len(samples) for name in names}


def verify_torque(robot, baseline, report):
    current = get_torques(robot)
    abnormal = []
    for name, value in current.items():
        if name in baseline and abs(value-baseline[name]) > TORQUE_DELTA_LIMIT_NM:
            abnormal.append({
                "joint": name,
                "delta": abs(value-baseline[name]),
                "current": value,
                "baseline": baseline[name],
            })
    if abnormal:
        report["status"] = "STOPPED_TORQUE"
        report["torque_abnormal"] = abnormal[:10]
        save_report(report)
        raise RuntimeError("力矩安全检查触发")


def set_pose(robot, left_pose, right_pose):
    request = agibot_gdk.EndEffectorPose()
    request.life_time = LIFE_TIME
    request.group = agibot_gdk.EndEffectorControlGroup.kBothArms
    lp, lq = request.left_end_effector_pose.position, request.left_end_effector_pose.orientation
    rp, rq = request.right_end_effector_pose.position, request.right_end_effector_pose.orientation
    lp.x, lp.y, lp.z = left_pose.position
    lq.x, lq.y, lq.z, lq.w = left_pose.orientation
    rp.x, rp.y, rp.z = right_pose.position
    rq.x, rq.y, rq.z, rq.w = right_pose.orientation
    result = robot.end_effector_pose_control(request)
    if result != 0:
        raise RuntimeError(f"end_effector_pose_control失败: {result}")


def validate_command_tcp(command, minimum_tcp_z, report, label):
    tcp_xyz = tcp_from_end(command.position, command.orientation)
    if not workspace_ok(command.position, END_LIMITS):
        report["status"] = f"STOPPED_{label.upper()}_END_WORKSPACE"
        save_report(report)
        raise RuntimeError(f"{label} End命令超出工作空间")
    if not workspace_ok(tcp_xyz, TCP_LIMITS):
        report["status"] = f"STOPPED_{label.upper()}_TCP_WORKSPACE"
        save_report(report)
        raise RuntimeError(f"{label} TCP命令超出工作空间")
    if minimum_tcp_z is not None and tcp_xyz[2] < minimum_tcp_z:
        report["status"] = f"STOPPED_{label.upper()}_TCP_TOO_LOW"
        report["unsafe_tcp_xyz"] = tcp_xyz
        save_report(report)
        raise RuntimeError(
            f"{label}预测TCP高度过低: {tcp_xyz[2]:.4f} < {minimum_tcp_z:.4f}"
        )


def move_pose(robot, left_pose, start_pose, target_pose, baseline, report, label, minimum_tcp_z=None):
    translation = distance(start_pose.position, target_pose.position)
    angle = q_error_deg(start_pose.orientation, target_pose.orientation)
    steps = max(2, int(math.ceil(max(translation/TRANSLATION_STEP_M, angle/ORIENTATION_STEP_DEG))))
    print(f"[{label}] 位移={translation:.4f}m 姿态={angle:.2f}deg 步数={steps}")

    for index in range(1, steps+1):
        verify_torque(robot, baseline, report)
        alpha = index/steps
        command = PoseData(
            lerp(start_pose.position, target_pose.position, alpha),
            slerp(start_pose.orientation, target_pose.orientation, alpha),
        )
        validate_command_tcp(command, minimum_tcp_z, report, label)
        set_pose(robot, left_pose, command)
        report["motion_command_sent"] = True
        time.sleep(DT)

    for _ in range(max(1, int(HOLD_SECONDS*RATE_HZ))):
        set_pose(robot, left_pose, target_pose)
        time.sleep(DT)


def require_reached(tf_api, frame, target, label, report, pos_tol=POSITION_TOLERANCE_M, ori_tol=ORIENTATION_TOLERANCE_DEG):
    time.sleep(TF_SETTLE_SECONDS)
    actual = wait_pose(tf_api, frame)
    position_error = distance(actual.position, target.position)
    orientation_error = q_error_deg(actual.orientation, target.orientation)
    report.setdefault("stages", []).append({
        "stage": label,
        "actual": actual.__dict__,
        "position_error_m": position_error,
        "orientation_error_deg": orientation_error,
    })
    save_report(report)
    print(f"[{label}验证] 位置误差={position_error:.4f}m 姿态误差={orientation_error:.2f}deg")
    if position_error > pos_tol or orientation_error > ori_tol:
        report["status"] = f"STOPPED_{label.upper()}_NOT_REACHED"
        save_report(report)
        raise RuntimeError(f"{label}未到位，停止后续动作")
    return actual


def set_grippers(robot, position):
    states = agibot_gdk.JointStates()
    states.group = "dual_tool"
    states.target_type = "omnipicker"
    left_joint = agibot_gdk.JointState()
    right_joint = agibot_gdk.JointState()
    left_joint.position = position
    right_joint.position = position
    states.states = [left_joint, right_joint]
    states.nums = 2
    result = robot.move_ee_pos(states)
    if result != 0:
        raise RuntimeError(f"夹爪控制失败: {result}")
    time.sleep(0.6)


def main():
    report = {
        "program": "46_yolo_tcp_pick_lift_10cm_start_raise_safe.py",
        "enable_real_motion": ENABLE_REAL_MOTION,
        "motion_command_sent": False,
        "gripper_command_sent": False,
        "status": "INITIALIZED",
        "stages": [],
    }
    initialized = False
    camera = None

    try:
        if not Path(MODEL_PATH).exists():
            raise RuntimeError(f"模型不存在: {MODEL_PATH}")
        if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("GDK初始化失败")
        initialized = True

        robot = agibot_gdk.Robot()
        camera = agibot_gdk.Camera()
        tf_api = agibot_gdk.TF()
        model = YOLO(MODEL_PATH)
        time.sleep(3.0)

        left_initial = wait_pose(tf_api, LEFT_FRAME)
        end_initial = wait_pose(tf_api, RIGHT_FRAME)
        tcp_initial = wait_pose(tf_api, TCP_FRAME)
        detection = detect_product(model, camera, tf_api)
        product = detection["base_xyz"]
        down = normalize_q(DOWN_QUATERNION_XYZW)

        initial_raise_end = [
            end_initial.position[0],
            end_initial.position[1],
            end_initial.position[2] + START_RAISE_M,
        ]
        initial_raise_tcp = tcp_from_end(initial_raise_end, end_initial.orientation)

        high_tcp_z = max(
            product[2] + HIGH_STAGING_ABOVE_PRODUCT_M,
            initial_raise_tcp[2],
        )
        high_tcp_current_orientation = [product[0], product[1], high_tcp_z]
        high_end_current_orientation = end_from_tcp(
            high_tcp_current_orientation, end_initial.orientation
        )
        high_end_down = end_from_tcp(high_tcp_current_orientation, down)

        tcp_pregrasp = [product[0], product[1], product[2] + PREGRASP_ABOVE_PRODUCT_M]
        tcp_grasp = [product[0], product[1], product[2] + GRASP_ABOVE_PRODUCT_M]
        tcp_lift = [product[0], product[1], product[2] + GRASP_ABOVE_PRODUCT_M + LIFT_AFTER_GRASP_M]
        end_pregrasp = end_from_tcp(tcp_pregrasp, down)
        end_grasp = end_from_tcp(tcp_grasp, down)
        end_lift = end_from_tcp(tcp_lift, down)

        report.update({
            "detection": detection,
            "product_base_xyz": product,
            "initial_end": end_initial.__dict__,
            "initial_tcp": tcp_initial.__dict__,
            "targets": {
                "initial_raise_end": initial_raise_end,
                "initial_raise_tcp": initial_raise_tcp,
                "high_tcp_staging": high_tcp_current_orientation,
                "high_end_current_orientation": high_end_current_orientation,
                "high_end_down": high_end_down,
                "tcp_pregrasp": tcp_pregrasp,
                "tcp_grasp": tcp_grasp,
                "tcp_lift_10cm": tcp_lift,
                "end_pregrasp": end_pregrasp,
                "end_grasp": end_grasp,
                "end_lift_10cm": end_lift,
                "down_quaternion_xyzw": down,
            },
        })
        save_report(report)

        all_end_targets = [initial_raise_end, high_end_current_orientation, high_end_down, end_pregrasp, end_grasp, end_lift]
        all_tcp_targets = [initial_raise_tcp, high_tcp_current_orientation, tcp_pregrasp, tcp_grasp, tcp_lift]
        if not all(workspace_ok(point, END_LIMITS) for point in all_end_targets):
            raise RuntimeError("规划的End目标超出工作空间")
        if not all(workspace_ok(point, TCP_LIMITS) for point in all_tcp_targets):
            raise RuntimeError("规划的TCP目标超出工作空间")
        if distance(end_initial.position, initial_raise_end) > MAX_TOTAL_MOVE_M:
            raise RuntimeError("起始抬高距离异常")
        if q_error_deg(end_initial.orientation, down) > MAX_ORIENTATION_CHANGE_DEG:
            raise RuntimeError("朝下姿态变化超过安全上限")

        print("=" * 78)
        print("46_yolo_tcp_pick_lift_10cm_start_raise_safe.py")
        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print(f"产品 Base XYZ       : {np.round(product, 5).tolist()}")
        print(f"YOLO confidence     : {detection['confidence']:.3f}")
        print(f"第一动作抬高        : {START_RAISE_M*100:.0f} cm")
        print(f"抬高后 TCP XYZ      : {np.round(initial_raise_tcp, 5).tolist()}")
        print(f"高位姿态切换 TCP    : {np.round(high_tcp_current_orientation, 5).tolist()}")
        print(f"TCP PreGrasp        : {np.round(tcp_pregrasp, 5).tolist()}")
        print(f"TCP Grasp           : {np.round(tcp_grasp, 5).tolist()}")
        print(f"TCP Lift 10cm       : {np.round(tcp_lift, 5).tolist()}")
        print("流程: 抬高25cm -> 高位移到产品XY -> 高位旋转朝下 -> 打开夹爪 -> 下探 -> 闭合 -> 抬升10cm")
        print("=" * 78)

        if not ENABLE_REAL_MOTION:
            report["status"] = "DRY_RUN_PASS"
            save_report(report)
            print("DRY RUN通过，未发送运动或夹爪命令。")
            return

        confirmation = input(
            "确认急停可用、桌面上方扫掠区域无障碍后输入 PICK："
        ).strip()
        if confirmation != "PICK":
            report["status"] = "CANCELLED_BY_USER"
            save_report(report)
            return

        baseline = establish_baseline(robot)

        # Stage 1: 第一动作垂直抬高25cm，保持当前姿态。
        raise_target = PoseData(initial_raise_end, end_initial.orientation.copy())
        move_pose(
            robot, left_initial, wait_pose(tf_api, RIGHT_FRAME), raise_target,
            baseline, report, "initial_raise_25cm",
            minimum_tcp_z=tcp_initial.position[2],
        )
        require_reached(tf_api, RIGHT_FRAME, raise_target, "initial_raise_25cm", report)
        raised_tcp = wait_pose(tf_api, TCP_FRAME)
        achieved_raise = raised_tcp.position[2] - tcp_initial.position[2]
        report["achieved_start_raise_m"] = achieved_raise
        save_report(report)
        print(f"[起始抬高验证] TCP实际抬高={achieved_raise:.4f}m")
        if achieved_raise < MIN_START_RAISE_ACHIEVED_M:
            report["status"] = "STOPPED_INITIAL_RAISE_INSUFFICIENT"
            save_report(report)
            raise RuntimeError("TCP实际抬高不足20cm，禁止继续")

        # Stage 2: 保持启动姿态，在高位把TCP移到产品XY上方。
        high_translate_target = PoseData(
            high_end_current_orientation,
            end_initial.orientation.copy(),
        )
        move_pose(
            robot, left_initial, wait_pose(tf_api, RIGHT_FRAME), high_translate_target,
            baseline, report, "high_translate_xy",
            minimum_tcp_z=product[2] + HIGH_STAGING_ABOVE_PRODUCT_M - 0.020,
        )
        require_reached(tf_api, RIGHT_FRAME, high_translate_target, "high_translate_xy", report)

        # Stage 3: 在产品上方高位改变整条右臂构型，使TCP朝下。
        high_down_target = PoseData(high_end_down, down)
        move_pose(
            robot, left_initial, wait_pose(tf_api, RIGHT_FRAME), high_down_target,
            baseline, report, "high_rotate_down",
            minimum_tcp_z=product[2] + HIGH_STAGING_ABOVE_PRODUCT_M - 0.020,
        )
        require_reached(tf_api, RIGHT_FRAME, high_down_target, "high_rotate_down", report)
        high_tcp_actual = wait_pose(tf_api, TCP_FRAME)
        if distance(high_tcp_actual.position, high_tcp_current_orientation) > TCP_TOLERANCE_M:
            raise RuntimeError("高位朝下后TCP未保持在安全点")

        # 姿态和高位检查通过后才打开夹爪。
        set_grippers(robot, -0.785)
        report["gripper_command_sent"] = True
        save_report(report)

        # Stage 4: 垂直下降到产品上方10cm。
        pregrasp_target = PoseData(end_pregrasp, down)
        move_pose(
            robot, left_initial, wait_pose(tf_api, RIGHT_FRAME), pregrasp_target,
            baseline, report, "descend_to_pregrasp",
            minimum_tcp_z=product[2] + PREGRASP_ABOVE_PRODUCT_M - 0.010,
        )
        require_reached(tf_api, RIGHT_FRAME, pregrasp_target, "descend_to_pregrasp", report)

        # Stage 5: 垂直下探到产品中心上方15mm。
        grasp_target = PoseData(end_grasp, down)
        move_pose(
            robot, left_initial, wait_pose(tf_api, RIGHT_FRAME), grasp_target,
            baseline, report, "descend_to_grasp",
            minimum_tcp_z=product[2] + 0.010,
        )
        require_reached(tf_api, RIGHT_FRAME, grasp_target, "descend_to_grasp", report)

        # Stage 6: 关闭夹爪。
        set_grippers(robot, 0.0)
        report["gripper_command_sent"] = True
        save_report(report)

        # Stage 7: 垂直抬升10cm。
        lift_target = PoseData(end_lift, down)
        move_pose(
            robot, left_initial, wait_pose(tf_api, RIGHT_FRAME), lift_target,
            baseline, report, "lift_10cm",
            minimum_tcp_z=product[2] + 0.010,
        )
        require_reached(tf_api, RIGHT_FRAME, lift_target, "lift_10cm", report)

        final_tcp = wait_pose(tf_api, TCP_FRAME)
        report["final_tcp"] = final_tcp.__dict__
        report["status"] = "PICK_AND_LIFT_COMPLETED"
        save_report(report)
        print("抓取并抬升10cm流程完成。")

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
        if camera is not None:
            try:
                camera.close_camera()
            except Exception:
                pass
        if initialized:
            agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
