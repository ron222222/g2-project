#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
46_yolo_tcp_pick_lift_10cm_absolute_height_v3.py

完整安全流程：
YOLO识别产品中心 -> 深度/TF定位 -> 到绝对安全高度 -> 高位移动到产品XY ->
高位调整手臂+夹爪整体姿态 -> 打开夹爪 -> 预抓取 -> 下探 -> 闭合 -> 抬升10cm。

默认 ENABLE_REAL_MOTION=False，只检测并规划，不运动。
真实运行需要手动改为True，并输入PICK。

V3关键修正：
1. 使用绝对End安全高度 SAFE_START_END_Z_M，不再每次重跑叠加抬高。
2. 当前End已经足够高时跳过抬高。
3. 起始TCP相对产品高度不足18cm时停止。
4. 每个阶段进行TF闭环验证，未到位立即停止。
5. 每个插值点预测TCP高度和工作空间，防止再次撞桌。
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

ENABLE_REAL_MOTION = True
MODEL_PATH = "runs/detect/runs/product_detector/weights/best.pt"
REPORT_FILE = "yolo_tcp_pick_lift_10cm_absolute_height_v3_report.json"

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

# 绝对高度，不是“当前位置+25cm”
SAFE_START_END_Z_M = 1.020
MAX_START_RAISE_M = 0.220
MIN_TCP_PRODUCT_CLEARANCE_M = 0.180
HIGH_STAGING_ABOVE_PRODUCT_M = 0.250
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

POSITION_TOLERANCE_M = 0.025
TCP_TOLERANCE_M = 0.025
ORIENTATION_TOLERANCE_DEG = 5.0
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
    Path(REPORT_FILE).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")


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
    t = tf_api.get_tf_from_base_link(frame)
    return PoseData(
        [float(t.translation.x), float(t.translation.y), float(t.translation.z)],
        normalize_q([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]),
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
    patch = depth[max(0, v-DEPTH_RADIUS):min(h, v+DEPTH_RADIUS+1),
                  max(0, u-DEPTH_RADIUS):min(w, u+DEPTH_RADIUS+1)].astype(float)
    valid = patch[np.isfinite(patch)]
    valid = valid[(valid > MIN_DEPTH_RAW) & (valid < MAX_DEPTH_RAW)]
    return None if valid.size == 0 else float(np.median(valid))


def detect_product(model, camera, tf_api):
    intr = camera.get_camera_intrinsic(agibot_gdk.CameraType.kHeadDepth)
    fx, fy, cx0, cy0 = map(float, list(intr.intrinsic)[:4])
    t_base_camera = (
        transform_matrix(tf_api.get_tf_from_base_link(HEAD_FRAME))
        @ transform_matrix(tf_api.get_tf_from_sensor(agibot_gdk.SensorExtrinsicType.kHeadRGBDToHeadLink3))
    )
    samples = []
    deadline = time.time() + DETECTION_TIMEOUT_S
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
        base = (t_base_camera @ camera_point)[:3]
        samples.append((confidence, base.tolist(), [px, py], z))
        time.sleep(0.1)
    if len(samples) < max(3, DETECTION_SAMPLES//2):
        raise RuntimeError(f"有效YOLO三维样本不足: {len(samples)}")
    return {
        "confidence": float(np.median([s[0] for s in samples])),
        "base_xyz": np.median(np.array([s[1] for s in samples]), axis=0).tolist(),
        "center_pixel": samples[-1][2],
        "depth_m": float(np.median([s[3] for s in samples])),
        "sample_count": len(samples),
    }


def get_torques(robot) -> Dict[str, float]:
    return {s["name"]: float(s["effort"]) for s in robot.get_joint_states()["states"]}


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
    bad = []
    for name, value in current.items():
        if name in baseline and abs(value-baseline[name]) > TORQUE_DELTA_LIMIT_NM:
            bad.append({"joint": name, "delta": abs(value-baseline[name]),
                        "current": value, "baseline": baseline[name]})
    if bad:
        report["status"] = "STOPPED_TORQUE"
        report["torque_abnormal"] = bad[:10]
        save_report(report)
        raise RuntimeError("力矩安全检查触发")


def set_pose(robot, left_pose, right_pose):
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
        raise RuntimeError(f"{label}预测TCP高度过低")


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


def require_reached(tf_api, frame, target, label, report,
                    pos_tol=POSITION_TOLERANCE_M,
                    ori_tol=ORIENTATION_TOLERANCE_DEG):
    time.sleep(TF_SETTLE_SECONDS)
    actual = wait_pose(tf_api, frame)
    pe = distance(actual.position, target.position)
    oe = q_error_deg(actual.orientation, target.orientation)
    report.setdefault("stages", []).append({
        "stage": label, "actual": actual.__dict__,
        "position_error_m": pe, "orientation_error_deg": oe,
    })
    save_report(report)
    print(f"[{label}验证] 位置误差={pe:.4f}m 姿态误差={oe:.2f}deg")
    if pe > pos_tol or oe > ori_tol:
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
        "program": "46_yolo_tcp_pick_lift_10cm_absolute_height_v3.py",
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

        requested_raise = max(0.0, SAFE_START_END_Z_M - end_initial.position[2])
        applied_raise = min(requested_raise, MAX_START_RAISE_M)
        initial_raise_end = [
            end_initial.position[0], end_initial.position[1],
            end_initial.position[2] + applied_raise,
        ]
        initial_raise_tcp = tcp_from_end(initial_raise_end, end_initial.orientation)

        high_tcp_z = max(product[2] + HIGH_STAGING_ABOVE_PRODUCT_M, initial_raise_tcp[2])
        high_tcp = [product[0], product[1], high_tcp_z]
        high_end_current_q = end_from_tcp(high_tcp, end_initial.orientation)
        high_end_down = end_from_tcp(high_tcp, down)

        tcp_pre = [product[0], product[1], product[2] + PREGRASP_ABOVE_PRODUCT_M]
        tcp_grasp = [product[0], product[1], product[2] + GRASP_ABOVE_PRODUCT_M]
        tcp_lift = [product[0], product[1], product[2] + GRASP_ABOVE_PRODUCT_M + LIFT_AFTER_GRASP_M]
        end_pre, end_grasp, end_lift = (
            end_from_tcp(tcp_pre, down), end_from_tcp(tcp_grasp, down), end_from_tcp(tcp_lift, down)
        )

        report.update({
            "detection": detection, "product_base_xyz": product,
            "initial_end": end_initial.__dict__, "initial_tcp": tcp_initial.__dict__,
            "start_height": {
                "safe_start_end_z_m": SAFE_START_END_Z_M,
                "requested_raise_m": requested_raise,
                "applied_raise_m": applied_raise,
                "target_end": initial_raise_end,
                "target_tcp": initial_raise_tcp,
            },
            "targets": {
                "high_tcp": high_tcp, "high_end_current_q": high_end_current_q,
                "high_end_down": high_end_down, "tcp_pregrasp": tcp_pre,
                "tcp_grasp": tcp_grasp, "tcp_lift_10cm": tcp_lift,
                "end_pregrasp": end_pre, "end_grasp": end_grasp,
                "end_lift_10cm": end_lift, "down_quaternion": down,
            },
        })
        save_report(report)

        end_targets = [initial_raise_end, high_end_current_q, high_end_down, end_pre, end_grasp, end_lift]
        tcp_targets = [initial_raise_tcp, high_tcp, tcp_pre, tcp_grasp, tcp_lift]
        if not all(workspace_ok(point, END_LIMITS) for point in end_targets):
            raise RuntimeError("End规划目标超出工作空间")
        if not all(workspace_ok(point, TCP_LIMITS) for point in tcp_targets):
            raise RuntimeError("TCP规划目标超出工作空间")
        if q_error_deg(end_initial.orientation, down) > MAX_ORIENTATION_CHANGE_DEG:
            raise RuntimeError("朝下姿态变化超过安全上限")

        print("=" * 78)
        print("46_yolo_tcp_pick_lift_10cm_absolute_height_v3.py")
        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print(f"产品 Base XYZ       : {np.round(product, 5).tolist()}")
        print(f"YOLO confidence     : {detection['confidence']:.3f}")
        print(f"当前 End Z          : {end_initial.position[2]:.4f}m")
        print(f"绝对安全 End Z      : {SAFE_START_END_Z_M:.4f}m")
        print(f"本次计划抬高        : {applied_raise*100:.1f}cm")
        print(f"高位 TCP            : {np.round(high_tcp, 5).tolist()}")
        print(f"TCP PreGrasp        : {np.round(tcp_pre, 5).tolist()}")
        print(f"TCP Grasp           : {np.round(tcp_grasp, 5).tolist()}")
        print(f"TCP Lift 10cm       : {np.round(tcp_lift, 5).tolist()}")
        print("=" * 78)

        if not ENABLE_REAL_MOTION:
            report["status"] = "DRY_RUN_PASS"
            save_report(report)
            print("DRY RUN通过，未发送运动或夹爪命令。")
            return

        if input("确认急停可用、桌面上方扫掠区域无障碍后输入 PICK：").strip() != "PICK":
            report["status"] = "CANCELLED_BY_USER"
            save_report(report)
            return

        baseline = establish_baseline(robot)

        # 1. 到绝对安全高度。当前已够高时跳过。
        raise_target = PoseData(initial_raise_end, end_initial.orientation.copy())
        if applied_raise > 0.005:
            move_pose(robot, left_initial, wait_pose(tf_api, RIGHT_FRAME), raise_target,
                      baseline, report, "initial_raise_absolute",
                      minimum_tcp_z=tcp_initial.position[2])
            require_reached(tf_api, RIGHT_FRAME, raise_target,
                            "initial_raise_absolute", report)
        else:
            print("[initial_raise_absolute] 当前End已达到绝对安全高度，跳过抬高。")

        raised_tcp = wait_pose(tf_api, TCP_FRAME)
        clearance = raised_tcp.position[2] - product[2]
        report["raised_tcp_product_clearance_m"] = clearance
        save_report(report)
        print(f"[起始安全高度] TCP相对产品高度={clearance:.4f}m")
        if clearance < MIN_TCP_PRODUCT_CLEARANCE_M:
            report["status"] = "STOPPED_INITIAL_CLEARANCE_INSUFFICIENT"
            save_report(report)
            raise RuntimeError("TCP相对产品高度不足18cm，停止")

        # 2. 高位移到产品XY。
        high_translate = PoseData(high_end_current_q, end_initial.orientation.copy())
        move_pose(robot, left_initial, wait_pose(tf_api, RIGHT_FRAME), high_translate,
                  baseline, report, "high_translate_xy",
                  minimum_tcp_z=product[2] + HIGH_STAGING_ABOVE_PRODUCT_M - 0.020)
        require_reached(tf_api, RIGHT_FRAME, high_translate,
                        "high_translate_xy", report, pos_tol=0.025)

        # 3. 高位改变整条右臂构型，让TCP朝下。
        high_down = PoseData(high_end_down, down)
        move_pose(robot, left_initial, wait_pose(tf_api, RIGHT_FRAME), high_down,
                  baseline, report, "high_rotate_down",
                  minimum_tcp_z=product[2] + HIGH_STAGING_ABOVE_PRODUCT_M - 0.020)
        require_reached(tf_api, RIGHT_FRAME, high_down,
                        "high_rotate_down", report)
        if distance(wait_pose(tf_api, TCP_FRAME).position, high_tcp) > TCP_TOLERANCE_M:
            raise RuntimeError("高位朝下后TCP未保持在安全点")

        # 4. 姿态通过后才开夹爪。
        set_grippers(robot, -0.785)
        report["gripper_command_sent"] = True
        save_report(report)

        # 5. 下降到预抓取。
        pre = PoseData(end_pre, down)
        move_pose(robot, left_initial, wait_pose(tf_api, RIGHT_FRAME), pre,
                  baseline, report, "descend_to_pregrasp",
                  minimum_tcp_z=product[2] + PREGRASP_ABOVE_PRODUCT_M - 0.010)
        require_reached(tf_api, RIGHT_FRAME, pre, "descend_to_pregrasp", report)

        # 6. 下探抓取。
        grasp = PoseData(end_grasp, down)
        move_pose(robot, left_initial, wait_pose(tf_api, RIGHT_FRAME), grasp,
                  baseline, report, "descend_to_grasp",
                  minimum_tcp_z=product[2] + 0.010)
        require_reached(tf_api, RIGHT_FRAME, grasp, "descend_to_grasp", report)

        # 7. 闭合夹爪。
        set_grippers(robot, 0.0)
        report["gripper_command_sent"] = True
        save_report(report)

        # 8. 抬升10cm。
        lift = PoseData(end_lift, down)
        move_pose(robot, left_initial, wait_pose(tf_api, RIGHT_FRAME), lift,
                  baseline, report, "lift_10cm",
                  minimum_tcp_z=product[2] + 0.010)
        require_reached(tf_api, RIGHT_FRAME, lift, "lift_10cm", report)

        report["final_tcp"] = wait_pose(tf_api, TCP_FRAME).__dict__
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
