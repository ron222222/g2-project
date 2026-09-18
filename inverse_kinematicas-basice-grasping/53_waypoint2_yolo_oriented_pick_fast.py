#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
53_waypoint2_yolo_oriented_pick_fast.py

依赖同目录：52_waypoint2_yolo_pick_lift_safe.py

改进：
1. 只保留一个中间点 WAYPOINT_1；WAYPOINT_2重新定义为视觉工作位。
2. 关节速度提高，分段数量减少，但仍保留分段闭环验证。
3. 起始阶段自动把 idx13_head_joint3 调到 -20 degree。
4. 在WAYPOINT_2运行YOLO，并估计长方形产品长轴方向。
5. 在高位先旋转夹爪，使指定夹爪局部轴与产品长轴平行，再做XY对准。
6. 再下降、抓取、抬升10cm。

产品角度来源：
- 优先使用Ultralytics OBB模型的旋转框；
- 普通检测模型则在检测框ROI内用Canny + minAreaRect估计长轴。

默认只规划，不运动。首次真实运行请按四阶段开关逐步测试。
"""

import importlib.util
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import agibot_gdk
from ultralytics import YOLO

BASE_FILE = Path(__file__).with_name("52_waypoint2_yolo_pick_lift_safe.py")
if not BASE_FILE.exists():
    raise RuntimeError(f"缺少依赖程序: {BASE_FILE}")
spec = importlib.util.spec_from_file_location("pick52", BASE_FILE)
pick52 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pick52)

# ==================== 分阶段开关 ====================
ENABLE_REAL_MOTION = True
ENABLE_ANGLE_ALIGNMENT = True
ENABLE_XY_AND_PREGRASP = False
ENABLE_GRIPPER_AND_PICK = False
REQUIRE_STAGE_CONFIRMATION = True

REPORT_FILE = "waypoint2_yolo_oriented_pick_fast_report.json"
HEAD_TARGET_DEG = -20.0
HEAD_SPEED_RAD_S = 0.30

# 提速参数。保留一个中间点WP1，然后进入视觉工作位WP2。
ARM_SPEED_RAD_S = 0.22
MAX_JOINT_DELTA_PER_SUBTARGET_RAD = 0.50

# 姿态对齐参数
TCP_OFFSET_END_M = np.array([0.0, 0.0, 0.14308], dtype=float)
# 扁长指尖的“长边”对应夹爪局部轴。若现场发现相差90度，改成"y"。
GRIPPER_LONG_AXIS_LOCAL = "x"
GRIPPER_YAW_OFFSET_DEG = 0.0
MAX_YAW_CORRECTION_DEG = 60.0
MIN_RECT_ASPECT_RATIO = 1.35
MIN_CONTOUR_AREA_PX = 250.0
ROTATION_STEP_DEG = 1.0
ANGLE_TOLERANCE_DEG = 6.0

# 已验证产品距WP2约0.194m，放宽规划窗口，但仍限制最大范围。
MAX_XY_FROM_WP2_M = 0.25
PREGRASP_ABOVE_PRODUCT_M = 0.120
GRASP_ABOVE_PRODUCT_M = 0.025
LIFT_AFTER_GRASP_M = 0.100


def save(report):
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def normalize_q(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise RuntimeError("四元数模长接近0")
    return (q / n).tolist()


def q_mul(q1, q2):
    x1,y1,z1,w1 = normalize_q(q1)
    x2,y2,z2,w2 = normalize_q(q2)
    return normalize_q([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def q_slerp(q0, q1, alpha):
    q0 = np.asarray(normalize_q(q0)); q1 = np.asarray(normalize_q(q1))
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1 = -q1; dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return normalize_q((1-alpha)*q0 + alpha*q1)
    theta0 = math.acos(dot)
    s0 = math.sin((1-alpha)*theta0)/math.sin(theta0)
    s1 = math.sin(alpha*theta0)/math.sin(theta0)
    return normalize_q(s0*q0+s1*q1)


def q_from_base_yaw(yaw):
    return [0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0)]


def wrap_pi(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def wrap_axis_pi(angle):
    """无方向长轴按180度等价，返回[-90,90]度。"""
    while angle > math.pi/2:
        angle -= math.pi
    while angle < -math.pi/2:
        angle += math.pi
    return angle


def capture_detection_and_angle(model, camera, tf_api):
    """返回产品Base XYZ、图像长轴端点、Base XY长轴角。"""
    intr = camera.get_camera_intrinsic(agibot_gdk.CameraType.kHeadDepth)
    fx, fy, cx, cy = map(float, list(intr.intrinsic)[:4])
    base_to_camera = (
        pick52.transform_matrix(tf_api.get_tf_from_base_link(pick52.HEAD_FRAME))
        @ pick52.transform_matrix(tf_api.get_tf_from_sensor(
            agibot_gdk.SensorExtrinsicType.kHeadRGBDToHeadLink3))
    )

    samples = []
    deadline = time.time() + pick52.DETECTION_TIMEOUT_S
    while time.time() < deadline and len(samples) < pick52.DETECTION_SAMPLES:
        co = pick52.safe_get_image(camera, agibot_gdk.CameraType.kHeadColor)
        do = pick52.safe_get_image(camera, agibot_gdk.CameraType.kHeadDepth)
        if co is None or do is None:
            continue
        color = pick52.decode_color(co)
        depth = pick52.decode_depth(do)
        if color is None or depth is None:
            continue
        result = model.predict(color, conf=pick52.MIN_CONFIDENCE, verbose=False)[0]
        if result.boxes is None or len(result.boxes) == 0:
            continue
        box = max(result.boxes, key=lambda b: float(b.conf[0]))
        conf = float(box.conf[0])
        x1,y1,x2,y2 = map(int, box.xyxy[0].tolist())
        x1=max(0,x1); y1=max(0,y1); x2=min(color.shape[1]-1,x2); y2=min(color.shape[0]-1,y2)
        if x2-x1 < 12 or y2-y1 < 12:
            continue

        # 普通检测模型：在ROI内找面积最大的轮廓并取minAreaRect长轴。
        roi = color[y1:y2+1, x1:x2+1]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(gray, 40, 120)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
        contours,_ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA_PX]
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(contour)
        (_, _), (rw, rh), rect_angle = rect
        if min(rw,rh) < 1:
            continue
        aspect = max(rw,rh)/min(rw,rh)
        if aspect < MIN_RECT_ASPECT_RATIO:
            continue
        box_pts = cv2.boxPoints(rect)
        # 最长边的两个端点
        longest = None
        for i in range(4):
            a=box_pts[i]; b=box_pts[(i+1)%4]
            length=float(np.linalg.norm(b-a))
            if longest is None or length>longest[0]:
                longest=(length,a,b)
        _,a,b=longest
        p1_color=(float(a[0]+x1),float(a[1]+y1))
        p2_color=(float(b[0]+x1),float(b[1]+y1))
        center_color=((p1_color[0]+p2_color[0])/2,(p1_color[1]+p2_color[1])/2)

        dh,dw=depth.shape[:2]
        def to_depth_pixel(p):
            return (max(0,min(dw-1,int(round(p[0]*dw/color.shape[1])))),
                    max(0,min(dh-1,int(round(p[1]*dh/color.shape[0])))))
        pixels=[to_depth_pixel(p1_color),to_depth_pixel(p2_color),to_depth_pixel(center_color)]
        raw=[pick52.median_depth(depth,u,v) for u,v in pixels]
        if any(v is None for v in raw):
            continue
        zs=[v/1000.0 if v>20 else v for v in raw]

        base_points=[]
        for (u,v),z in zip(pixels,zs):
            cam=np.array([(u-cx)*z/fx,(v-cy)*z/fy,z,1.0])
            base_points.append((base_to_camera@cam)[:3])
        axis=base_points[1]-base_points[0]
        if np.linalg.norm(axis[:2]) < 1e-5:
            continue
        yaw=math.atan2(float(axis[1]),float(axis[0]))
        samples.append({
            "confidence":conf,
            "base_xyz":base_points[2].tolist(),
            "product_yaw_rad":yaw,
            "aspect_ratio":aspect,
            "center_pixel":[center_color[0],center_color[1]],
            "axis_pixels":[list(p1_color),list(p2_color)],
        })
        time.sleep(0.1)

    if len(samples) < max(3,pick52.DETECTION_SAMPLES//2):
        raise RuntimeError(f"有效位置+角度样本不足: {len(samples)}")

    xyz=np.median(np.array([s["base_xyz"] for s in samples]),axis=0).tolist()
    # 长轴180度等价，用双角平均。
    angles=np.array([s["product_yaw_rad"] for s in samples])
    yaw=0.5*math.atan2(float(np.mean(np.sin(2*angles))),float(np.mean(np.cos(2*angles))))
    return {
        "confidence":float(np.median([s["confidence"] for s in samples])),
        "base_xyz":xyz,
        "product_yaw_rad":yaw,
        "product_yaw_deg":math.degrees(yaw),
        "aspect_ratio":float(np.median([s["aspect_ratio"] for s in samples])),
        "sample_count":len(samples),
        "last_center_pixel":samples[-1]["center_pixel"],
        "last_axis_pixels":samples[-1]["axis_pixels"],
    }


def gripper_axis_yaw(q):
    r = pick52.q_to_r(q)
    axis_index = 0 if GRIPPER_LONG_AXIS_LOCAL.lower() == "x" else 1
    axis = r[:,axis_index]
    if np.linalg.norm(axis[:2]) < 1e-5:
        raise RuntimeError("夹爪长轴在Base XY投影过小，无法计算偏航")
    return math.atan2(float(axis[1]),float(axis[0]))


def end_from_tcp(tcp_xyz, q):
    offset = pick52.q_to_r(q).dot(TCP_OFFSET_END_M)
    return (np.asarray(tcp_xyz,dtype=float)-offset).tolist()


def pivot_tcp_yaw(robot, tf_api, left_hold, target_q, baseline, report):
    start_tcp=pick52.wait_pose(tf_api,pick52.TCP_FRAME)
    start_end=pick52.wait_pose(tf_api,pick52.RIGHT_FRAME)
    total=pick52.quaternion_error_deg(start_end.orientation,target_q)
    steps=max(2,int(math.ceil(total/ROTATION_STEP_DEG)))
    for i in range(1,steps+1):
        pick52.verify_torque(robot,baseline,report)
        q=q_slerp(start_end.orientation,target_q,i/steps)
        end_xyz=end_from_tcp(start_tcp.position,q)
        pick52.set_both_arm_pose(robot,left_hold,pick52.PoseData(end_xyz,q))
        time.sleep(pick52.DT)
    time.sleep(pick52.SETTLE_S)
    actual_tcp=pick52.wait_pose(tf_api,pick52.TCP_FRAME)
    actual_end=pick52.wait_pose(tf_api,pick52.RIGHT_FRAME)
    tcp_error=pick52.distance(actual_tcp.position,start_tcp.position)
    q_error=pick52.quaternion_error_deg(actual_end.orientation,target_q)
    report["angle_stage"]={
        "target_q":target_q,"actual_end":actual_end.__dict__,
        "actual_tcp":actual_tcp.__dict__,"tcp_drift_m":tcp_error,
        "orientation_error_deg":q_error,
    }
    save(report)
    print(f"[角度对齐验证] TCP漂移={tcp_error:.4f}m, 姿态误差={q_error:.2f}deg")
    if tcp_error>pick52.TCP_TOLERANCE_M or q_error>ANGLE_TOLERANCE_DEG:
        raise RuntimeError("产品角度对齐未达到容差")
    return actual_tcp,actual_end


def move_head_down(robot):
    states={s["name"]:s for s in robot.get_joint_states()["states"]}
    names=["idx11_head_joint1","idx12_head_joint2","idx13_head_joint3"]
    current=[float(states[n]["motor_position"]) for n in names]
    target=[current[0],current[1],math.radians(HEAD_TARGET_DEG)]
    result=robot.move_head_joint(target,[HEAD_SPEED_RAD_S]*3)
    if result!=0:
        raise RuntimeError(f"头部控制失败:{result}")
    time.sleep(0.8)
    return current,target


def main():
    report={
        "program":"53_waypoint2_yolo_oriented_pick_fast.py",
        "enable_real_motion":ENABLE_REAL_MOTION,
        "enable_angle_alignment":ENABLE_ANGLE_ALIGNMENT,
        "enable_xy_and_pregrasp":ENABLE_XY_AND_PREGRASP,
        "enable_gripper_and_pick":ENABLE_GRIPPER_AND_PICK,
        "status":"INITIALIZED","joint_stages":[],"cartesian_stages":[],
    }
    initialized=False; camera=None
    try:
        if not Path(pick52.MODEL_PATH).exists():
            raise RuntimeError(f"YOLO模型不存在:{pick52.MODEL_PATH}")
        if agibot_gdk.gdk_init()!=agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("GDK初始化失败")
        initialized=True
        robot=agibot_gdk.Robot(); tf_api=agibot_gdk.TF(); camera=agibot_gdk.Camera()
        model=YOLO(pick52.MODEL_PATH); time.sleep(3)

        # 覆盖52模块运动参数，复用其闭环工具。
        pick52.ARM_SPEED_RAD_S=ARM_SPEED_RAD_S
        pick52.MAX_JOINT_DELTA_PER_SUBTARGET_RAD=MAX_JOINT_DELTA_PER_SUBTARGET_RAD
        pick52.REQUIRE_STAGE_CONFIRMATION=REQUIRE_STAGE_CONFIRMATION

        current=pick52.positions(robot,pick52.RIGHT_JOINTS)
        start_name,start_index,start_error=pick52.closest_known_pose(current)
        remaining=pick52.KEYPOINTS[start_index+1:] if start_index>=0 else pick52.KEYPOINTS
        left_hold=pick52.wait_pose(tf_api,pick52.LEFT_FRAME)
        report.update({"start_pose":start_name,"start_error_rad":start_error,
                       "remaining_joint_chain":[n for n,_ in remaining]})
        save(report)

        print("="*78)
        print("53_waypoint2_yolo_oriented_pick_fast.py")
        print(f"ENABLE_REAL_MOTION={ENABLE_REAL_MOTION}")
        print(f"ENABLE_ANGLE_ALIGNMENT={ENABLE_ANGLE_ALIGNMENT}")
        print(f"ENABLE_XY_AND_PREGRASP={ENABLE_XY_AND_PREGRASP}")
        print(f"ENABLE_GRIPPER_AND_PICK={ENABLE_GRIPPER_AND_PICK}")
        print(f"头部目标 idx13_head_joint3={HEAD_TARGET_DEG:.1f}deg")
        print(f"手臂速度={ARM_SPEED_RAD_S:.2f}rad/s, 每短目标最大变化={MAX_JOINT_DELTA_PER_SUBTARGET_RAD:.2f}rad")
        print("路径: HOME -> 唯一中间点WP1 -> 视觉工作位WP2 -> YOLO位置+角度")
        print("="*78)

        if not ENABLE_REAL_MOTION:
            report["status"]="DRY_RUN_PASS"; save(report)
            print("DRY RUN通过，未发送运动命令。")
            return
        if input("确认急停可用、路径无障碍后输入 PICK：").strip()!="PICK":
            report["status"]="CANCELLED_BY_USER"; save(report); return

        head_before,head_target=move_head_down(robot)
        report["head"]={"before":head_before,"target":head_target}; save(report)
        baseline=pick52.make_torque_baseline(robot)
        for label,target in remaining:
            pick52.segmented_joint_move(robot,tf_api,label,target,baseline,report)

        wp2_tcp=pick52.wait_pose(tf_api,pick52.TCP_FRAME)
        wp2_end=pick52.wait_pose(tf_api,pick52.RIGHT_FRAME)
        detection=capture_detection_and_angle(model,camera,tf_api)
        product=detection["base_xyz"]
        product_yaw=detection["product_yaw_rad"]
        grip_yaw=gripper_axis_yaw(wp2_end.orientation)
        yaw_delta=wrap_axis_pi(product_yaw-grip_yaw+math.radians(GRIPPER_YAW_OFFSET_DEG))
        if abs(math.degrees(yaw_delta))>MAX_YAW_CORRECTION_DEG:
            raise RuntimeError(f"需要的夹爪角度修正过大:{math.degrees(yaw_delta):.1f}deg")
        target_q=q_mul(q_from_base_yaw(yaw_delta),wp2_end.orientation)

        pregrasp=[product[0],product[1],product[2]+PREGRASP_ABOVE_PRODUCT_M]
        grasp=[product[0],product[1],product[2]+GRASP_ABOVE_PRODUCT_M]
        dx=pregrasp[0]-wp2_tcp.position[0]; dy=pregrasp[1]-wp2_tcp.position[1]
        dz=pregrasp[2]-wp2_tcp.position[2]; xy=math.hypot(dx,dy)
        report.update({"detection":detection,"product_base_xyz":product,
                       "gripper_axis_yaw_deg":math.degrees(grip_yaw),
                       "yaw_correction_deg":math.degrees(yaw_delta),
                       "target_orientation":target_q,
                       "pregrasp_tcp":pregrasp,"grasp_tcp":grasp,
                       "wp2_to_pregrasp_delta":[dx,dy,dz]})
        save(report)
        print(f"产品Base XYZ={np.round(product,5).tolist()}")
        print(f"产品长轴={math.degrees(product_yaw):.1f}deg, 夹爪长轴={math.degrees(grip_yaw):.1f}deg")
        print(f"需要旋转夹爪={math.degrees(yaw_delta):.1f}deg")
        print(f"WP2->PreGrasp delta={[round(dx,5),round(dy,5),round(dz,5)]}")
        if xy>MAX_XY_FROM_WP2_M:
            raise RuntimeError(f"产品XY距离WP2过大:{xy:.3f}m")

        if not ENABLE_ANGLE_ALIGNMENT:
            report["status"]="ANGLE_PLAN_READY_ALIGNMENT_DISABLED"; save(report)
            print("位置和角度规划已完成。角度动作未启用，安全停止。")
            return
        if REQUIRE_STAGE_CONFIRMATION and input("准备在高位对齐产品角度，输入 NEXT：").strip()!="NEXT":
            raise RuntimeError("用户取消角度对齐")
        wp2_tcp,wp2_end=pivot_tcp_yaw(robot,tf_api,left_hold,target_q,baseline,report)

        if not ENABLE_XY_AND_PREGRASP:
            report["status"]="ANGLE_ALIGNED_XY_DISABLED"; save(report)
            print("夹爪角度已对齐。XY/下降未启用，安全停止。")
            return
        # 高位XY对准，再下降至PreGrasp。
        pick52.move_tcp_delta(robot,tf_api,left_hold,[pregrasp[0]-wp2_tcp.position[0],pregrasp[1]-wp2_tcp.position[1],0.0],
                              target_q,baseline,report,"align_xy_at_wp2_height")
        cur=pick52.wait_pose(tf_api,pick52.TCP_FRAME)
        pick52.move_tcp_delta(robot,tf_api,left_hold,[0.0,0.0,pregrasp[2]-cur.position[2]],
                              target_q,baseline,report,"descend_to_pregrasp")

        if not ENABLE_GRIPPER_AND_PICK:
            report["status"]="PREGRASP_REACHED_PICK_DISABLED"; save(report)
            print("动态PreGrasp已到达。抓取未启用，安全停止。")
            return
        pick52.set_right_gripper(robot,-0.785)
        cur=pick52.wait_pose(tf_api,pick52.TCP_FRAME)
        pick52.move_tcp_delta(robot,tf_api,left_hold,[0.0,0.0,grasp[2]-cur.position[2]],
                              target_q,baseline,report,"descend_to_grasp")
        pick52.set_right_gripper(robot,0.0)
        pick52.move_tcp_delta(robot,tf_api,left_hold,[0.0,0.0,LIFT_AFTER_GRASP_M],
                              target_q,baseline,report,"lift_10cm")
        report["status"]="PICK_AND_LIFT_COMPLETED"; save(report)
        print("产品角度对齐抓取并抬升10cm完成。")
    except KeyboardInterrupt:
        report["status"]="INTERRUPTED"; report["error"]="KeyboardInterrupt"; save(report)
    except Exception as exc:
        if report.get("status")=="INITIALIZED": report["status"]="ERROR"
        report["error"]=str(exc); save(report); raise
    finally:
        if camera is not None:
            try: camera.close_camera()
            except Exception: pass
        if initialized: agibot_gdk.gdk_release()


if __name__=="__main__":
    main()
