#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
48_joint_waypoint_yolo_pick_safe.py

基于VR示教关节位置的右臂抓取程序。

阶段A：按关节空间依次进入示教路径，最终到PREGRASP。
阶段B：YOLO检查产品中心相对示教产品中心的偏差。
阶段C：仅在偏差很小时，保持PREGRASP姿态进行TCP小范围修正。
阶段D：可选下探、闭合夹爪、抬升。

默认所有真实动作关闭：
  ENABLE_REAL_MOTION = False
  ENABLE_LOCAL_VISION_CORRECTION = False
  ENABLE_DESCEND_GRIP_LIFT = False

重要：当前尚未示教GRASP和LIFT关节姿态，因此默认禁止下探抓取。
第一轮只测试 HOME附近 -> Waypoint链 -> PREGRASP。
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

# ==================== 功能开关 ====================
ENABLE_REAL_MOTION = True
ENABLE_LOCAL_VISION_CORRECTION = False
ENABLE_DESCEND_GRIP_LIFT = False

MODEL_PATH = "runs/detect/runs/product_detector/weights/best.pt"
REPORT_FILE = "joint_waypoint_yolo_pick_report.json"

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

# ==================== VR示教关节姿态 ====================
# HOME仅用于识别起始状态，不强制先回HOME。
HOME = [-1.57079643, -1.57079608, 1.57079585, -1.57079645,
        0.00000024, 0.00000000, 0.00000036]

# 按用户实际记录顺序保存。首次真实测试必须逐段观察。
WAYPOINT_1 = [-1.91304851, 0.20393955, 1.83169374, -1.84651811,
              0.29203423, -0.28742294, 0.91643847]
WAYPOINT_2 = [-2.34999969, 0.99999998, 2.39123433, -1.79048179,
              -1.50846242, -0.95681261, -0.11304288]
WAYPOINT_3 = [-2.35000014, 0.99999983, 2.27358603, -1.75903363,
              0.97554659, 0.01765348, 0.59585902]
PREGRASP = [-0.26450533, -1.08659669, 0.16999971, -1.47974414,
            -0.31172467, 0.44504713, 1.37577128]

POSE_CHAIN = [
    ("waypoint_1", WAYPOINT_1),
    ("waypoint_2", WAYPOINT_2),
    ("waypoint_3", WAYPOINT_3),
    ("pregrasp", PREGRASP),
]

# 记录PREGRASP时的TCP，用于TF验收。
RECORDED_PREGRASP_TCP = [0.66409, -0.09406, 0.82812]
RECORDED_PREGRASP_Q = [0.492028, 0.861327, 0.092413, 0.086507]

# 关节规划速度。move_arm_joint会规划到目标位置后返回。
ARM_SPEED_RAD_S = 0.12
JOINT_TARGET_TOLERANCE_RAD = 0.05
TCP_PREGRASP_TOLERANCE_M = 0.035
TCP_ORIENTATION_TOLERANCE_DEG = 8.0
TORQUE_DELTA_LIMIT_NM = 30.0
BASELINE_SAMPLES = 30

# 局部视觉修正与抓取限制。
MIN_CONFIDENCE = 0.60
MAX_LOCAL_XY_CORRECTION_M = 0.030
MAX_LOCAL_Z_CORRECTION_M = 0.020
PREGRASP_ABOVE_PRODUCT_M = 0.100
GRASP_ABOVE_PRODUCT_M = 0.020
LIFT_M = 0.100
TCP_MIN_Z_M = 0.72

RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
LIFE_TIME = 0.02
CART_STEP_M = 0.001
TF_SETTLE_S = 0.7


@dataclass
class PoseData:
    position: List[float]
    orientation: List[float]


def save(report):
    Path(REPORT_FILE).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")


def normalize_q(q):
    n = math.sqrt(sum(float(v) ** 2 for v in q))
    if n < 1e-12:
        raise RuntimeError("四元数模长接近0")
    return [float(v)/n for v in q]


def q_error_deg(q1, q2):
    q1, q2 = normalize_q(q1), normalize_q(q2)
    dot = abs(sum(a*b for a,b in zip(q1,q2)))
    return math.degrees(2*math.acos(max(-1.0,min(1.0,dot))))


def distance(a,b):
    return math.sqrt(sum((float(b[i])-float(a[i]))**2 for i in range(3)))


def lerp(a,b,t):
    return [(1-t)*a[i]+t*b[i] for i in range(len(a))]


def read_pose(tf_api, frame):
    t = tf_api.get_tf_from_base_link(frame)
    return PoseData(
        [float(t.translation.x),float(t.translation.y),float(t.translation.z)],
        normalize_q([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]),
    )


def wait_pose(tf_api, frame, timeout=10.0):
    deadline=time.time()+timeout
    while time.time()<deadline:
        if tf_api.can_transform("base_link",frame):
            return read_pose(tf_api,frame)
        time.sleep(0.2)
    raise RuntimeError(f"等待TF超时: {frame}")


def joint_map(robot):
    return {s["name"]:s for s in robot.get_joint_states()["states"]}


def arm_positions(robot,names):
    states=joint_map(robot)
    return [float(states[name]["motor_position"]) for name in names]


def joint_max_error(robot,target_right):
    actual=arm_positions(robot,RIGHT_JOINTS)
    errors=[abs(a-b) for a,b in zip(actual,target_right)]
    return max(errors),actual,errors


def get_torques(robot)->Dict[str,float]:
    return {s["name"]:float(s["effort"]) for s in robot.get_joint_states()["states"]}


def torque_baseline(robot):
    samples=[]
    for _ in range(BASELINE_SAMPLES):
        samples.append(get_torques(robot)); time.sleep(0.05)
    names=set().union(*(s.keys() for s in samples))
    return {n:sum(s.get(n,0.0) for s in samples)/len(samples) for n in names}


def verify_torque(robot,baseline,report):
    current=get_torques(robot); bad=[]
    for name,value in current.items():
        if name in baseline and abs(value-baseline[name])>TORQUE_DELTA_LIMIT_NM:
            bad.append({"joint":name,"delta":abs(value-baseline[name])})
    if bad:
        report["status"]="STOPPED_TORQUE"
        report["torque_abnormal"]=bad[:10]
        save(report)
        raise RuntimeError("力矩安全检查触发")


def move_right_joint_waypoint(robot,target_right,label,baseline,report):
    verify_torque(robot,baseline,report)
    current_left=arm_positions(robot,LEFT_JOINTS)
    positions=current_left+target_right
    velocities=[ARM_SPEED_RAD_S]*14
    print(f"[{label}] move_arm_joint, 右臂目标={['%.3f'%v for v in target_right]}")
    result=robot.move_arm_joint(positions,velocities,1)
    if result!=0:
        raise RuntimeError(f"{label} move_arm_joint返回失败: {result}")
    time.sleep(TF_SETTLE_S)
    max_error,actual,errors=joint_max_error(robot,target_right)
    record={"stage":label,"target":target_right,"actual":actual,
            "joint_errors":errors,"max_joint_error_rad":max_error}
    report["joint_stages"].append(record); save(report)
    print(f"[{label}验证] 最大关节误差={max_error:.4f}rad")
    if max_error>JOINT_TARGET_TOLERANCE_RAD:
        report["status"]=f"STOPPED_{label.upper()}_JOINT_ERROR"; save(report)
        raise RuntimeError(f"{label}关节未到位，停止")


def set_end_pose(robot,left,right):
    req=agibot_gdk.EndEffectorPose(); req.life_time=LIFE_TIME
    req.group=agibot_gdk.EndEffectorControlGroup.kBothArms
    lp,lq=req.left_end_effector_pose.position,req.left_end_effector_pose.orientation
    rp,rq=req.right_end_effector_pose.position,req.right_end_effector_pose.orientation
    lp.x,lp.y,lp.z=left.position; lq.x,lq.y,lq.z,lq.w=left.orientation
    rp.x,rp.y,rp.z=right.position; rq.x,rq.y,rq.z,rq.w=right.orientation
    result=robot.end_effector_pose_control(req)
    if result!=0: raise RuntimeError(f"end_effector_pose_control失败:{result}")


def cartesian_move(robot,left,start,target,baseline,report,label):
    d=distance(start.position,target.position)
    steps=max(2,int(math.ceil(d/CART_STEP_M)))
    for i in range(1,steps+1):
        verify_torque(robot,baseline,report)
        p=lerp(start.position,target.position,i/steps)
        if p[2]<TCP_MIN_Z_M:
            raise RuntimeError(f"{label}目标高度低于安全门限")
        set_end_pose(robot,left,PoseData(p,target.orientation))
        time.sleep(DT)
    time.sleep(TF_SETTLE_S)


def set_gripper(robot,position):
    req=agibot_gdk.JointStates(); req.group="dual_tool"; req.target_type="omnipicker"
    l=agibot_gdk.JointState(); r=agibot_gdk.JointState()
    l.position=position; r.position=position
    req.states=[l,r]; req.nums=2
    result=robot.move_ee_pos(req)
    if result!=0: raise RuntimeError(f"夹爪控制失败:{result}")
    time.sleep(0.6)


def main():
    report={"program":"48_joint_waypoint_yolo_pick_safe.py",
            "enable_real_motion":ENABLE_REAL_MOTION,
            "enable_local_vision_correction":ENABLE_LOCAL_VISION_CORRECTION,
            "enable_descend_grip_lift":ENABLE_DESCEND_GRIP_LIFT,
            "joint_stages":[],"status":"INITIALIZED"}
    initialized=False
    try:
        if agibot_gdk.gdk_init()!=agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("GDK初始化失败")
        initialized=True
        robot=agibot_gdk.Robot(); tf_api=agibot_gdk.TF(); time.sleep(2)
        initial_right=arm_positions(robot,RIGHT_JOINTS)
        initial_tcp=wait_pose(tf_api,TCP_FRAME)
        report["initial_right_joints"]=initial_right
        report["initial_tcp"]=initial_tcp.__dict__
        report["pose_chain"]=[{"name":n,"right_joints":p} for n,p in POSE_CHAIN]
        save(report)

        print("="*78)
        print("48_joint_waypoint_yolo_pick_safe.py")
        print(f"ENABLE_REAL_MOTION={ENABLE_REAL_MOTION}")
        print(f"ENABLE_LOCAL_VISION_CORRECTION={ENABLE_LOCAL_VISION_CORRECTION}")
        print(f"ENABLE_DESCEND_GRIP_LIFT={ENABLE_DESCEND_GRIP_LIFT}")
        print("关节路径: WAYPOINT_1 -> WAYPOINT_2 -> WAYPOINT_3 -> PREGRASP")
        print("第一轮只验证关节路径，不下探、不闭合夹爪。")
        print("="*78)

        if not ENABLE_REAL_MOTION:
            report["status"]="DRY_RUN_PASS"; save(report)
            print("DRY RUN通过，未发送运动命令。")
            return

        if input("确认急停可用、路径无障碍后输入 CHAIN：").strip()!="CHAIN":
            report["status"]="CANCELLED_BY_USER"; save(report); return

        baseline=torque_baseline(robot)
        for label,target in POSE_CHAIN:
            move_right_joint_waypoint(robot,target,label,baseline,report)

        tcp=wait_pose(tf_api,TCP_FRAME)
        tcp_error=distance(tcp.position,RECORDED_PREGRASP_TCP)
        q_error=q_error_deg(tcp.orientation,RECORDED_PREGRASP_Q)
        report["pregrasp_validation"]={"actual":tcp.__dict__,"tcp_error_m":tcp_error,
                                       "orientation_error_deg":q_error}
        print(f"[PREGRASP TF验证] TCP误差={tcp_error:.4f}m 姿态误差={q_error:.2f}deg")
        if tcp_error>TCP_PREGRASP_TOLERANCE_M or q_error>TCP_ORIENTATION_TOLERANCE_DEG:
            report["status"]="STOPPED_PREGRASP_TF_ERROR"; save(report)
            raise RuntimeError("PREGRASP TF未通过，禁止抓取")

        # 尚未示教GRASP/LIFT，因此默认到此停止。
        if not ENABLE_LOCAL_VISION_CORRECTION:
            report["status"]="PREGRASP_REACHED"
            save(report)
            print("PREGRASP已到达。局部视觉修正未启用，安全停止。")
            return

        if not ENABLE_DESCEND_GRIP_LIFT:
            report["status"]="PREGRASP_REACHED_GRASP_DISABLED"
            save(report)
            print("抓取阶段未启用，安全停止。")
            return

        # 抓取阶段必须在后续获得GRASP/LIFT示教点或验证局部下探后再启用。
        raise RuntimeError("未配置经过示教验证的GRASP与LIFT点，拒绝执行抓取")

    except KeyboardInterrupt:
        report["status"]="INTERRUPTED"; report["error"]="KeyboardInterrupt"; save(report)
    except Exception as exc:
        if report.get("status")=="INITIALIZED": report["status"]="ERROR"
        report["error"]=str(exc); save(report); raise
    finally:
        if initialized: agibot_gdk.gdk_release()


if __name__=="__main__":
    main()
