#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
45_yolo_tcp_pick_lift_10cm_safe.py

YOLO识别产品中心 -> 深度/TF得到Base XYZ -> TCP垂直预抓取 ->
打开右夹爪 -> 垂直下探 -> 关闭右夹爪 -> 垂直抬升10cm。

默认 ENABLE_REAL_MOTION=False：只检测、规划和输出JSON，不运动。
真实运行必须手动改为True，并输入 PICK。

重要安全门：
- 检测置信度、深度、工作空间检查
- 使用 gripper_r_center_link 作为TCP
- URDF TCP偏移 [0,0,0.14308]m
- 每阶段读取TF闭环校验
- 朝下姿态未达到时禁止下探
- 力矩异常立即停止
- 不执行放置，仅抓取后抬升10cm
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
REPORT_FILE = "yolo_tcp_pick_lift_10cm_report.json"

MIN_CONFIDENCE = 0.60
DETECTION_SAMPLES = 8
DETECTION_TIMEOUT_S = 20.0
DEPTH_RADIUS = 3
MIN_DEPTH_RAW = 50.0
MAX_DEPTH_RAW = 10000.0
CAMERA_TIMEOUT_MS = 1000.0

LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"
TCP_FRAME = "gripper_r_center_link"
HEAD_FRAME = "head_link3"
TCP_OFFSET_END_M = [0.0, 0.0, 0.14308]

# 垂直抓取：TCP局部+Z指向base_link-Z。
DOWN_QUATERNION_XYZW = [1.0, 0.0, 0.0, 0.0]
PREGRASP_HEIGHT_M = 0.100       # 产品中心上方10cm
GRASP_CLEARANCE_M = 0.015       # TCP最终停在产品中心上方15mm
LIFT_HEIGHT_M = 0.100           # 抓取后抬升10cm

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

END_WORKSPACE_X = (0.20, 1.20)
END_WORKSPACE_Y = (-0.80, 0.80)
END_WORKSPACE_Z = (0.20, 1.50)
TCP_WORKSPACE_X = (0.20, 1.20)
TCP_WORKSPACE_Y = (-0.80, 0.80)
TCP_WORKSPACE_Z = (0.20, 1.50)


@dataclass
class PoseData:
    position: List[float]
    orientation: List[float]


def save_report(report):
    Path(REPORT_FILE).write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def normalize_quaternion(q):
    n = math.sqrt(sum(float(v) ** 2 for v in q))
    if n < 1e-12:
        raise RuntimeError("四元数模长接近0")
    return [float(v) / n for v in q]


def quaternion_angle_error_deg(q1, q2):
    q1, q2 = normalize_quaternion(q1), normalize_quaternion(q2)
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def slerp(q0, q1, alpha):
    q0, q1 = normalize_quaternion(q0), normalize_quaternion(q1)
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0:
        q1, dot = [-v for v in q1], -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return normalize_quaternion([(1-alpha)*a + alpha*b for a,b in zip(q0,q1)])
    theta0 = math.acos(dot)
    theta = theta0 * alpha
    sin0 = math.sin(theta0)
    s0 = math.cos(theta) - dot * math.sin(theta) / sin0
    s1 = math.sin(theta) / sin0
    return normalize_quaternion([s0*a + s1*b for a,b in zip(q0,q1)])


def q_to_r(q):
    x,y,z,w = normalize_quaternion(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=float)


def transform_matrix(t):
    m = np.eye(4)
    m[:3,:3] = q_to_r([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
    m[:3,3] = [t.translation.x,t.translation.y,t.translation.z]
    return m


def distance(a,b):
    return math.sqrt(sum((float(b[i])-float(a[i]))**2 for i in range(3)))


def lerp(a,b,alpha):
    return [(1-alpha)*a[i] + alpha*b[i] for i in range(3)]


def workspace_ok(xyz, limits):
    return all(limits[i][0] < xyz[i] < limits[i][1] for i in range(3))


def read_pose(tf_api, frame):
    t = tf_api.get_tf_from_base_link(frame)
    return PoseData(
        [float(t.translation.x),float(t.translation.y),float(t.translation.z)],
        normalize_quaternion([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]),
    )


def wait_pose(tf_api, frame, timeout=10.0):
    end = time.time()+timeout
    last = None
    while time.time() < end:
        try:
            if tf_api.can_transform("base_link", frame):
                return read_pose(tf_api, frame)
        except Exception as exc:
            last = exc
        time.sleep(0.2)
    raise RuntimeError(f"等待TF超时: {frame}; {last}")


def end_from_tcp(tcp_xyz, q):
    offset = q_to_r(q).dot(np.array(TCP_OFFSET_END_M, float))
    return (np.array(tcp_xyz,float)-offset).tolist()


def safe_get_image(camera, camera_type):
    try:
        return camera.get_latest_image(camera_type, CAMERA_TIMEOUT_MS)
    except Exception:
        return None


def decode_color(img):
    return cv2.imdecode(np.frombuffer(img.data,np.uint8),cv2.IMREAD_COLOR)


def decode_depth(img):
    try:
        dtype = np.float32 if img.bit_depth == 32 else np.uint16
        return np.frombuffer(img.data,dtype=dtype).reshape((img.height,img.width))
    except Exception:
        return None


def median_depth(depth,u,v):
    h,w = depth.shape[:2]
    p = depth[max(0,v-DEPTH_RADIUS):min(h,v+DEPTH_RADIUS+1),
              max(0,u-DEPTH_RADIUS):min(w,u+DEPTH_RADIUS+1)].astype(float)
    valid = p[np.isfinite(p)]
    valid = valid[(valid>MIN_DEPTH_RAW)&(valid<MAX_DEPTH_RAW)]
    return None if valid.size == 0 else float(np.median(valid))


def detect_product(model, camera, tf_api):
    intrinsic = camera.get_camera_intrinsic(agibot_gdk.CameraType.kHeadDepth)
    fx,fy,cx0,cy0 = map(float,list(intrinsic.intrinsic)[:4])
    t_base_head = transform_matrix(tf_api.get_tf_from_base_link(HEAD_FRAME))
    t_head_camera = transform_matrix(
        tf_api.get_tf_from_sensor(agibot_gdk.SensorExtrinsicType.kHeadRGBDToHeadLink3)
    )
    t_base_camera = t_base_head @ t_head_camera

    samples=[]
    deadline=time.time()+DETECTION_TIMEOUT_S
    while time.time()<deadline and len(samples)<DETECTION_SAMPLES:
        c=safe_get_image(camera,agibot_gdk.CameraType.kHeadColor)
        d=safe_get_image(camera,agibot_gdk.CameraType.kHeadDepth)
        if c is None or d is None:
            continue
        color,depth=decode_color(c),decode_depth(d)
        if color is None or depth is None:
            continue
        results=model.predict(color,conf=MIN_CONFIDENCE,verbose=False)
        if not len(results) or results[0].boxes is None or not len(results[0].boxes):
            continue
        box=max(results[0].boxes,key=lambda b:float(b.conf[0]))
        conf=float(box.conf[0])
        x1,y1,x2,y2=map(int,box.xyxy[0].tolist())
        px,py=int((x1+x2)/2),int((y1+y2)/2)
        dh,dw=depth.shape[:2]
        du=max(0,min(dw-1,int(round(px*dw/color.shape[1]))))
        dv=max(0,min(dh-1,int(round(py*dh/color.shape[0]))))
        raw=median_depth(depth,du,dv)
        if raw is None:
            continue
        z=raw/1000.0 if raw>20 else raw
        cam=np.array([(du-cx0)*z/fx,(dv-cy0)*z/fy,z,1.0])
        base=(t_base_camera@cam)[:3]
        samples.append((conf,base.tolist(),[px,py],z))
        time.sleep(0.1)

    if len(samples)<max(3,DETECTION_SAMPLES//2):
        raise RuntimeError(f"有效YOLO三维样本不足: {len(samples)}")
    xyz=np.median(np.array([s[1] for s in samples]),axis=0).tolist()
    return {
        "confidence":float(np.median([s[0] for s in samples])),
        "base_xyz":xyz,
        "center_pixel":samples[-1][2],
        "depth_m":float(np.median([s[3] for s in samples])),
        "sample_count":len(samples),
    }


def get_torques(robot)->Dict[str,float]:
    return {s["name"]:float(s["effort"]) for s in robot.get_joint_states()["states"]}


def torque_baseline(robot):
    vals=[]
    print("建立力矩基线...")
    for i in range(BASELINE_SAMPLES):
        vals.append(get_torques(robot)); time.sleep(0.05)
    names=set().union(*(v.keys() for v in vals))
    return {n:sum(v.get(n,0.0) for v in vals)/len(vals) for n in names}


def verify_torque(robot,baseline,report):
    cur=get_torques(robot); bad=[]
    for n,v in cur.items():
        if n in baseline and abs(v-baseline[n])>TORQUE_DELTA_LIMIT_NM:
            bad.append({"joint":n,"delta":abs(v-baseline[n]),"current":v,"baseline":baseline[n]})
    if bad:
        report["status"]="STOPPED_TORQUE"; report["torque_abnormal"]=bad[:10]
        save_report(report); raise RuntimeError("力矩安全检查触发")


def set_pose(robot,left,right):
    req=agibot_gdk.EndEffectorPose(); req.life_time=LIFE_TIME
    req.group=agibot_gdk.EndEffectorControlGroup.kBothArms
    lp,lq=req.left_end_effector_pose.position,req.left_end_effector_pose.orientation
    rp,rq=req.right_end_effector_pose.position,req.right_end_effector_pose.orientation
    lp.x,lp.y,lp.z=left.position; lq.x,lq.y,lq.z,lq.w=left.orientation
    rp.x,rp.y,rp.z=right.position; rq.x,rq.y,rq.z,rq.w=right.orientation
    result=robot.end_effector_pose_control(req)
    if result!=0: raise RuntimeError(f"end_effector_pose_control失败:{result}")


def move_pose(robot,left,start,target,baseline,report,label):
    d=distance(start.position,target.position)
    a=quaternion_angle_error_deg(start.orientation,target.orientation)
    steps=max(2,int(math.ceil(max(d/TRANSLATION_STEP_M,a/ORIENTATION_STEP_DEG))))
    print(f"[{label}] 距离={d:.4f}m 角度={a:.2f}deg 步数={steps}")
    for i in range(1,steps+1):
        verify_torque(robot,baseline,report)
        alpha=i/steps
        cmd=PoseData(lerp(start.position,target.position,alpha),slerp(start.orientation,target.orientation,alpha))
        set_pose(robot,left,cmd); report["motion_command_sent"]=True; time.sleep(DT)
    for _ in range(max(1,int(HOLD_SECONDS*RATE_HZ))):
        set_pose(robot,left,target); time.sleep(DT)


def require_reached(tf_api,frame,target,label,report,pos_tol=POSITION_TOLERANCE_M,ori_tol=ORIENTATION_TOLERANCE_DEG):
    time.sleep(TF_SETTLE_SECONDS); actual=wait_pose(tf_api,frame)
    pe=distance(actual.position,target.position)
    oe=quaternion_angle_error_deg(actual.orientation,target.orientation)
    report.setdefault("stages",[]).append({"stage":label,"actual":actual.__dict__,"position_error_m":pe,"orientation_error_deg":oe})
    save_report(report); print(f"[{label}验证] 位置误差={pe:.4f}m 姿态误差={oe:.2f}deg")
    if pe>pos_tol or oe>ori_tol:
        report["status"]=f"STOPPED_{label.upper()}_NOT_REACHED"; save_report(report)
        raise RuntimeError(f"{label}未到位，停止后续动作")
    return actual


def set_grippers(robot,position):
    states=agibot_gdk.JointStates(); states.group="dual_tool"; states.target_type="omnipicker"
    l=agibot_gdk.JointState(); r=agibot_gdk.JointState(); l.position=position; r.position=position
    states.states=[l,r]; states.nums=2
    result=robot.move_ee_pos(states)
    if result!=0: raise RuntimeError(f"夹爪控制失败:{result}")
    time.sleep(0.6)


def main():
    report={"program":"45_yolo_tcp_pick_lift_10cm_safe.py","enable_real_motion":ENABLE_REAL_MOTION,
            "motion_command_sent":False,"gripper_command_sent":False,"status":"INITIALIZED","stages":[]}
    initialized=False
    try:
        if not Path(MODEL_PATH).exists(): raise RuntimeError(f"模型不存在:{MODEL_PATH}")
        if agibot_gdk.gdk_init()!=agibot_gdk.GDKRes.kSuccess: raise RuntimeError("GDK初始化失败")
        initialized=True
        robot=agibot_gdk.Robot(); camera=agibot_gdk.Camera(); tf_api=agibot_gdk.TF(); time.sleep(3)
        model=YOLO(MODEL_PATH)
        left=wait_pose(tf_api,LEFT_FRAME); end0=wait_pose(tf_api,RIGHT_FRAME); tcp0=wait_pose(tf_api,TCP_FRAME)
        detection=detect_product(model,camera,tf_api); product=detection["base_xyz"]
        down=normalize_quaternion(DOWN_QUATERNION_XYZW)

        tcp_pre=[product[0],product[1],product[2]+PREGRASP_HEIGHT_M]
        tcp_grasp=[product[0],product[1],product[2]+GRASP_CLEARANCE_M]
        tcp_lift=[product[0],product[1],product[2]+GRASP_CLEARANCE_M+LIFT_HEIGHT_M]
        end_pre=end_from_tcp(tcp_pre,down); end_grasp=end_from_tcp(tcp_grasp,down); end_lift=end_from_tcp(tcp_lift,down)
        q_change=quaternion_angle_error_deg(end0.orientation,down)

        report.update({"detection":detection,"product_base_xyz":product,
                       "tcp_targets":{"pregrasp":tcp_pre,"grasp":tcp_grasp,"lift_10cm":tcp_lift},
                       "end_targets":{"pregrasp":end_pre,"grasp":end_grasp,"lift_10cm":end_lift,
                                      "quaternion_xyzw":down},
                       "initial_end":end0.__dict__,"initial_tcp":tcp0.__dict__,
                       "orientation_change_deg":q_change})
        save_report(report)

        limits=(TCP_WORKSPACE_X,TCP_WORKSPACE_Y,TCP_WORKSPACE_Z)
        end_limits=(END_WORKSPACE_X,END_WORKSPACE_Y,END_WORKSPACE_Z)
        if not all(workspace_ok(p,limits) for p in (tcp_pre,tcp_grasp,tcp_lift)):
            raise RuntimeError("TCP目标超出工作空间")
        if not all(workspace_ok(p,end_limits) for p in (end_pre,end_grasp,end_lift)):
            raise RuntimeError("End目标超出工作空间")
        if distance(end0.position,end_pre)>MAX_TOTAL_MOVE_M: raise RuntimeError("到PreGrasp距离过大")
        if q_change>MAX_ORIENTATION_CHANGE_DEG: raise RuntimeError("朝下姿态变化过大")

        print("="*76)
        print("45_yolo_tcp_pick_lift_10cm_safe.py")
        print(f"ENABLE_REAL_MOTION={ENABLE_REAL_MOTION}")
        print(f"产品Base XYZ={np.round(product,5).tolist()} conf={detection['confidence']:.3f}")
        print(f"TCP PreGrasp={np.round(tcp_pre,5).tolist()}")
        print(f"TCP Grasp={np.round(tcp_grasp,5).tolist()}")
        print(f"TCP Lift10cm={np.round(tcp_lift,5).tolist()}")
        print(f"朝下姿态变化={q_change:.2f}deg")
        print("="*76)

        if not ENABLE_REAL_MOTION:
            report["status"]="DRY_RUN_PASS"; save_report(report)
            print("DRY RUN通过，未发送运动或夹爪命令。")
            return

        if input("确认急停可用、工作区无人、产品与桌面安全后输入 PICK：").strip()!="PICK":
            report["status"]="CANCELLED_BY_USER"; save_report(report); return

        baseline=torque_baseline(robot)
        # 打开夹爪
        set_grippers(robot,-0.785); report["gripper_command_sent"]=True; save_report(report)
        # 联合位置+姿态移动到TCP预抓取
        pre_target=PoseData(end_pre,down)
        move_pose(robot,left,wait_pose(tf_api,RIGHT_FRAME),pre_target,baseline,report,"pregrasp")
        require_reached(tf_api,RIGHT_FRAME,pre_target,"pregrasp",report)
        tcp_actual=wait_pose(tf_api,TCP_FRAME)
        if distance(tcp_actual.position,tcp_pre)>TCP_TOLERANCE_M:
            raise RuntimeError("TCP PreGrasp未到位")
        # 垂直下探
        grasp_target=PoseData(end_grasp,down)
        move_pose(robot,left,wait_pose(tf_api,RIGHT_FRAME),grasp_target,baseline,report,"descend")
        require_reached(tf_api,RIGHT_FRAME,grasp_target,"descend",report)
        # 关闭夹爪
        set_grippers(robot,0.0); report["gripper_command_sent"]=True; save_report(report)
        # 抬升10cm
        lift_target=PoseData(end_lift,down)
        move_pose(robot,left,wait_pose(tf_api,RIGHT_FRAME),lift_target,baseline,report,"lift_10cm")
        require_reached(tf_api,RIGHT_FRAME,lift_target,"lift_10cm",report)
        final_tcp=wait_pose(tf_api,TCP_FRAME)
        report.update({"final_tcp":final_tcp.__dict__,"status":"PICK_AND_LIFT_COMPLETED"})
        save_report(report)
        print("抓取并抬升10cm流程完成。")

    except KeyboardInterrupt:
        report["status"]="INTERRUPTED"; report["error"]="KeyboardInterrupt"; save_report(report)
        print("用户中断，停止后续动作。")
    except Exception as exc:
        if report.get("status")=="INITIALIZED": report["status"]="ERROR"
        report["error"]=str(exc); save_report(report); raise
    finally:
        if initialized:
            try: camera.close_camera()
            except Exception: pass
            agibot_gdk.gdk_release()


if __name__=="__main__":
    main()
