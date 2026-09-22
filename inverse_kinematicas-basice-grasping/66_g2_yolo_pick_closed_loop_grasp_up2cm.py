#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
66_g2_yolo_pick_closed_loop_grasp_up2cm.py

独立完整单文件，不依赖52/53/58/62等本地程序。

流程：
HOME -> WAYPOINT_1（一次到点） -> 头部idx13 +20度
-> WAYPOINT_2（一次到点） -> YOLO中心/长轴/深度/TF
-> 高位分段角度对齐 -> 高位XY闭环对准
-> 下降至PreGrasp -> 打开右夹爪 -> 下降至Grasp
-> 闭合右夹爪 -> 抬升10cm。

本版重点修复：
1. 高位XY首次误差25~50mm时，最多执行2次剩余误差闭环修正。
2. 正常通过门限仍为25mm，不简单放宽。
3. 误差超过50mm立即停止，禁止自动追赶。
4. 夹爪扁长指尖使用局部Y轴。
5. Grasp TCP位于产品视觉表面上方25mm，即相对61版提高30mm。
6. 自动准备OpenCV Qt字体目录，并实时显示/保存YOLO画面。

默认所有真实动作关闭。请按阶段逐项启用。
"""

import os

# OpenCV Qt会查找该固定目录。在import cv2前创建字体链接。
SYSTEM_FONT_DIR = "/usr/share/fonts/truetype/dejavu"
CV2_QT_FONT_DIR = os.path.expanduser(
    "~/.local/lib/python3.10/site-packages/cv2/qt/fonts"
)

def prepare_qt_fonts():
    result = {"created": False, "linked": 0, "error": None}
    try:
        os.makedirs(CV2_QT_FONT_DIR, exist_ok=True)
        if os.path.isdir(SYSTEM_FONT_DIR):
            for name in os.listdir(SYSTEM_FONT_DIR):
                if not name.lower().endswith((".ttf", ".otf")):
                    continue
                src = os.path.join(SYSTEM_FONT_DIR, name)
                dst = os.path.join(CV2_QT_FONT_DIR, name)
                if not os.path.lexists(dst):
                    os.symlink(src, dst)
                result["linked"] += 1
        os.environ["QT_QPA_FONTDIR"] = CV2_QT_FONT_DIR
        result["created"] = True
    except Exception as exc:
        result["error"] = str(exc)
    return result

QT_FONT_RESULT = prepare_qt_fonts()

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

# -----------------------------------------------------------------------------
# 阶段开关
# -----------------------------------------------------------------------------
ENABLE_REAL_MOTION = True
ENABLE_ANGLE_ALIGNMENT = True
ENABLE_XY_AND_PREGRASP = True
ENABLE_GRIPPER_AND_PICK = True
REQUIRE_CONFIRMATION = True

MODEL_PATH = "runs/detect/runs/product_detector/weights/best.pt"
REPORT_FILE = "g2_yolo_pick_closed_loop_grasp_up2cm_report.json"
LIVE_IMAGE_FILE = "g2_yolo_grasp_up2cm_live.jpg"
ANGLE_IMAGE_FILE = "g2_yolo_grasp_up2cm_angle.jpg"
WINDOW_NAME = "G2 YOLO Live"
SHOW_YOLO_WINDOW = True

# -----------------------------------------------------------------------------
# 坐标系和关节
# -----------------------------------------------------------------------------
LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"
TCP_FRAME = "gripper_r_center_link"
HEAD_FRAME = "head_link3"
RIGHT_JOINTS = [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)]
HEAD_JOINTS = ["idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3"]

HOME = [-1.57079643, -1.57079608, 1.57079585, -1.57079645,
        0.00000024, 0.00000000, 0.00000036]
WAYPOINT_1 = [-1.91304851, 0.20393955, 1.83169374, -1.84651811,
              0.29203423, -0.28742294, 0.91643847]
WAYPOINT_2 = [-2.34999915, 1.00000018, 2.39123385, -1.79048167,
              -1.50846278, -0.95681237, -0.11304288]
RECORDED_TCP = {
    "home": [0.73185, -0.24350, 0.81644],
    "waypoint_1": [0.58639, -0.34289, 0.93608],
    "waypoint_2": [0.65699, -0.16556, 0.98419],
}

# -----------------------------------------------------------------------------
# 运动参数
# -----------------------------------------------------------------------------
DIRECT_ARM_SPEED_RAD_S = 0.20
JOINT_TOLERANCE_RAD = 0.06
KEYPOINT_TCP_TOLERANCE_M = 0.050
START_POSE_TOLERANCE_RAD = 0.15
HEAD_TARGET_DEG = 20.0
HEAD_SPEED_RAD_S = 0.20
HEAD_TOLERANCE_DEG = 1.5
SETTLE_S = 1.0

JOINT_LIMITS = [
    (-3.071796, 3.071796), (-2.059505, 2.059505),
    (-3.071796, 3.071796), (-2.495838, 1.012308),
    (-3.071796, 3.071796), (-1.012308, 1.012308),
    (-1.535907, 1.535907),
]

# -----------------------------------------------------------------------------
# YOLO/深度/角度
# -----------------------------------------------------------------------------
MIN_CONFIDENCE = 0.2
DETECTION_SAMPLES = 8
MIN_POSITION_SAMPLES = 4
MIN_ANGLE_SAMPLES = 3
DETECTION_TIMEOUT_S = 25.0
CAMERA_TIMEOUT_MS = 1000.0
DEPTH_RADIUS = 4
MIN_DEPTH_RAW = 50.0
MAX_DEPTH_RAW = 10000.0
MIN_ASPECT_RATIO = 1.20
MIN_CONTOUR_AREA_RATIO = 0.025
ROI_PADDING_RATIO = 0.08

# -----------------------------------------------------------------------------
# 抓取参数
# -----------------------------------------------------------------------------
PREGRASP_ABOVE_PRODUCT_M = 0.120
GRASP_ABOVE_PRODUCT_M = 0.045  # 相对63版向上提高20mm: 25mm -> 45mm  # 相对61版提高30mm
LIFT_AFTER_GRASP_M = 0.100
MAX_XY_FROM_WP2_M = 0.250
MAX_PREGRASP_DESCENT_M = 0.250
MIN_TCP_Z_M = 0.690
GRIPPER_LONG_AXIS_LOCAL = "y"
GRIPPER_YAW_OFFSET_DEG = 0.0
MAX_YAW_CORRECTION_DEG = 60.0
TCP_OFFSET_END_M = np.array([0.0, 0.0, 0.14308], dtype=float)

# -----------------------------------------------------------------------------
# 笛卡尔和闭环修正
# -----------------------------------------------------------------------------
RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
LIFE_TIME = 0.02
CART_STEP_M = 0.0015
TCP_TOLERANCE_M = 0.025
ORIENTATION_TOLERANCE_DEG = 6.0
HARD_TCP_ERROR_LIMIT_M = 0.050
MAX_CORRECTION_ATTEMPTS = 2
CORRECTION_STEP_M = 0.001
MAX_YAW_PER_STAGE_DEG = 7.5
ROTATION_STEP_DEG = 0.75
ROTATION_FINAL_HOLD_S = 0.45
MAX_ROTATION_STAGE_DRIFT_M = 0.025

# -----------------------------------------------------------------------------
# 力矩
# -----------------------------------------------------------------------------
BASELINE_SAMPLES = 30
TORQUE_DELTA_LIMIT_NM = 30.0


@dataclass
class PoseData:
    position: List[float]
    orientation: List[float]


def save(report):
    Path(REPORT_FILE).write_text(json.dumps(report, ensure_ascii=False, indent=2),
                                 encoding="utf-8")


def norm_q(q):
    arr = np.asarray(q, dtype=float)
    n = float(np.linalg.norm(arr))
    if n < 1e-12:
        raise RuntimeError("四元数无效")
    return (arr / n).tolist()


def q_mul(q1, q2):
    x1,y1,z1,w1 = norm_q(q1); x2,y2,z2,w2 = norm_q(q2)
    return norm_q([
        w1*x2+x1*w2+y1*z2-z1*y2,
        w1*y2-x1*z2+y1*w2+z1*x2,
        w1*z2+x1*y2-y1*x2+z1*w2,
        w1*w2-x1*x2-y1*y2-z1*z2,
    ])


def q_yaw(yaw):
    return [0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0)]


def slerp(q0, q1, a):
    q0=np.asarray(norm_q(q0)); q1=np.asarray(norm_q(q1)); dot=float(q0@q1)
    if dot < 0: q1=-q1; dot=-dot
    dot=max(-1.0,min(1.0,dot))
    if dot > 0.9995: return norm_q((1-a)*q0+a*q1)
    th=math.acos(dot); st=math.sin(th)
    return norm_q(math.sin((1-a)*th)/st*q0 + math.sin(a*th)/st*q1)


def q_error_deg(q1, q2):
    dot=abs(sum(a*b for a,b in zip(norm_q(q1),norm_q(q2))))
    return math.degrees(2*math.acos(max(-1.0,min(1.0,dot))))


def q_to_r(q):
    x,y,z,w=norm_q(q)
    return np.array([
        [1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)],
    ])


def tf_matrix(t):
    m=np.eye(4)
    m[:3,:3]=q_to_r([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
    m[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
    return m


def dist(a,b): return math.sqrt(sum((float(b[i])-float(a[i]))**2 for i in range(3)))
def lerp(a,b,t): return [(1-t)*a[i]+t*b[i] for i in range(3)]


def wrap_axis(angle):
    while angle > math.pi/2: angle -= math.pi
    while angle < -math.pi/2: angle += math.pi
    return angle


def axis_average(values):
    v=np.asarray(values)
    return 0.5*math.atan2(float(np.mean(np.sin(2*v))),float(np.mean(np.cos(2*v))))


def read_pose(tf_api, frame):
    t=tf_api.get_tf_from_base_link(frame)
    return PoseData([float(t.translation.x),float(t.translation.y),float(t.translation.z)],
                    norm_q([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]))


def wait_pose(tf_api, frame, timeout=10.0):
    deadline=time.time()+timeout
    while time.time()<deadline:
        if tf_api.can_transform("base_link",frame): return read_pose(tf_api,frame)
        time.sleep(0.2)
    raise RuntimeError(f"TF超时: {frame}")


def states_map(robot):
    return {s["name"]:s for s in robot.get_joint_states()["states"]}


def get_positions(robot,names):
    states=states_map(robot)
    missing=[n for n in names if n not in states]
    if missing: raise RuntimeError(f"缺少关节: {missing}")
    return [float(states[n]["motor_position"]) for n in names]


def max_joint_error(a,b): return max(abs(x-y) for x,y in zip(a,b))


def identify_start(current):
    candidates=[(max_joint_error(current,HOME),"home",0),
                (max_joint_error(current,WAYPOINT_1),"waypoint_1",1),
                (max_joint_error(current,WAYPOINT_2),"waypoint_2",2)]
    error,name,index=min(candidates)
    if error>START_POSE_TOLERANCE_RAD:
        raise RuntimeError(f"当前姿态未知，最近{name}，误差={error:.3f}rad")
    return name,index,error


def get_torques(robot):
    return {s["name"]:float(s["effort"]) for s in robot.get_joint_states()["states"]}


def torque_baseline(robot):
    print("建立力矩基线，请保持机器人静止...")
    samples=[]
    for _ in range(BASELINE_SAMPLES): samples.append(get_torques(robot)); time.sleep(0.05)
    names=set().union(*(s.keys() for s in samples))
    return {n:sum(s.get(n,0.0) for s in samples)/len(samples) for n in names}


def verify_torque(robot,base,report):
    bad=[]
    for n,v in get_torques(robot).items():
        if n in base and abs(v-base[n])>TORQUE_DELTA_LIMIT_NM:
            bad.append({"joint":n,"delta":abs(v-base[n])})
    if bad:
        report["status"]="STOPPED_TORQUE"; report["torque_abnormal"]=bad[:10]; save(report)
        raise RuntimeError("力矩安全检查触发")


def move_arm_once(robot,tf_api,label,target,base,report):
    verify_torque(robot,base,report)
    for i,(v,lim) in enumerate(zip(target,JOINT_LIMITS)):
        if not lim[0]<=v<=lim[1]: raise RuntimeError(f"{label}关节{i+1}超限")
    print(f"[{label}] 一次到点，速度={DIRECT_ARM_SPEED_RAD_S:.2f}rad/s")
    if robot.move_arm_joint(list(target),[DIRECT_ARM_SPEED_RAD_S]*7,1)!=0:
        raise RuntimeError(f"{label}运动失败")
    time.sleep(SETTLE_S)
    actual=get_positions(robot,RIGHT_JOINTS); je=max_joint_error(actual,target)
    tcp=wait_pose(tf_api,TCP_FRAME); te=dist(tcp.position,RECORDED_TCP[label])
    report["joint_stages"].append({"stage":label,"joint_error":je,"tcp_error":te,
                                   "actual":actual,"tcp":tcp.__dict__}); save(report)
    print(f"[{label}验证] 关节误差={je:.5f}rad, TCP误差={te:.5f}m")
    if je>JOINT_TOLERANCE_RAD or te>KEYPOINT_TCP_TOLERANCE_M:
        raise RuntimeError(f"{label}未到位")


def ensure_head(robot,report):
    before=get_positions(robot,HEAD_JOINTS)
    target=[before[0],before[1],math.radians(HEAD_TARGET_DEG)]
    if abs(math.degrees(before[2])-HEAD_TARGET_DEG)>HEAD_TOLERANCE_DEG:
        print("[WAYPOINT_1已验证] 头部idx13移动到+20度")
        if robot.move_head_joint(target,[HEAD_SPEED_RAD_S]*3)!=0:
            raise RuntimeError("头部运动失败")
        time.sleep(SETTLE_S)
    after=get_positions(robot,HEAD_JOINTS); error=abs(math.degrees(after[2])-20.0)
    report["head"]={"before":before,"after":after,"error_deg":error}; save(report)
    print(f"[头部验证] idx13={math.degrees(after[2]):.3f}deg, 误差={error:.3f}deg")
    if error>HEAD_TOLERANCE_DEG: raise RuntimeError("头部+20度未到位")


def safe_image(camera,kind):
    try: return camera.get_latest_image(kind,CAMERA_TIMEOUT_MS)
    except Exception: return None


def decode_color(img): return cv2.imdecode(np.frombuffer(img.data,np.uint8),cv2.IMREAD_COLOR)


def decode_depth(img):
    try:
        dtype=np.float32 if img.bit_depth==32 else np.uint16
        return np.frombuffer(img.data,dtype=dtype).reshape((img.height,img.width))
    except Exception: return None


def median_depth(depth,u,v):
    h,w=depth.shape[:2]
    p=depth[max(0,v-DEPTH_RADIUS):min(h,v+DEPTH_RADIUS+1),
            max(0,u-DEPTH_RADIUS):min(w,u+DEPTH_RADIUS+1)].astype(float)
    p=p[np.isfinite(p)]; p=p[(p>MIN_DEPTH_RAW)&(p<MAX_DEPTH_RAW)]
    return None if p.size==0 else float(np.median(p))


def show_frame(frame,lines):
    canvas=frame.copy(); y=28
    for line in lines:
        cv2.putText(canvas,line,(12,y),cv2.FONT_HERSHEY_SIMPLEX,0.65,(0,255,255),2)
        y+=27
    cv2.imwrite(LIVE_IMAGE_FILE,canvas)
    if SHOW_YOLO_WINDOW:
        try:
            cv2.imshow(WINDOW_NAME,canvas); cv2.waitKey(20)
        except cv2.error as exc: print(f"[YOLO窗口] {exc}")


def detect_product(model,camera,tf_api,report):
    if SHOW_YOLO_WINDOW:
        try:
            cv2.namedWindow(WINDOW_NAME,cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW_NAME,960,540); cv2.moveWindow(WINDOW_NAME,40,40)
        except cv2.error as exc: print(f"[YOLO窗口创建失败] {exc}")
    intr=camera.get_camera_intrinsic(agibot_gdk.CameraType.kHeadDepth)
    fx,fy,cx,cy=map(float,list(intr.intrinsic)[:4])
    b2c=tf_matrix(tf_api.get_tf_from_base_link(HEAD_FRAME)) @ tf_matrix(
        tf_api.get_tf_from_sensor(agibot_gdk.SensorExtrinsicType.kHeadRGBDToHeadLink3))
    pos=[]; angles=[]; rejects={"no_image":0,"no_detection":0,"no_depth":0,"no_angle":0}
    last=None; deadline=time.time()+DETECTION_TIMEOUT_S
    while time.time()<deadline and len(pos)<DETECTION_SAMPLES:
        co=safe_image(camera,agibot_gdk.CameraType.kHeadColor)
        do=safe_image(camera,agibot_gdk.CameraType.kHeadDepth)
        if co is None or do is None: rejects["no_image"]+=1; continue
        color=decode_color(co); depth=decode_depth(do)
        if color is None or depth is None: rejects["no_image"]+=1; continue
        result=model.predict(color,conf=MIN_CONFIDENCE,verbose=False)[0]
        if result.boxes is None or not len(result.boxes):
            rejects["no_detection"]+=1; show_frame(color,["YOLO: NO DETECTION"]); time.sleep(0.03); continue
        box=max(result.boxes,key=lambda b:float(b.conf[0])); conf=float(box.conf[0])
        x1,y1,x2,y2=map(int,box.xyxy[0].tolist()); h,w=color.shape[:2]
        x1=max(0,x1);y1=max(0,y1);x2=min(w-1,x2);y2=min(h-1,y2)
        px=(x1+x2)//2; py=(y1+y2)//2; dh,dw=depth.shape[:2]
        u=max(0,min(dw-1,int(round(px*dw/w)))); v=max(0,min(dh-1,int(round(py*dh/h))))
        raw=median_depth(depth,u,v)
        annotated=color.copy(); cv2.rectangle(annotated,(x1,y1),(x2,y2),(0,255,0),2)
        cv2.circle(annotated,(px,py),5,(0,255,255),-1)
        if raw is None:
            rejects["no_depth"]+=1; show_frame(annotated,["YOLO DETECTED","DEPTH INVALID"]); continue
        z=raw/1000.0 if raw>20 else raw
        cam=np.array([(u-cx)*z/fx,(v-cy)*z/fy,z,1.0]); base=(b2c@cam)[:3]
        pos.append(base.tolist())

        roi=color[y1:y2+1,x1:x2+1]
        gray=cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY); blur=cv2.GaussianBlur(gray,(5,5),0)
        masks=[]
        edge=cv2.Canny(blur,30,110)
        edge=cv2.morphologyEx(edge,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8),iterations=2)
        masks.append(edge)
        _,otsu=cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        masks += [otsu,cv2.bitwise_not(otsu)]
        best=None; roi_area=max(1,roi.shape[0]*roi.shape[1])
        for mask in masks:
            contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area=cv2.contourArea(c); ratio=area/roi_area
                if ratio<MIN_CONTOUR_AREA_RATIO: continue
                rect=cv2.minAreaRect(c); (_, _),(rw,rh),_=rect
                if min(rw,rh)<3 or max(rw,rh)/min(rw,rh)<MIN_ASPECT_RATIO: continue
                pts=cv2.boxPoints(rect); longest=None
                for i in range(4):
                    a=pts[i];b=pts[(i+1)%4];length=float(np.linalg.norm(b-a))
                    if longest is None or length>longest[0]: longest=(length,a,b)
                score=area*max(rw,rh)/max(1,min(rw,rh))
                if best is None or score>best[0]: best=(score,longest[1],longest[2])
        if best is not None:
            a=best[1]+np.array([x1,y1]); b=best[2]+np.array([x1,y1])
            pbase=[]
            for p in (a,b):
                uu=max(0,min(dw-1,int(round(p[0]*dw/w)))); vv=max(0,min(dh-1,int(round(p[1]*dh/h))))
                cc=np.array([(uu-cx)*z/fx,(vv-cy)*z/fy,z,1.0]); pbase.append((b2c@cc)[:3])
            axis=pbase[1]-pbase[0]
            if np.linalg.norm(axis[:2])>1e-5:
                yaw=math.atan2(float(axis[1]),float(axis[0])); angles.append(yaw)
                cv2.line(annotated,tuple(np.round(a).astype(int)),tuple(np.round(b).astype(int)),(0,0,255),3)
            else: rejects["no_angle"]+=1
        else: rejects["no_angle"]+=1
        last=annotated
        show_frame(annotated,[f"conf={conf:.3f}",f"position={len(pos)}/8",f"angle={len(angles)}"])
        time.sleep(0.1)
    if last is not None: cv2.imwrite(ANGLE_IMAGE_FILE,last)
    report["vision"]={"position_samples":len(pos),"angle_samples":len(angles),"rejects":rejects};save(report)
    if len(pos)<MIN_POSITION_SAMPLES: raise RuntimeError(f"产品位置样本不足: {len(pos)}, {rejects}")
    data={"base_xyz":np.median(np.array(pos),axis=0).tolist(),"angle_valid":False,
          "position_samples":len(pos),"angle_samples":len(angles)}
    if len(angles)>=MIN_ANGLE_SAMPLES:
        yaw=axis_average(angles);data.update({"angle_valid":True,"yaw_rad":yaw,"yaw_deg":math.degrees(yaw)})
    return data


def set_pose(robot,left,right):
    req=agibot_gdk.EndEffectorPose();req.life_time=LIFE_TIME
    req.group=agibot_gdk.EndEffectorControlGroup.kBothArms
    lp,lq=req.left_end_effector_pose.position,req.left_end_effector_pose.orientation
    rp,rq=req.right_end_effector_pose.position,req.right_end_effector_pose.orientation
    lp.x,lp.y,lp.z=left.position;lq.x,lq.y,lq.z,lq.w=left.orientation
    rp.x,rp.y,rp.z=right.position;rq.x,rq.y,rq.z,rq.w=right.orientation
    if robot.end_effector_pose_control(req)!=0: raise RuntimeError("末端控制失败")


def end_from_tcp(tcp,q): return (np.asarray(tcp)-q_to_r(q).dot(TCP_OFFSET_END_M)).tolist()


def command_cart_path(robot,left,start_end,target_end,q,base,report,label,step=CART_STEP_M):
    length=dist(start_end,target_end);n=max(2,int(math.ceil(length/step)))
    for i in range(1,n+1):
        verify_torque(robot,base,report);p=lerp(start_end,target_end,i/n)
        set_pose(robot,left,PoseData(p,q.copy()));time.sleep(DT)


def move_tcp_closed_loop(robot,tf_api,left,delta,q,base,report,label):
    if REQUIRE_CONFIRMATION:
        if input(f"准备执行{label}，delta={[round(v,5) for v in delta]}，输入 NEXT：").strip()!="NEXT":
            raise RuntimeError(f"用户取消{label}")
    start_tcp=wait_pose(tf_api,TCP_FRAME);start_end=wait_pose(tf_api,RIGHT_FRAME)
    target_tcp=[start_tcp.position[i]+delta[i] for i in range(3)]
    if target_tcp[2]<MIN_TCP_Z_M: raise RuntimeError(f"{label}目标过低")
    target_end=[start_end.position[i]+delta[i] for i in range(3)]
    command_cart_path(robot,left,start_end.position,target_end,q,base,report,label)
    attempts=[]
    for attempt in range(MAX_CORRECTION_ATTEMPTS+1):
        time.sleep(SETTLE_S);actual_tcp=wait_pose(tf_api,TCP_FRAME);actual_end=wait_pose(tf_api,RIGHT_FRAME)
        te=dist(actual_tcp.position,target_tcp);qe=q_error_deg(actual_end.orientation,q)
        attempts.append({"attempt":attempt,"tcp_error":te,"orientation_error":qe,
                         "actual_tcp":actual_tcp.__dict__})
        print(f"[{label}闭环{attempt}] TCP误差={te:.4f}m, 姿态误差={qe:.2f}deg")
        if te<=TCP_TOLERANCE_M and qe<=ORIENTATION_TOLERANCE_DEG: break
        if te>HARD_TCP_ERROR_LIMIT_M:
            raise RuntimeError(f"{label} TCP误差{te:.4f}m超过50mm硬限制")
        if attempt>=MAX_CORRECTION_ATTEMPTS:
            raise RuntimeError(f"{label}闭环修正后仍未到位")
        residual=[target_tcp[i]-actual_tcp.position[i] for i in range(3)]
        print(f"[{label}] 自动修正{attempt+1}: {[round(v,5) for v in residual]}")
        corr_target=[actual_end.position[i]+residual[i] for i in range(3)]
        command_cart_path(robot,left,actual_end.position,corr_target,q,base,report,label,CORRECTION_STEP_M)
    report["cartesian_stages"].append({"stage":label,"target_tcp":target_tcp,"attempts":attempts});save(report)
    return wait_pose(tf_api,TCP_FRAME),wait_pose(tf_api,RIGHT_FRAME)


def gripper_yaw(q):
    axis=q_to_r(q)[:,1 if GRIPPER_LONG_AXIS_LOCAL=="y" else 0]
    return math.atan2(float(axis[1]),float(axis[0]))


def rotate_yaw(robot,tf_api,left,delta,base,report):
    stages=max(1,int(math.ceil(abs(math.degrees(delta))/MAX_YAW_PER_STAGE_DEG)))
    if REQUIRE_CONFIRMATION and input(f"准备角度对齐{math.degrees(delta):.1f}deg/{stages}段，输入 NEXT：").strip()!="NEXT":
        raise RuntimeError("用户取消角度对齐")
    initial_end=wait_pose(tf_api,RIGHT_FRAME);initial_q=initial_end.orientation.copy()
    records=[]
    for si in range(1,stages+1):
        target_q=q_mul(q_yaw(delta*si/stages),initial_q)
        start_end=wait_pose(tf_api,RIGHT_FRAME);start_tcp=wait_pose(tf_api,TCP_FRAME)
        n=max(2,int(math.ceil(q_error_deg(start_end.orientation,target_q)/ROTATION_STEP_DEG)))
        for i in range(1,n+1):
            q=slerp(start_end.orientation,target_q,i/n);p=end_from_tcp(start_tcp.position,q)
            set_pose(robot,left,PoseData(p,q));time.sleep(DT)
        hold=max(1,int(ROTATION_FINAL_HOLD_S*RATE_HZ));p=end_from_tcp(start_tcp.position,target_q)
        for _ in range(hold): set_pose(robot,left,PoseData(p,target_q));time.sleep(DT)
        time.sleep(SETTLE_S);at=wait_pose(tf_api,TCP_FRAME);ae=wait_pose(tf_api,RIGHT_FRAME)
        drift=dist(at.position,start_tcp.position);qe=q_error_deg(ae.orientation,target_q)
        records.append({"stage":si,"drift":drift,"orientation_error":qe})
        print(f"[角度对齐 {si}/{stages}] TCP漂移={drift:.4f}m, 姿态误差={qe:.2f}deg")
        if drift>MAX_ROTATION_STAGE_DRIFT_M or qe>ORIENTATION_TOLERANCE_DEG:
            raise RuntimeError(f"角度对齐第{si}段超限")
    report["angle_stages"]=records;save(report)
    return wait_pose(tf_api,TCP_FRAME),wait_pose(tf_api,RIGHT_FRAME)


def set_gripper(robot,value):
    req=agibot_gdk.JointStates();req.group="right_tool";req.target_type="omnipicker"
    j=agibot_gdk.JointState();j.position=value;req.states=[j];req.nums=1
    if robot.move_ee_pos(req)!=0: raise RuntimeError("右夹爪控制失败")
    time.sleep(0.7)


def main():
    report={"program":"66_g2_yolo_pick_closed_loop_grasp_up2cm.py",
            "enable_real_motion":ENABLE_REAL_MOTION,
            "enable_angle_alignment":ENABLE_ANGLE_ALIGNMENT,
            "enable_xy_and_pregrasp":ENABLE_XY_AND_PREGRASP,
            "enable_gripper_and_pick":ENABLE_GRIPPER_AND_PICK,
            "qt_font":QT_FONT_RESULT,
            "grasp_height_calibration": {
                "source_program": "63_g2_yolo_pick_closed_loop_complete.py",
                "previous_grasp_above_product_m": 0.025,
                "new_grasp_above_product_m": 0.045,
                "z_change_m": 0.020,
                "direction": "up",
            },
            "joint_stages":[],"cartesian_stages":[],"status":"INITIALIZED"}
    initialized=False;camera=None
    try:
        if not Path(MODEL_PATH).exists(): raise RuntimeError(f"模型不存在: {MODEL_PATH}")
        if agibot_gdk.gdk_init()!=agibot_gdk.GDKRes.kSuccess: raise RuntimeError("GDK初始化失败")
        initialized=True;robot=agibot_gdk.Robot();tf_api=agibot_gdk.TF();camera=agibot_gdk.Camera()
        model=YOLO(MODEL_PATH);time.sleep(3)
        current=get_positions(robot,RIGHT_JOINTS);name,index,error=identify_start(current)
        left=wait_pose(tf_api,LEFT_FRAME)
        print("="*78)
        print("66_g2_yolo_pick_closed_loop_grasp_up2cm.py")
        print(f"REAL={ENABLE_REAL_MOTION}, ANGLE={ENABLE_ANGLE_ALIGNMENT}, XY={ENABLE_XY_AND_PREGRASP}, PICK={ENABLE_GRIPPER_AND_PICK}")
        print(f"起始姿态={name}, 夹爪长轴={GRIPPER_LONG_AXIS_LOCAL.upper()}, Grasp Z偏移={GRASP_ABOVE_PRODUCT_M:.3f}m")
        print("桌面抓取高度相对63版向上调整=0.020m")
        print("63版偏移=0.025m, 66版偏移=0.045m")
        print(f"XY闭环: 容差25mm, 硬限制50mm, 最多修正{MAX_CORRECTION_ATTEMPTS}次")
        print("="*78)
        if not ENABLE_REAL_MOTION:
            report["status"]="DRY_RUN_PASS";save(report);print("DRY RUN通过");return
        if input("确认急停可用，输入 PICK：").strip()!="PICK": return
        base=torque_baseline(robot)
        if index==0:
            move_arm_once(robot,tf_api,"waypoint_1",WAYPOINT_1,base,report);ensure_head(robot,report)
            move_arm_once(robot,tf_api,"waypoint_2",WAYPOINT_2,base,report)
        elif index==1:
            ensure_head(robot,report);move_arm_once(robot,tf_api,"waypoint_2",WAYPOINT_2,base,report)
        else: ensure_head(robot,report)
        wp2tcp=wait_pose(tf_api,TCP_FRAME);wp2end=wait_pose(tf_api,RIGHT_FRAME)
        det=detect_product(model,camera,tf_api,report);product=det["base_xyz"]
        pre=[product[0],product[1],product[2]+PREGRASP_ABOVE_PRODUCT_M]
        grasp=[product[0],product[1],product[2]+GRASP_ABOVE_PRODUCT_M]
        dx,dy,dz=pre[0]-wp2tcp.position[0],pre[1]-wp2tcp.position[1],pre[2]-wp2tcp.position[2]
        report.update({"detection":det,"product":product,"pregrasp":pre,"grasp":grasp});save(report)
        print(f"产品={np.round(product,5).tolist()}, PreGrasp={np.round(pre,5).tolist()}")
        if math.hypot(dx,dy)>MAX_XY_FROM_WP2_M or -dz>MAX_PREGRASP_DESCENT_M:
            raise RuntimeError("产品超出工作窗口")
        if not det["angle_valid"]: raise RuntimeError("产品角度样本不足")
        yaw=det["yaw_rad"];gy=gripper_yaw(wp2end.orientation)
        delta=wrap_axis(yaw-gy+math.radians(GRIPPER_YAW_OFFSET_DEG))
        print(f"产品长轴={math.degrees(yaw):.1f}deg, 夹爪长轴={math.degrees(gy):.1f}deg, 修正={math.degrees(delta):.1f}deg")
        if abs(math.degrees(delta))>MAX_YAW_CORRECTION_DEG: raise RuntimeError("角度修正过大")
        if not ENABLE_ANGLE_ALIGNMENT:
            report["status"]="ANGLE_PLAN_READY";save(report);return
        aligned_tcp,aligned_end=rotate_yaw(robot,tf_api,left,delta,base,report);q=aligned_end.orientation
        if not ENABLE_XY_AND_PREGRASP:
            report["status"]="ANGLE_ALIGNED";save(report);return
        aligned_tcp,_=move_tcp_closed_loop(robot,tf_api,left,
            [pre[0]-aligned_tcp.position[0],pre[1]-aligned_tcp.position[1],0.0],q,base,report,"align_xy")
        cur=wait_pose(tf_api,TCP_FRAME)
        move_tcp_closed_loop(robot,tf_api,left,[0.0,0.0,pre[2]-cur.position[2]],q,base,report,"descend_pregrasp")
        if not ENABLE_GRIPPER_AND_PICK:
            report["status"]="PREGRASP_REACHED";save(report);return
        set_gripper(robot,-0.785);cur=wait_pose(tf_api,TCP_FRAME)
        move_tcp_closed_loop(robot,tf_api,left,[0.0,0.0,grasp[2]-cur.position[2]],q,base,report,"descend_grasp")
        set_gripper(robot,0.0)
        move_tcp_closed_loop(robot,tf_api,left,[0.0,0.0,LIFT_AFTER_GRASP_M],q,base,report,"lift_10cm")
        report["status"]="PICK_AND_LIFT_COMPLETED";save(report);print("抓取并抬升10cm完成")
    except KeyboardInterrupt:
        report["status"]="INTERRUPTED";save(report)
    except Exception as exc:
        report["status"]="ERROR" if report.get("status")=="INITIALIZED" else report.get("status")
        report["error"]=str(exc);save(report);raise
    finally:
        try: cv2.destroyAllWindows()
        except Exception: pass
        if camera is not None:
            try: camera.close_camera()
            except Exception: pass
        if initialized: agibot_gdk.gdk_release()


if __name__=="__main__":
    main()
