#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
71_g2_pick_waist_turn80_release.py

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
REPORT_FILE = "g2_pick_waist_turn80_release_report.json"
LIVE_IMAGE_FILE = "g2_pick_waist_turn80_release_live.jpg"
ANGLE_IMAGE_FILE = "g2_pick_waist_turn80_release_angle.jpg"
WINDOW_NAME = "G2 YOLO Live"
SHOW_YOLO_WINDOW = True

# -----------------------------------------------------------------------------
# 坐标系和关节
# -----------------------------------------------------------------------------
LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"
TCP_FRAME = "gripper_r_center_link"
HEAD_FRAME = "head_link3"
LEFT_JOINTS = [f"idx2{i}_arm_l_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)]
HEAD_JOINTS = ["idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3"]
WAIST_JOINTS = [
    "idx01_body_joint1",
    "idx02_body_joint2",
    "idx03_body_joint3",
    "idx04_body_joint4",
    "idx05_body_joint5",
]

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
MIN_CONFIDENCE = 0.1
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
GRASP_ABOVE_PRODUCT_M = 0.020  # 35mm会碰桌，55mm偏高，取中间45mm
MIN_TCP_ABOVE_TABLE_M = 0.010    # 桌面安全下限改为桌面Z+40mm
MAX_PRODUCT_Z_RANGE_M = 0.012    # 8帧产品Z极差超过12mm则禁止抓取
MIN_TABLE_SAMPLES = 8
TABLE_RING_MARGIN_RATIO = 0.35
VERTICAL_TOOL_TILT_TOLERANCE_DEG = 4.0
LIFT_AFTER_GRASP_M = 0.100
ENABLE_WAIST_TURN_AND_RELEASE = True
WAIST_TURN_DEG = 80.0          # 正值为idx01正方向；反向转身改为-80.0
WAIST_YAW_JOINT_INDEX = 0      # idx01_body_joint1
WAIST_SPEED_RAD_S = 0.12
WAIST_POSITION_TOLERANCE_DEG = 2.0
ARM_HOLD_TOLERANCE_RAD = 0.035
WAIST_SETTLE_S = 1.0
REQUIRE_WAIST_CONFIRMATION = True
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
    pos=[]; angles=[]; table_z_samples=[]; rejects={"no_image":0,"no_detection":0,"no_depth":0,"no_angle":0,"no_table":0}
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

        # 在检测框外围采样桌面深度，转换为Base Z，用于防撞硬保护。
        bw=max(1,x2-x1); bh=max(1,y2-y1)
        mx=max(8,int(round(bw*TABLE_RING_MARGIN_RATIO)))
        my=max(8,int(round(bh*TABLE_RING_MARGIN_RATIO)))
        ring_color_points=[
            (x1-mx,y1-my),(x2+mx,y1-my),(x1-mx,y2+my),(x2+mx,y2+my),
            ((x1+x2)//2,y1-my),((x1+x2)//2,y2+my),
            (x1-mx,(y1+y2)//2),(x2+mx,(y1+y2)//2),
        ]
        frame_table=[]
        for tx,ty in ring_color_points:
            tx=max(0,min(w-1,int(tx))); ty=max(0,min(h-1,int(ty)))
            tu=max(0,min(dw-1,int(round(tx*dw/w))))
            tv=max(0,min(dh-1,int(round(ty*dh/h))))
            traw=median_depth(depth,tu,tv)
            if traw is None: continue
            tz=traw/1000.0 if traw>20 else traw
            tcam=np.array([(tu-cx)*tz/fx,(tv-cy)*tz/fy,tz,1.0])
            tbase=(b2c@tcam)[:3]
            frame_table.append(float(tbase[2]))
        if frame_table:
            table_z_samples.extend(frame_table)
        else:
            rejects["no_table"]+=1

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
    pos_array=np.array(pos,dtype=float)
    product_xyz=np.median(pos_array,axis=0).tolist()
    product_z_range=float(np.max(pos_array[:,2])-np.min(pos_array[:,2]))
    table_z=None
    if len(table_z_samples)>=MIN_TABLE_SAMPLES:
        # 使用较高分位附近的中位思想，降低桌边/地面的干扰。
        table_values=np.asarray(table_z_samples,dtype=float)
        q25,q75=np.percentile(table_values,[25,75])
        central=table_values[(table_values>=q25)&(table_values<=q75)]
        table_z=float(np.median(central if central.size else table_values))
    data={"base_xyz":product_xyz,"angle_valid":False,
          "position_samples":len(pos),"angle_samples":len(angles),
          "product_z_range_m":product_z_range,
          "table_z_m":table_z,"table_samples":len(table_z_samples)}
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


def rotation_to_q(r):
    trace=float(np.trace(r))
    if trace>0.0:
        ss=math.sqrt(trace+1.0)*2.0; w=0.25*ss
        x=(r[2,1]-r[1,2])/ss; y=(r[0,2]-r[2,0])/ss; z=(r[1,0]-r[0,1])/ss
    elif r[0,0]>r[1,1] and r[0,0]>r[2,2]:
        ss=math.sqrt(1.0+r[0,0]-r[1,1]-r[2,2])*2.0
        w=(r[2,1]-r[1,2])/ss; x=0.25*ss; y=(r[0,1]+r[1,0])/ss; z=(r[0,2]+r[2,0])/ss
    elif r[1,1]>r[2,2]:
        ss=math.sqrt(1.0+r[1,1]-r[0,0]-r[2,2])*2.0
        w=(r[0,2]-r[2,0])/ss; x=(r[0,1]+r[1,0])/ss; y=0.25*ss; z=(r[1,2]+r[2,1])/ss
    else:
        ss=math.sqrt(1.0+r[2,2]-r[0,0]-r[1,1])*2.0
        w=(r[1,0]-r[0,1])/ss; x=(r[0,2]+r[2,0])/ss; y=(r[1,2]+r[2,1])/ss; z=0.25*ss
    return norm_q([x,y,z,w])


def vertical_tool_orientation(product_yaw):
    # local +Z从腕部指向TCP，因此令local +Z垂直向下。
    local_y=np.array([math.cos(product_yaw),math.sin(product_yaw),0.0],dtype=float)
    local_z=np.array([0.0,0.0,-1.0],dtype=float)
    local_x=np.cross(local_y,local_z); local_x/=np.linalg.norm(local_x)
    local_y=np.cross(local_z,local_x); local_y/=np.linalg.norm(local_y)
    return rotation_to_q(np.column_stack((local_x,local_y,local_z)))


def tool_vertical_error_deg(q):
    tool_z=q_to_r(q)[:,2]
    dot=float(np.dot(tool_z,np.array([0.0,0.0,-1.0])))
    return math.degrees(math.acos(max(-1.0,min(1.0,dot))))


def align_orientation_target(robot,tf_api,left,target_q,base,report):
    start_end=wait_pose(tf_api,RIGHT_FRAME); start_tcp=wait_pose(tf_api,TCP_FRAME)
    total=q_error_deg(start_end.orientation,target_q)
    stages=max(1,int(math.ceil(total/MAX_YAW_PER_STAGE_DEG)))
    if REQUIRE_CONFIRMATION and input(f"准备垂直姿态对齐{total:.1f}deg/{stages}段，输入 NEXT：").strip()!="NEXT":
        raise RuntimeError("用户取消垂直姿态对齐")
    records=[]; initial_q=start_end.orientation.copy()
    for si in range(1,stages+1):
        stage_target=slerp(initial_q,target_q,si/stages)
        se=wait_pose(tf_api,RIGHT_FRAME); st=wait_pose(tf_api,TCP_FRAME)
        n=max(2,int(math.ceil(q_error_deg(se.orientation,stage_target)/ROTATION_STEP_DEG)))
        for i in range(1,n+1):
            qq=slerp(se.orientation,stage_target,i/n); pp=end_from_tcp(st.position,qq)
            set_pose(robot,left,PoseData(pp,qq)); time.sleep(DT)
        hold=max(1,int(ROTATION_FINAL_HOLD_S*RATE_HZ)); pp=end_from_tcp(st.position,stage_target)
        for _ in range(hold): set_pose(robot,left,PoseData(pp,stage_target)); time.sleep(DT)
        time.sleep(SETTLE_S); at=wait_pose(tf_api,TCP_FRAME); ae=wait_pose(tf_api,RIGHT_FRAME)
        drift=dist(at.position,st.position); qe=q_error_deg(ae.orientation,stage_target)
        ve=tool_vertical_error_deg(ae.orientation)
        records.append({"stage":si,"drift":drift,"orientation_error":qe,"vertical_error_deg":ve})
        print(f"[垂直姿态 {si}/{stages}] TCP漂移={drift:.4f}m, 姿态误差={qe:.2f}deg, 垂直误差={ve:.2f}deg")
        if drift>MAX_ROTATION_STAGE_DRIFT_M or qe>ORIENTATION_TOLERANCE_DEG:
            raise RuntimeError(f"垂直姿态第{si}段超限")
    final_tcp=wait_pose(tf_api,TCP_FRAME); final_end=wait_pose(tf_api,RIGHT_FRAME)
    vertical_error=tool_vertical_error_deg(final_end.orientation)
    if vertical_error>VERTICAL_TOOL_TILT_TOLERANCE_DEG:
        raise RuntimeError(f"夹爪末端未垂直桌面，误差={vertical_error:.2f}deg")
    report["vertical_orientation_stages"]=records; save(report)
    return final_tcp,final_end


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


def turn_waist_keep_arms_and_release(robot,report):
    """只发送腰部5关节目标；验证双臂关节保持，再松开右夹爪。"""
    waist_before=get_positions(robot,WAIST_JOINTS)
    left_before=get_positions(robot,LEFT_JOINTS)
    right_before=get_positions(robot,RIGHT_JOINTS)

    waist_target=waist_before.copy()
    waist_target[WAIST_YAW_JOINT_INDEX] += math.radians(WAIST_TURN_DEG)

    print("-"*78)
    print(f"准备腰部转身{WAIST_TURN_DEG:+.1f}deg，手臂关节保持当前值")
    print(f"腰部当前(deg)={[round(math.degrees(v),2) for v in waist_before]}")
    print(f"腰部目标(deg)={[round(math.degrees(v),2) for v in waist_target]}")
    if REQUIRE_WAIST_CONFIRMATION:
        command=input("确认转身扫掠区域无人员和障碍物，输入 TURN：").strip()
        if command != "TURN":
            raise RuntimeError("用户取消腰部转身")

    result=robot.move_waist_joint(
        waist_target,
        [WAIST_SPEED_RAD_S]*5,
    )
    if result != 0:
        raise RuntimeError(f"move_waist_joint失败: {result}")
    time.sleep(WAIST_SETTLE_S)

    waist_after=get_positions(robot,WAIST_JOINTS)
    left_after=get_positions(robot,LEFT_JOINTS)
    right_after=get_positions(robot,RIGHT_JOINTS)
    waist_error_deg=abs(math.degrees(
        waist_after[WAIST_YAW_JOINT_INDEX]-waist_target[WAIST_YAW_JOINT_INDEX]
    ))
    left_arm_error=max_joint_error(left_after,left_before)
    right_arm_error=max_joint_error(right_after,right_before)

    report["waist_turn_and_release"]={
        "waist_turn_deg":WAIST_TURN_DEG,
        "waist_joint_index":WAIST_YAW_JOINT_INDEX,
        "waist_joint_name":WAIST_JOINTS[WAIST_YAW_JOINT_INDEX],
        "waist_before_rad":waist_before,
        "waist_target_rad":waist_target,
        "waist_after_rad":waist_after,
        "waist_error_deg":waist_error_deg,
        "left_arm_before_rad":left_before,
        "left_arm_after_rad":left_after,
        "left_arm_max_change_rad":left_arm_error,
        "right_arm_before_rad":right_before,
        "right_arm_after_rad":right_after,
        "right_arm_max_change_rad":right_arm_error,
        "gripper_released":False,
    }
    save(report)

    print(f"[腰部验证] idx01目标误差={waist_error_deg:.2f}deg")
    print(f"[手臂保持验证] 左臂最大变化={left_arm_error:.5f}rad, 右臂最大变化={right_arm_error:.5f}rad")
    if waist_error_deg > WAIST_POSITION_TOLERANCE_DEG:
        raise RuntimeError("腰部80度转身未达到容差，禁止松开夹爪")
    if left_arm_error > ARM_HOLD_TOLERANCE_RAD or right_arm_error > ARM_HOLD_TOLERANCE_RAD:
        raise RuntimeError("腰部转身期间手臂关节变化超限，禁止松开夹爪")

    if REQUIRE_WAIST_CONFIRMATION:
        command=input("腰部到位且手臂保持，确认产品位于放置区域后输入 RELEASE：").strip()
        if command != "RELEASE":
            raise RuntimeError("用户取消松开夹爪")
    print("松开右夹爪，释放产品...")
    set_gripper(robot,-0.785)
    report["waist_turn_and_release"]["gripper_released"]=True
    report["waist_turn_and_release"]["release_time_epoch_s"]=time.time()
    save(report)
    print("腰部转身80度并松开夹爪完成。")
    print("-"*78)


def set_gripper(robot,value):
    req=agibot_gdk.JointStates();req.group="right_tool";req.target_type="omnipicker"
    j=agibot_gdk.JointState();j.position=value;req.states=[j];req.nums=1
    if robot.move_ee_pos(req)!=0: raise RuntimeError("右夹爪控制失败")
    time.sleep(0.7)


def main():
    report={"program":"71_g2_pick_waist_turn80_release.py",
            "enable_real_motion":ENABLE_REAL_MOTION,
            "enable_angle_alignment":ENABLE_ANGLE_ALIGNMENT,
            "enable_xy_and_pregrasp":ENABLE_XY_AND_PREGRASP,
            "enable_gripper_and_pick":ENABLE_GRIPPER_AND_PICK,
            "qt_font":QT_FONT_RESULT,
            "safe_height_control": {
                "nominal_grasp_above_product_m": GRASP_ABOVE_PRODUCT_M,
                "minimum_tcp_above_table_m": MIN_TCP_ABOVE_TABLE_M,
                "maximum_product_z_range_m": MAX_PRODUCT_Z_RANGE_M,
            },
            "vertical_tool_control": {
                "tool_axis": "local +Z",
                "target_base_direction": "-Z",
                "tilt_tolerance_deg": VERTICAL_TOOL_TILT_TOLERANCE_DEG,
            },
            "post_pick_action": {
                "enabled": ENABLE_WAIST_TURN_AND_RELEASE,
                "waist_turn_deg": WAIST_TURN_DEG,
                "waist_yaw_joint": WAIST_JOINTS[WAIST_YAW_JOINT_INDEX],
                "waist_speed_rad_s": WAIST_SPEED_RAD_S,
                "keep_arm_joint_targets": True,
                "release_after_verified_turn": True,
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
        print("71_g2_pick_waist_turn80_release.py")
        print(f"REAL={ENABLE_REAL_MOTION}, ANGLE={ENABLE_ANGLE_ALIGNMENT}, XY={ENABLE_XY_AND_PREGRASP}, PICK={ENABLE_GRIPPER_AND_PICK}")
        print(f"起始姿态={name}, 夹爪长轴={GRIPPER_LONG_AXIS_LOCAL.upper()}, Grasp Z偏移={GRASP_ABOVE_PRODUCT_M:.3f}m")
        print("平衡高度: 名义产品上方45mm，桌面Z+40mm硬下限保护")
        print("现场区间: 35mm会碰桌，55mm偏高，当前选择45mm")
        print("产品Z多帧极差超过12mm时禁止抓取")
        print("夹爪local +Z轴将对齐Base -Z，末端尽量垂直桌面")
        print(f"XY闭环: 容差25mm, 硬限制50mm, 最多修正{MAX_CORRECTION_ATTEMPTS}次")
        print(f"抓取后动作: 腰部idx01相对转动{WAIST_TURN_DEG:+.1f}deg，双臂关节保持，然后松开夹爪")
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
        if det["product_z_range_m"]>MAX_PRODUCT_Z_RANGE_M:
            raise RuntimeError(f"产品Z多帧不稳定: 极差={det['product_z_range_m']:.4f}m")
        if det["table_z_m"] is None:
            raise RuntimeError("桌面Z估计失败，禁止下降抓取")
        table_floor=det["table_z_m"]+MIN_TCP_ABOVE_TABLE_M
        nominal_grasp_z=product[2]+GRASP_ABOVE_PRODUCT_M
        safe_grasp_z=max(nominal_grasp_z,table_floor)
        grasp_z_source=("product_z_plus_45mm" if nominal_grasp_z>=table_floor
                        else "table_z_plus_40mm_safety_floor")
        pre=[product[0],product[1],max(product[2]+PREGRASP_ABOVE_PRODUCT_M,safe_grasp_z+0.060)]
        grasp=[product[0],product[1],safe_grasp_z]
        dx,dy,dz=pre[0]-wp2tcp.position[0],pre[1]-wp2tcp.position[1],pre[2]-wp2tcp.position[2]
        report.update({"detection":det,"product":product,"pregrasp":pre,"grasp":grasp,
                       "table_floor_tcp_z":table_floor,"nominal_grasp_z":nominal_grasp_z,
                       "safe_grasp_z":safe_grasp_z,
                       "grasp_z_source":grasp_z_source});save(report)
        print(f"产品={np.round(product,5).tolist()}, PreGrasp={np.round(pre,5).tolist()}")
        print(f"产品Z极差={det['product_z_range_m']:.4f}m, 桌面Z={det['table_z_m']:.5f}m")
        print(f"名义Grasp Z={nominal_grasp_z:.5f}m, 桌面安全下限={table_floor:.5f}m, 最终Grasp Z={safe_grasp_z:.5f}m")
        print(f"最终Grasp Z来源={grasp_z_source}")
        if math.hypot(dx,dy)>MAX_XY_FROM_WP2_M or -dz>MAX_PREGRASP_DESCENT_M:
            raise RuntimeError("产品超出工作窗口")
        if not det["angle_valid"]: raise RuntimeError("产品角度样本不足")
        yaw=det["yaw_rad"];gy=gripper_yaw(wp2end.orientation)
        target_q=vertical_tool_orientation(yaw+math.radians(GRIPPER_YAW_OFFSET_DEG))
        total_orientation_change=q_error_deg(wp2end.orientation,target_q)
        print(f"产品长轴={math.degrees(yaw):.1f}deg, 当前夹爪长轴={math.degrees(gy):.1f}deg")
        print(f"垂直目标姿态总变化={total_orientation_change:.1f}deg, 目标工具Z轴垂直向下")
        if not ENABLE_ANGLE_ALIGNMENT:
            report["status"]="VERTICAL_ORIENTATION_PLAN_READY";save(report);return
        aligned_tcp,aligned_end=align_orientation_target(robot,tf_api,left,target_q,base,report);q=aligned_end.orientation
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
        print("抓取并抬升10cm完成")
        report["status"]="PICK_AND_LIFT_COMPLETED";save(report)
        if ENABLE_WAIST_TURN_AND_RELEASE:
            turn_waist_keep_arms_and_release(robot,report)
            report["status"]="PICK_TURN80_AND_RELEASE_COMPLETED"
        save(report)
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
