#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
33_yolo_product_xyz_locator_v2.py
READ ONLY

YOLO -> center_pixel -> Depth -> Camera XYZ -> Base XYZ
基于已验证成功的:
19_camera_xyz_to_base_link_v3.py
23_pose_debugger_live.py

S 保存结果
Q/ESC 退出
"""
import json
import time
from pathlib import Path
import cv2
import numpy as np
import agibot_gdk
from ultralytics import YOLO

MODEL_PATH='runs/detect/runs/product_detector/weights/best.pt'
OUTPUT_JSON='product_xyz_v2.json'
HEAD_FRAME='head_link3'
CAMERA_TIMEOUT_MS=1000.0
DEPTH_RADIUS=3
MIN_DEPTH_RAW=50.0
MAX_DEPTH_RAW=10000.0


def safe_get_image(camera,camera_type):
    try:
        return camera.get_latest_image(camera_type,CAMERA_TIMEOUT_MS)
    except Exception:
        return None


def decode_color(image):
    data=np.frombuffer(image.data,dtype=np.uint8)
    frame=cv2.imdecode(data,cv2.IMREAD_COLOR)
    return frame


def decode_depth(image):
    try:
        if image.bit_depth==32:
            data=np.frombuffer(image.data,dtype=np.float32)
        else:
            data=np.frombuffer(image.data,dtype=np.uint16)
        return data.reshape((image.height,image.width))
    except Exception:
        return None


def read_depth_intrinsics(camera):
    obj=camera.get_camera_intrinsic(agibot_gdk.CameraType.kHeadDepth)
    fx,fy,cx,cy=map(float,list(obj.intrinsic)[:4])
    return fx,fy,cx,cy


def median_depth(depth_map,u,v,radius=DEPTH_RADIUS):
    h,w=depth_map.shape[:2]
    patch=depth_map[max(0,v-radius):min(h,v+radius+1),max(0,u-radius):min(w,u+radius+1)].astype(np.float64)
    valid=patch[np.isfinite(patch)]
    valid=valid[(valid>MIN_DEPTH_RAW)&(valid<MAX_DEPTH_RAW)]
    if valid.size==0:
        return None
    return float(np.median(valid))


def quaternion_to_matrix(x,y,z,w):
    q=np.array([x,y,z,w],dtype=np.float64)
    q=q/np.linalg.norm(q)
    x,y,z,w=q
    return np.array([
        [1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]
    ])


def transform_to_matrix(t):
    m=np.eye(4)
    m[:3,:3]=quaternion_to_matrix(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)
    m[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
    return m


def build_camera_to_base_matrix(tf_api):
    base_to_head=tf_api.get_tf_from_base_link(HEAD_FRAME)
    camera_to_head=tf_api.get_tf_from_sensor(agibot_gdk.SensorExtrinsicType.kHeadRGBDToHeadLink3)
    return transform_to_matrix(base_to_head) @ transform_to_matrix(camera_to_head)


def transform_point(matrix,xyz):
    p=np.array([xyz[0],xyz[1],xyz[2],1.0])
    return (matrix@p)[:3]


if not Path(MODEL_PATH).exists():
    raise RuntimeError(f'Model not found: {MODEL_PATH}')

model=YOLO(MODEL_PATH)

if agibot_gdk.gdk_init()!=agibot_gdk.GDKRes.kSuccess:
    raise RuntimeError('GDK init failed')

camera=agibot_gdk.Camera()
tf_api=agibot_gdk.TF()
time.sleep(3)
fx,fy,cx0,cy0=read_depth_intrinsics(camera)
t_base_camera=build_camera_to_base_matrix(tf_api)

print('33_yolo_product_xyz_locator_v2.py')
print('READ ONLY MODE')

try:
    while True:
        color_img=safe_get_image(camera,agibot_gdk.CameraType.kHeadColor)
        depth_img_obj=safe_get_image(camera,agibot_gdk.CameraType.kHeadDepth)
        if color_img is None or depth_img_obj is None:
            continue

        color=decode_color(color_img)
        depth_map=decode_depth(depth_img_obj)
        if color is None or depth_map is None:
            continue

        result=model.predict(color,conf=0.25,verbose=False)
        view=color.copy()
        save_obj=None

        if len(result) and len(result[0].boxes):
            b=result[0].boxes[0]
            x1,y1,x2,y2=map(int,b.xyxy[0].tolist())
            conf=float(b.conf[0])
            cx=int((x1+x2)/2)
            cy=int((y1+y2)/2)

            dh,dw=depth_map.shape[:2]
            du=int(round(cx*dw/color.shape[1]))
            dv=int(round(cy*dh/color.shape[0]))
            du=max(0,min(dw-1,du))
            dv=max(0,min(dh-1,dv))

            raw=median_depth(depth_map,du,dv)
            if raw is not None:
                depth_m=raw/1000.0 if raw>20 else raw
                camera_xyz=np.array([(du-cx0)*depth_m/fx,(dv-cy0)*depth_m/fy,depth_m])
                base_xyz=transform_point(t_base_camera,camera_xyz)

                cv2.rectangle(view,(x1,y1),(x2,y2),(0,255,0),2)
                cv2.circle(view,(cx,cy),5,(0,0,255),-1)
                cv2.putText(view,f'Base=({base_xyz[0]:.3f},{base_xyz[1]:.3f},{base_xyz[2]:.3f})',(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,255),2)

                save_obj={
                    'confidence':round(conf,4),
                    'center_pixel':[cx,cy],
                    'depth_m':round(depth_m,4),
                    'camera_xyz':[round(float(v),4) for v in camera_xyz],
                    'base_xyz':[round(float(v),4) for v in base_xyz]
                }

        cv2.imshow('YOLO Product XYZ Locator V2',view)
        k=cv2.waitKey(1)&0xFF
        if k in (27,ord('q'),ord('Q')):
            break
        if k in (ord('s'),ord('S')) and save_obj:
            Path(OUTPUT_JSON).write_text(json.dumps(save_obj,indent=2),encoding='utf-8')
            print(save_obj)
finally:
    cv2.destroyAllWindows()
    try:
        camera.close_camera()
    except Exception:
        pass
    agibot_gdk.gdk_release()
