#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
33_yolo_product_xyz_locator_v3.py
READ ONLY

YOLO -> center_pixel -> Depth -> Camera XYZ -> Base XYZ
增强显示版本，参考19_camera_xyz_to_base_link_v3.py和23_pose_debugger_live.py

S 保存JSON
Q/ESC 退出
"""
import json,time,cv2,numpy as np,agibot_gdk
from pathlib import Path
from ultralytics import YOLO

MODEL_PATH='runs/detect/runs/product_detector/weights/best.pt'
OUT_JSON='product_xyz_v3.json'
HEAD_FRAME='head_link3'
CAMERA_TIMEOUT_MS=1000.0
DEPTH_RADIUS=3
MIN_DEPTH_RAW=50.0
MAX_DEPTH_RAW=10000.0
TABLE_HEIGHT_M=0.750
TABLE_Z_TOLERANCE_M=0.150

# 复用验证成功逻辑

def safe_get_image(cam,tp):
    try:return cam.get_latest_image(tp,CAMERA_TIMEOUT_MS)
    except: return None

def decode_color(img):
    return cv2.imdecode(np.frombuffer(img.data,np.uint8),cv2.IMREAD_COLOR)

def decode_depth(img):
    try:
        dt=np.float32 if img.bit_depth==32 else np.uint16
        arr=np.frombuffer(img.data,dtype=dt)
        return arr.reshape((img.height,img.width))
    except: return None

def read_intrinsic(cam):
    vals=list(cam.get_camera_intrinsic(agibot_gdk.CameraType.kHeadDepth).intrinsic)
    return map(float,vals[:4])

def median_depth(dm,u,v):
    h,w=dm.shape[:2]
    p=dm[max(0,v-DEPTH_RADIUS):min(h,v+DEPTH_RADIUS+1),max(0,u-DEPTH_RADIUS):min(w,u+DEPTH_RADIUS+1)].astype(float)
    valid=p[np.isfinite(p)]
    valid=valid[(valid>MIN_DEPTH_RAW)&(valid<MAX_DEPTH_RAW)]
    if valid.size==0:return None,0
    return float(np.median(valid)),int(valid.size)

def qmat(x,y,z,w):
    q=np.array([x,y,z,w],float);q=q/np.linalg.norm(q);x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def tfmat(t):
    m=np.eye(4);m[:3,:3]=qmat(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w);m[:3,3]=[t.translation.x,t.translation.y,t.translation.z];return m

def build_tf(tf_api):
    return tfmat(tf_api.get_tf_from_base_link(HEAD_FRAME)) @ tfmat(tf_api.get_tf_from_sensor(agibot_gdk.SensorExtrinsicType.kHeadRGBDToHeadLink3))

def transform(m,p):
    return (m@np.array([p[0],p[1],p[2],1.0]))[:3]

model=YOLO(MODEL_PATH)
agibot_gdk.gdk_init();cam=agibot_gdk.Camera();tf_api=agibot_gdk.TF();time.sleep(3)
fx,fy,cx,cy=read_intrinsic(cam)
T=build_tf(tf_api)
print('33_yolo_product_xyz_locator_v3.py')
print('READ ONLY MODE')
print(f'fx={fx:.3f} fy={fy:.3f} cx={cx:.3f} cy={cy:.3f}')
try:
  while True:
    c=safe_get_image(cam,agibot_gdk.CameraType.kHeadColor)
    d=safe_get_image(cam,agibot_gdk.CameraType.kHeadDepth)
    if c is None or d is None: continue
    color=decode_color(c);depth=decode_depth(d)
    if color is None or depth is None: continue
    view=color.copy();saved=None
    res=model.predict(color,conf=0.25,verbose=False)
    if len(res) and len(res[0].boxes):
      b=res[0].boxes[0]
      x1,y1,x2,y2=map(int,b.xyxy[0].tolist())
      conf=float(b.conf[0]);px=int((x1+x2)/2);py=int((y1+y2)/2)
      dh,dw=depth.shape[:2]
      du=max(0,min(dw-1,int(round(px*dw/color.shape[1]))))
      dv=max(0,min(dh-1,int(round(py*dh/color.shape[0]))))
      raw,count=median_depth(depth,du,dv)
      if raw is not None:
        zm=raw/1000.0 if raw>20 else raw
        cxyz=np.array([(du-cx)*zm/fx,(dv-cy)*zm/fy,zm])
        bxyz=transform(T,cxyz)
        delta=abs(float(bxyz[2])-TABLE_HEIGHT_M)
        state='PASS' if delta<=TABLE_Z_TOLERANCE_M else 'WARNING'
        cv2.rectangle(view,(x1,y1),(x2,y2),(0,255,0),2)
        cv2.circle(view,(px,py),5,(0,0,255),-1)
        lines=[
        f'Conf={conf:.3f}',
        f'RGB Pixel=({px},{py})',
        f'Depth Pixel=({du},{dv})',
        f'Raw Depth={raw:.1f}',
        f'Samples={count}',
        f'Camera XYZ=({cxyz[0]:.3f},{cxyz[1]:.3f},{cxyz[2]:.3f})',
        f'Base XYZ=({bxyz[0]:.3f},{bxyz[1]:.3f},{bxyz[2]:.3f})',
        f'TABLE CHECK {state} delta={delta:.3f}m']
        for i,t in enumerate(lines): cv2.putText(view,t,(10,25+i*25),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,255),2)
        saved={'confidence':conf,'center_pixel':[px,py],'depth_pixel':[du,dv],'raw_depth':raw,'sample_count':count,'camera_xyz':[float(v) for v in cxyz],'base_xyz':[float(v) for v in bxyz],'table_check':state}
    cv2.imshow('YOLO Product XYZ Locator V3',view)
    k=cv2.waitKey(1)&0xFF
    if k in (27,ord('q'),ord('Q')): break
    if k in (ord('s'),ord('S')) and saved:
      Path(OUT_JSON).write_text(json.dumps(saved,indent=2),encoding='utf-8');print(saved)
finally:
  cv2.destroyAllWindows();
  try: cam.close_camera()
  except: pass
  agibot_gdk.gdk_release()
