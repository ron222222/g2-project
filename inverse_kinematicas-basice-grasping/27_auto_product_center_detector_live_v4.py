#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
import json,time
from pathlib import Path
import cv2, numpy as np
import agibot_gdk

SAVE_JSON='product_center_live_v4.json'
SAVE_IMG='product_center_live_v4.png'

# 固定产品工作区(根据你的实拍图)
WORK_X_MIN=260
WORK_X_MAX=460
WORK_Y_MIN=90
WORK_Y_MAX=230

agibot_gdk.gdk_init()
camera=agibot_gdk.Camera()
time.sleep(3)

print('27_auto_product_center_detector_live_v4.py')
print('READ ONLY MODE')
print('固定产品工作区ROI + 黑色HSV + 尺寸过滤')

try:
    while True:
        rgb=camera.get_latest_image(agibot_gdk.CameraType.kHeadColor,1000.0)
        if rgb is None:
            continue

        frame=cv2.imdecode(np.frombuffer(rgb.data,np.uint8),cv2.IMREAD_COLOR)
        if frame is None:
            continue

        h,w=frame.shape[:2]

        x1=max(0,min(WORK_X_MIN,w-1))
        x2=max(x1+1,min(WORK_X_MAX,w))
        y1=max(0,min(WORK_Y_MIN,h-1))
        y2=max(y1+1,min(WORK_Y_MAX,h))

        roi=frame[y1:y2,x1:x2].copy()

        hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
        lower=np.array([0,0,0])
        upper=np.array([180,255,90])
        mask=cv2.inRange(hsv,lower,upper)

        kernel=np.ones((5,5),np.uint8)
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)

        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        best=None
        best_score=-1
        center=None
        area=0.0
        view=frame.copy()

        cv2.rectangle(view,(x1,y1),(x2,y2),(255,0,0),2)

        for c in contours:
            area_i=cv2.contourArea(c)
            if area_i<100:
                continue

            rect=cv2.minAreaRect(c)
            rw,rh=rect[1]
            long_side=max(rw,rh)
            short_side=max(min(rw,rh),1.0)
            aspect=long_side/short_side

            # V4新增：尺寸硬限制
            if not (50 <= long_side <= 180):
                continue
            if not (10 <= short_side <= 60):
                continue
            if aspect < 2.0:
                continue

            score=area_i*aspect
            if score>best_score:
                best_score=score
                best=(c,rect,area_i)

        if best is not None:
            c,rect,area=best
            box=cv2.boxPoints(rect).astype(int)
            box[:,0]+=x1
            box[:,1]+=y1

            cx=int(rect[0][0])+x1
            cy=int(rect[0][1])+y1
            center=[cx,cy]

            cv2.drawContours(view,[box],0,(0,255,0),2)
            cv2.circle(view,(cx,cy),8,(0,0,255),-1)
            cv2.putText(view,f'Center=({cx},{cy})',(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,255),2)

        cv2.imshow('Product Detector V4',view)
        key=cv2.waitKey(1)&0xFF

        if key in (27,ord('q'),ord('Q')):
            break

        if key in (ord('s'),ord('S')) and center is not None:
            cv2.imwrite(SAVE_IMG,view)
            Path(SAVE_JSON).write_text(json.dumps({
                'roi':[x1,y1,x2,y2],
                'center_pixel':center,
                'contour_area':float(area)
            },ensure_ascii=False,indent=2),encoding='utf-8')
            print('保存成功')
            print('center_pixel=',center)

finally:
    cv2.destroyAllWindows()
    try:
        camera.close_camera()
    except Exception:
        pass
    agibot_gdk.gdk_release()
