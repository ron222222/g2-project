#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
27_auto_product_center_detector_live_v3.py
READ ONLY

V3改进:
1. 使用 get_latest_image(kHeadColor)
2. ROI裁掉桌面外区域
3. HSV黑色目标提取
4. 面积过滤
5. 长宽比过滤
6. 输出 center_pixel
7. 保存 product_center_live_v3.json
8. 不控制机器人
"""
import json, time
from pathlib import Path
import cv2, numpy as np
import agibot_gdk

SAVE_JSON='product_center_live_v3.json'
SAVE_IMG='product_center_live_v3.png'

agibot_gdk.gdk_init()
camera=agibot_gdk.Camera()
time.sleep(3)

print('27_auto_product_center_detector_live_v3.py')
print('READ ONLY MODE')
print('S 保存 Q退出')

try:
    while True:
        rgb=camera.get_latest_image(agibot_gdk.CameraType.kHeadColor,1000.0)
        if rgb is None:
            continue
        frame=cv2.imdecode(np.frombuffer(rgb.data,np.uint8),cv2.IMREAD_COLOR)
        if frame is None:
            continue

        h,w=frame.shape[:2]
        roi=frame[int(h*0.25):int(h*0.80), int(w*0.20):int(w*0.85)].copy()

        hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
        lower=np.array([0,0,0])
        upper=np.array([180,255,90])
        mask=cv2.inRange(hsv,lower,upper)

        kernel=np.ones((5,5),np.uint8)
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)

        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        best=None
        best_score=0
        for c in contours:
            area=cv2.contourArea(c)
            if area<500:
                continue
            rect=cv2.minAreaRect(c)
            rw,rh=rect[1]
            long_side=max(rw,rh)
            short_side=max(min(rw,rh),1.0)
            aspect=long_side/short_side
            if aspect<2.0:
                continue
            score=area*aspect
            if score>best_score:
                best_score=score
                best=(c,rect,area)

        view=frame.copy()
        cv2.rectangle(view,(int(w*0.20),int(h*0.25)),(int(w*0.85),int(h*0.80)),(255,0,0),2)
        center=None
        area=0
        if best:
            c,rect,area=best
            box=cv2.boxPoints(rect)
            box=np.int32(box)
            box[:,0]+=int(w*0.20)
            box[:,1]+=int(h*0.25)
            cx=int(rect[0][0])+int(w*0.20)
            cy=int(rect[0][1])+int(h*0.25)
            center=[cx,cy]
            cv2.drawContours(view,[box],0,(0,255,0),2)
            cv2.circle(view,(cx,cy),7,(0,0,255),-1)
            cv2.putText(view,f'Center=({cx},{cy})',(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,255),2)

        cv2.imshow('Product Detector V3',view)
        key=cv2.waitKey(1)&0xFF
        if key in (27,ord('q'),ord('Q')):
            break
        if key in (ord('s'),ord('S')) and center:
            cv2.imwrite(SAVE_IMG,view)
            Path(SAVE_JSON).write_text(json.dumps({'center_pixel':center,'contour_area':float(area)},ensure_ascii=False,indent=2),encoding='utf-8')
            print('保存成功',center,area)
finally:
    cv2.destroyAllWindows()
    try: camera.close_camera()
    except: pass
    agibot_gdk.gdk_release()
