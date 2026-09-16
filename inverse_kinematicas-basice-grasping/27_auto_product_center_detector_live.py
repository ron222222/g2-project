#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
27_auto_product_center_detector_live.py
DRY_RUN ONLY

功能:
1. 从G2头部RGB相机获取实时图像
2. OpenCV自动检测最大目标轮廓
3. 计算目标中心(cx,cy)
4. 实时显示检测结果
5. 按S保存结果
6. 不控制机器人
7. 不发送运动命令
"""

import json
import time
from pathlib import Path
import cv2
import numpy as np
import agibot_gdk

SAVE_JSON='product_center_live.json'
SAVE_IMG='product_center_live.png'

if agibot_gdk.gdk_init()!=agibot_gdk.GDKRes.kSuccess:
    raise RuntimeError('GDK初始化失败')

try:
    cam = agibot_gdk.Camera()
    time.sleep(2)

    print('27_auto_product_center_detector_live.py')
    print('DRY_RUN ONLY')
    print('按 S 保存')
    print('按 Q 退出')

    while True:
        frame=None

        # 常见头部彩色相机名称
        for name in ['kHeadColor','HEAD_COLOR','HeadColor']:
            try:
                if hasattr(agibot_gdk,name):
                    frame = cam.get_image(getattr(agibot_gdk,name))
                    break
            except Exception:
                pass

        if frame is None:
            print('未获取到RGB图像')
            break

        try:
            img = np.asarray(frame)
        except Exception:
            print('图像转换失败')
            break

        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        v = hsv[:,:,2]
        _, mask = cv2.threshold(v,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        kernel=np.ones((5,5),np.uint8)
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)

        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        view=img.copy()
        cx=None
        cy=None
        area=0

        if contours:
            cnt=max(contours,key=cv2.contourArea)
            area=float(cv2.contourArea(cnt))
            m=cv2.moments(cnt)
            if m['m00']!=0:
                cx=int(m['m10']/m['m00'])
                cy=int(m['m01']/m['m00'])
                cv2.drawContours(view,[cnt],-1,(0,255,0),2)
                cv2.circle(view,(cx,cy),8,(0,0,255),-1)

        cv2.imshow('Auto Product Center Detector',view)
        key=cv2.waitKey(1)&0xFF

        if key==ord('s') and cx is not None:
            cv2.imwrite(SAVE_IMG,view)
            Path(SAVE_JSON).write_text(json.dumps({
                'center_pixel':[cx,cy],
                'contour_area':area
            },ensure_ascii=False,indent=2),encoding='utf-8')
            print('保存成功')
            print('center_pixel =',[cx,cy])
            print('contour_area =',area)

        if key==ord('q'):
            break

    cv2.destroyAllWindows()
finally:
    try:
        agibot_gdk.gdk_release()
    except Exception:
        pass
