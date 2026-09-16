#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
27_auto_product_center_detector_live_v2.py
READ ONLY

基于已验证成功的 get_latest_image() 接口。
实时读取 HeadColor。
OpenCV 自动提取最大轮廓并计算中心点。
按 S 保存结果。
按 Q 或 ESC 退出。
不控制机器人。
"""

import json
import time
from pathlib import Path
import cv2
import numpy as np
import agibot_gdk

SAVE_JSON='product_center_live_v2.json'
SAVE_IMG='product_center_live_v2.png'

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    raise RuntimeError('GDK初始化失败')

camera = agibot_gdk.Camera()
time.sleep(3)

print('27_auto_product_center_detector_live_v2.py')
print('READ ONLY MODE')
print('S 保存结果')
print('Q/ESC 退出')

try:
    while True:
        rgb = camera.get_latest_image(
            agibot_gdk.CameraType.kHeadColor,
            1000.0
        )

        if rgb is None:
            continue

        frame = cv2.imdecode(
            np.frombuffer(rgb.data, np.uint8),
            cv2.IMREAD_COLOR
        )

        if frame is None:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)

        _, mask = cv2.threshold(
            blur, 0, 255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        view = frame.copy()
        center = None
        area = 0.0

        if contours:
            cnt = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(cnt))
            m = cv2.moments(cnt)

            if m['m00'] != 0:
                cx = int(m['m10']/m['m00'])
                cy = int(m['m01']/m['m00'])
                center = [cx, cy]

                cv2.drawContours(view, [cnt], -1, (0,255,0), 2)
                cv2.circle(view, (cx,cy), 8, (0,0,255), -1)
                cv2.putText(view, f'Center=({cx},{cy})',
                            (10,30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0,255,255), 2)

        cv2.imshow('Auto Product Center Detector V2', view)

        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord('q'), ord('Q')):
            break

        if key in (ord('s'), ord('S')) and center is not None:
            cv2.imwrite(SAVE_IMG, view)
            Path(SAVE_JSON).write_text(
                json.dumps({
                    'center_pixel': center,
                    'contour_area': area
                }, ensure_ascii=False, indent=2),
                encoding='utf-8'
            )
            print('保存成功')
            print('center_pixel =', center)
            print('contour_area =', area)

finally:
    cv2.destroyAllWindows()
    try:
        camera.close_camera()
    except Exception:
        pass
    agibot_gdk.gdk_release()
