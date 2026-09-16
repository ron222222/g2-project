#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
28_yolo_obb_dataset_capture.py
READ ONLY

YOLO OBB 数据采集工具

按键:
S  保存一张图片
A  开启自动采集(1秒1张)
D  停止自动采集
Q/ESC 退出

保存目录:
./dataset/images/
"""

import time
from pathlib import Path
import cv2
import numpy as np
import agibot_gdk

SAVE_DIR = Path('dataset/images')
SAVE_DIR.mkdir(parents=True, exist_ok=True)

counter = len(list(SAVE_DIR.glob('*.jpg')))
auto_mode = False
last_capture = 0

agibot_gdk.gdk_init()
camera = agibot_gdk.Camera()
time.sleep(3)

print('28_yolo_obb_dataset_capture.py')
print('READ ONLY MODE')
print('S=保存 A=自动采集 D=停止自动采集 Q=退出')

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

        view = frame.copy()
        cv2.putText(view,f'Images={counter}',(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,255),2)
        cv2.putText(view,f'Auto={auto_mode}',(10,60),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),2)

        now = time.time()
        if auto_mode and now-last_capture >= 1.0:
            counter += 1
            fn = SAVE_DIR / f'{counter:06d}.jpg'
            cv2.imwrite(str(fn), frame)
            print('AUTO SAVE:', fn)
            last_capture = now

        cv2.imshow('YOLO OBB Dataset Capture', view)
        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord('q'), ord('Q')):
            break
        elif key in (ord('s'), ord('S')):
            counter += 1
            fn = SAVE_DIR / f'{counter:06d}.jpg'
            cv2.imwrite(str(fn), frame)
            print('SAVE:', fn)
        elif key in (ord('a'), ord('A')):
            auto_mode = True
            print('自动采集开启')
        elif key in (ord('d'), ord('D')):
            auto_mode = False
            print('自动采集关闭')
finally:
    cv2.destroyAllWindows()
    try:
        camera.close_camera()
    except Exception:
        pass
    agibot_gdk.gdk_release()
