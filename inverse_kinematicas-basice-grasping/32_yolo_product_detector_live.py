#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
32_yolo_product_detector_live.py
READ ONLY

实时YOLO检测产品中心。
不控制机器人。

S 保存当前检测结果
Q/ESC 退出
"""
import json
import time
from pathlib import Path

import cv2
import numpy as np
import agibot_gdk
from ultralytics import YOLO

MODEL_PATH = 'runs/detect/runs/product_detector/weights/best.pt'
SAVE_JSON = 'product_detect.json'
SAVE_IMG = 'product_detect.png'

if not Path(MODEL_PATH).exists():
    raise RuntimeError(f'模型不存在: {MODEL_PATH}')

model = YOLO(MODEL_PATH)

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    raise RuntimeError('GDK初始化失败')

camera = agibot_gdk.Camera()
time.sleep(2)

print('32_yolo_product_detector_live.py')
print('READ ONLY MODE')
print('Model =', MODEL_PATH)
print('S 保存, Q退出')

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

        result_json = None

        results = model.predict(
            frame,
            verbose=False,
            conf=0.25
        )

        view = frame.copy()

        if len(results):
            r = results[0]

            if r.boxes is not None and len(r.boxes) > 0:
                box = r.boxes[0]

                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cls = int(box.cls[0])

                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                cv2.rectangle(view,(x1,y1),(x2,y2),(0,255,0),2)
                cv2.circle(view,(cx,cy),6,(0,0,255),-1)
                cv2.putText(
                    view,
                    f'product {conf:.3f}',
                    (x1,max(20,y1-10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,(0,255,255),2
                )

                result_json = {
                    'class_id': cls,
                    'class_name': 'product',
                    'confidence': conf,
                    'bbox':[x1,y1,x2,y2],
                    'center_pixel':[cx,cy]
                }

        cv2.imshow('YOLO Product Detector Live', view)

        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord('q'), ord('Q')):
            break

        if key in (ord('s'), ord('S')) and result_json:
            cv2.imwrite(SAVE_IMG, view)
            Path(SAVE_JSON).write_text(
                json.dumps(result_json, ensure_ascii=False, indent=2),
                encoding='utf-8'
            )
            print('保存成功')
            print(result_json)

finally:
    cv2.destroyAllWindows()
    try:
        camera.close_camera()
    except Exception:
        pass
    agibot_gdk.gdk_release()
