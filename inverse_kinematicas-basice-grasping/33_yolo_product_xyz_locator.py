#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
33_yolo_product_xyz_locator.py

YOLO -> center_pixel -> Depth -> Camera XYZ

READ ONLY
不运动机器人
仅输出检测结果与三维坐标

适配:
- Python 3.10
- agibot_gdk
- ultralytics

按键:
S 保存 JSON
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
OUTPUT_JSON = 'product_xyz.json'

# 根据实际相机参数修改
FX = 525.0
FY = 525.0
CX = 320.0
CY = 240.0


if not Path(MODEL_PATH).exists():
    raise RuntimeError(f'Model not found: {MODEL_PATH}')

model = YOLO(MODEL_PATH)

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    raise RuntimeError('GDK init failed')

camera = agibot_gdk.Camera()
time.sleep(2)


def depth_to_xyz(u, v, depth_m):
    x = (u - CX) * depth_m / FX
    y = (v - CY) * depth_m / FY
    z = depth_m
    return x, y, z


print('33_yolo_product_xyz_locator.py')
print('READ ONLY')

try:
    while True:

        rgb = camera.get_latest_image(
            agibot_gdk.CameraType.kHeadColor,
            1000.0
        )

        depth = camera.get_latest_image(
            agibot_gdk.CameraType.kHeadDepth,
            1000.0
        )

        if rgb is None or depth is None:
            continue

        color = cv2.imdecode(
            np.frombuffer(rgb.data, np.uint8),
            cv2.IMREAD_COLOR
        )

        depth_img = cv2.imdecode(
            np.frombuffer(depth.data, np.uint8),
            cv2.IMREAD_UNCHANGED
        )

        if color is None or depth_img is None:
            continue

        result_data = None

        results = model.predict(
            color,
            conf=0.25,
            verbose=False
        )

        view = color.copy()

        if len(results) > 0 and len(results[0].boxes) > 0:

            box = results[0].boxes[0]

            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = float(box.conf[0])

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            h, w = depth_img.shape[:2]

            if 0 <= cx < w and 0 <= cy < h:

                d = float(depth_img[cy, cx])

                # 常见深度相机毫米单位
                if d > 10:
                    depth_m = d / 1000.0
                else:
                    depth_m = d

                cam_x, cam_y, cam_z = depth_to_xyz(
                    cx,
                    cy,
                    depth_m
                )

                result_data = {
                    'confidence': round(conf, 4),
                    'center_pixel': [cx, cy],
                    'depth_m': round(depth_m, 4),
                    'camera_xyz': {
                        'x': round(cam_x, 4),
                        'y': round(cam_y, 4),
                        'z': round(cam_z, 4),
                    }
                }

                cv2.rectangle(view, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(view, (cx, cy), 5, (0, 0, 255), -1)

                txt = (
                    f'XYZ=({cam_x:.3f},'
                    f'{cam_y:.3f},'
                    f'{cam_z:.3f})m'
                )

                cv2.putText(
                    view,
                    txt,
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )

        cv2.imshow('YOLO Product XYZ Locator', view)

        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord('q'), ord('Q')):
            break

        if key in (ord('s'), ord('S')) and result_data:
            Path(OUTPUT_JSON).write_text(
                json.dumps(result_data, indent=2),
                encoding='utf-8'
            )
            print(result_data)

finally:
    cv2.destroyAllWindows()
    try:
        camera.close_camera()
    except Exception:
        pass
    agibot_gdk.gdk_release()
