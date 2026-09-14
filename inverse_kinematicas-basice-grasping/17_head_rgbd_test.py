#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
17_head_rgbd_test.py
使用 HeadColor + HeadDepth 读取深度
鼠标点击画面任意点，显示像素与深度值
ESC退出
"""

import time
import cv2
import numpy as np
import agibot_gdk

click_x = -1
click_y = -1
last_depth = None


def mouse_cb(event, x, y, flags, param):
    global click_x, click_y
    if event == cv2.EVENT_LBUTTONDOWN:
        click_x = x
        click_y = y
        print(f'Pixel=({x},{y})')


def decode_depth(img):
    try:
        if img.bit_depth == 16:
            arr = np.frombuffer(img.data, dtype=np.uint16)
        else:
            arr = np.frombuffer(img.data, dtype=np.float32)
        return arr.reshape((img.height, img.width))
    except Exception as e:
        print('Depth decode error:', e)
        return None


if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    raise RuntimeError('GDK初始化失败')

camera = agibot_gdk.Camera()
time.sleep(3)

cv2.namedWindow('Head RGBD Test')
cv2.setMouseCallback('Head RGBD Test', mouse_cb)

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

        frame = cv2.imdecode(
            np.frombuffer(rgb.data, np.uint8),
            cv2.IMREAD_COLOR
        )
        if frame is None:
            continue

        depth_map = decode_depth(depth)

        if depth_map is not None and click_x >= 0 and click_y >= 0:
            if click_y < depth_map.shape[0] and click_x < depth_map.shape[1]:
                last_depth = float(depth_map[click_y, click_x])
                cv2.circle(frame, (click_x, click_y), 6, (0,0,255), -1)

        cv2.putText(frame, f'Pixel: ({click_x},{click_y})',
                    (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0,255,0), 2)

        if last_depth is not None:
            cv2.putText(frame, f'Depth: {last_depth:.2f}',
                        (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0,255,255), 2)

        cv2.imshow('Head RGBD Test', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
finally:
    cv2.destroyAllWindows()
    try:
        camera.close_camera()
    except:
        pass
    agibot_gdk.gdk_release()
