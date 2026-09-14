#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
17_depth_test.py
G2 右手RGB+Depth测试
点击RGB图像任意位置，读取对应深度值
ESC退出
"""

import time
import cv2
import numpy as np
import agibot_gdk

clicked_x = -1
clicked_y = -1
last_depth = None


def mouse_callback(event, x, y, flags, param):
    global clicked_x, clicked_y
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_x = x
        clicked_y = y
        print(f"Pixel=({x},{y})")


def decode_depth(image):
    try:
        if image.bit_depth == 16:
            depth = np.frombuffer(image.data, dtype=np.uint16)
            depth = depth.reshape((image.height, image.width))
            return depth
        elif image.bit_depth == 32:
            depth = np.frombuffer(image.data, dtype=np.float32)
            depth = depth.reshape((image.height, image.width))
            return depth
    except Exception as e:
        print(f"Depth decode error: {e}")
    return None


def main():
    global last_depth

    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK初始化失败")
        return

    camera = agibot_gdk.Camera()
    time.sleep(3)

    cv2.namedWindow('G2 RGBD Depth Test')
    cv2.setMouseCallback('G2 RGBD Depth Test', mouse_callback)

    try:
        while True:
            rgb_img = camera.get_latest_image(
                agibot_gdk.CameraType.kHandRightColor,
                1000.0
            )

            depth_img = camera.get_latest_image(
                agibot_gdk.CameraType.kHandRightDepth,
                1000.0
            )

            if rgb_img is None or depth_img is None:
                continue

            frame = cv2.imdecode(
                np.frombuffer(rgb_img.data, np.uint8),
                cv2.IMREAD_COLOR
            )

            if frame is None:
                continue

            depth = decode_depth(depth_img)

            if depth is not None and clicked_x >= 0 and clicked_y >= 0:
                if clicked_y < depth.shape[0] and clicked_x < depth.shape[1]:
                    d = depth[clicked_y, clicked_x]
                    last_depth = float(d)
                    cv2.circle(frame, (clicked_x, clicked_y), 6, (0,0,255), -1)

            cv2.putText(
                frame,
                f'Pixel: ({clicked_x},{clicked_y})',
                (10,30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0,255,0),
                2
            )

            if last_depth is not None:
                cv2.putText(
                    frame,
                    f'Depth: {last_depth:.1f}',
                    (10,60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0,255,255),
                    2
                )

            cv2.imshow('G2 RGBD Depth Test', frame)

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


if __name__ == '__main__':
    main()
