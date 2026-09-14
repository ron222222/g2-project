#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
15_right_hand_camera_preview.py
G2右手相机预览程序
功能：
1. 显示右手RGB相机实时画面
2. 显示FPS
3. 按 S 保存图片
4. 按 ESC 退出
"""

import time
import cv2
import numpy as np
import agibot_gdk


def main():
    print("初始化GDK...")

    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK初始化失败")
        return

    camera = agibot_gdk.Camera()

    print("等待右手相机初始化...")
    time.sleep(3)

    frame_count = 0
    last_time = time.time()

    print("启动右手相机预览")
    print("按 S 保存图片")
    print("按 ESC 退出")

    while True:
        try:
            image = camera.get_latest_image(
                agibot_gdk.CameraType.kHandRightColor,
                1000.0
            )

            if image is None:
                continue

            nparr = np.frombuffer(image.data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is None:
                continue

            frame_count += 1
            current_time = time.time()

            if current_time - last_time >= 1.0:
                fps = frame_count / (current_time - last_time)
                frame_count = 0
                last_time = current_time
            else:
                fps = 0

            cv2.putText(
                frame,
                f"FPS: {fps:.1f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            cv2.imshow("G2 Right Hand Camera", frame)

            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                break

            if key == ord('s'):
                filename = f"right_hand_{image.timestamp_ns}.jpg"
                cv2.imwrite(filename, frame)
                print(f"保存成功: {filename}")

        except Exception as e:
            print(f"相机异常: {e}")

    cv2.destroyAllWindows()

    try:
        camera.close_camera()
    except:
        pass

    agibot_gdk.gdk_release()
    print("程序结束")


if __name__ == '__main__':
    main()
