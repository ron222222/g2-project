#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
16_product_detector_opencv_v2.py
针对G2右手相机 + 黑色产品 + 白色桌面优化
V2改进：
1. HSV黑色提取
2. 忽略下方夹爪区域
3. 长宽比过滤
4. 面积过滤
5. 输出中心点、角度、长宽
"""
import cv2
import time
import numpy as np
import agibot_gdk

ROI_BOTTOM_RATIO = 0.72
MIN_AREA = 5000
MIN_ASPECT = 2.5

class ProductDetectorV2:
    def __init__(self):
        self.camera = agibot_gdk.Camera()
        time.sleep(2)

    def detect(self, frame):
        h, w = frame.shape[:2]
        roi = frame[:int(h * ROI_BOTTOM_RATIO), :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 90])

        mask = cv2.inRange(hsv, lower_black, upper_black)

        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_score = 0

        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_AREA:
                continue

            rect = cv2.minAreaRect(c)
            (_, _), (rw, rh), _ = rect

            long_side = max(rw, rh)
            short_side = max(min(rw, rh), 1.0)
            aspect = long_side / short_side

            if aspect < MIN_ASPECT:
                continue

            score = area * aspect

            if score > best_score:
                best_score = score
                best = rect

        display = frame.copy()

        cv2.line(display,
                 (0, int(h * ROI_BOTTOM_RATIO)),
                 (w, int(h * ROI_BOTTOM_RATIO)),
                 (255, 0, 0), 2)

        if best is None:
            cv2.putText(display,
                        'Product Not Found',
                        (20,40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0,0,255),
                        2)
            return display, None

        (cx, cy), (rw, rh), angle = best

        box = cv2.boxPoints(best)
        box = np.int32(box)

        cv2.drawContours(display,[box],0,(0,255,0),3)
        cv2.circle(display,(int(cx),int(cy)),6,(0,0,255),-1)

        text1=f'Center=({int(cx)},{int(cy)})'
        text2=f'Angle={angle:.1f}'
        text3=f'Size={max(rw,rh):.0f}x{min(rw,rh):.0f}'

        cv2.putText(display,text1,(10,35),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2)
        cv2.putText(display,text2,(10,65),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2)
        cv2.putText(display,text3,(10,95),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2)

        result = {
            'cx': float(cx),
            'cy': float(cy),
            'angle': float(angle),
            'length_px': float(max(rw,rh)),
            'width_px': float(min(rw,rh))
        }

        return display, result

    def run(self):
        while True:
            image = self.camera.get_latest_image(
                agibot_gdk.CameraType.kHandRightColor,
                1000.0
            )

            if image is None:
                continue

            frame = cv2.imdecode(
                np.frombuffer(image.data,np.uint8),
                cv2.IMREAD_COLOR
            )

            if frame is None:
                continue

            display,result = self.detect(frame)

            cv2.imshow('Product Detector V2',display)

            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                break

            if key == ord('s') and result:
                print(result)

        cv2.destroyAllWindows()


def main():
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print('GDK初始化失败')
        return

    try:
        ProductDetectorV2().run()
    finally:
        agibot_gdk.gdk_release()

if __name__ == '__main__':
    main()
