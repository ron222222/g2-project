#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
27_auto_product_center_detector_live_v5.py
READ ONLY

V5功能：
1. 使用 get_latest_image() 读取 G2 头部彩色相机。
2. 固定产品工作区 ROI。
3. HSV 黑色目标提取。
4. 面积、长边、短边和宽松长宽比过滤。
5. 使用 minAreaRect 支持横向、倾斜和接近竖直放置。
6. 输出中心像素和产品方向角。
7. 保存 product_center_live_v5.json 和标注图。
8. 不控制机器人，不发送运动命令。
"""

import json
import time
from pathlib import Path

import cv2
import numpy as np
import agibot_gdk

SAVE_JSON = "product_center_live_v5.json"
SAVE_IMG = "product_center_live_v5.png"
WINDOW_NAME = "Product Detector V5"

WORK_X_MIN = 260
WORK_X_MAX = 460
WORK_Y_MIN = 90
WORK_Y_MAX = 230

HSV_LOWER = np.array([0, 0, 0], dtype=np.uint8)
HSV_UPPER = np.array([180, 255, 100], dtype=np.uint8)

MIN_AREA = 100.0
MIN_LONG_SIDE = 45.0
MAX_LONG_SIDE = 190.0
MIN_SHORT_SIDE = 8.0
MAX_SHORT_SIDE = 70.0
MIN_ASPECT_RATIO = 1.25


def normalized_long_axis_angle(rect):
    width, height = rect[1]
    angle = float(rect[2])
    if width < height:
        angle += 90.0
    while angle >= 90.0:
        angle -= 180.0
    while angle < -90.0:
        angle += 180.0
    return angle


def main():
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        raise RuntimeError("GDK初始化失败")

    camera = agibot_gdk.Camera()
    time.sleep(3.0)

    print("27_auto_product_center_detector_live_v5.py")
    print("READ ONLY MODE")
    print("旋转鲁棒识别 + 中心像素 + 方向角")
    print("S 保存，Q或ESC退出")

    try:
        while True:
            try:
                rgb = camera.get_latest_image(
                    agibot_gdk.CameraType.kHeadColor,
                    1000.0,
                )
            except Exception:
                time.sleep(0.05)
                continue

            if rgb is None:
                continue

            frame = cv2.imdecode(
                np.frombuffer(rgb.data, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if frame is None:
                continue

            height, width = frame.shape[:2]
            x1 = max(0, min(WORK_X_MIN, width - 1))
            x2 = max(x1 + 1, min(WORK_X_MAX, width))
            y1 = max(0, min(WORK_Y_MIN, height - 1))
            y2 = max(y1 + 1, min(WORK_Y_MAX, height))

            roi = frame[y1:y2, x1:x2].copy()
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

            kernel = np.ones((3, 3), dtype=np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            best = None
            best_score = -1.0

            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < MIN_AREA:
                    continue

                rect = cv2.minAreaRect(contour)
                rect_width, rect_height = rect[1]
                long_side = max(rect_width, rect_height)
                short_side = max(min(rect_width, rect_height), 1.0)
                aspect_ratio = long_side / short_side

                if not MIN_LONG_SIDE <= long_side <= MAX_LONG_SIDE:
                    continue
                if not MIN_SHORT_SIDE <= short_side <= MAX_SHORT_SIDE:
                    continue
                if aspect_ratio < MIN_ASPECT_RATIO:
                    continue

                score = area * aspect_ratio
                if score > best_score:
                    best_score = score
                    best = (
                        rect,
                        area,
                        long_side,
                        short_side,
                        aspect_ratio,
                    )

            view = frame.copy()
            cv2.rectangle(view, (x1, y1), (x2, y2), (255, 0, 0), 2)

            result = None

            if best is not None:
                rect, area, long_side, short_side, aspect_ratio = best
                center_x = int(round(rect[0][0])) + x1
                center_y = int(round(rect[0][1])) + y1
                angle_deg = normalized_long_axis_angle(rect)

                box = cv2.boxPoints(rect).astype(np.int32)
                box[:, 0] += x1
                box[:, 1] += y1

                cv2.drawContours(view, [box], 0, (0, 255, 0), 2)
                cv2.circle(view, (center_x, center_y), 8, (0, 0, 255), -1)
                cv2.putText(
                    view,
                    f"Center=({center_x},{center_y}) Angle={angle_deg:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2,
                )

                result = {
                    "program": "27_auto_product_center_detector_live_v5.py",
                    "read_only": True,
                    "roi": [x1, y1, x2, y2],
                    "center_pixel": [center_x, center_y],
                    "angle_deg": float(angle_deg),
                    "contour_area": float(area),
                    "long_side_px": float(long_side),
                    "short_side_px": float(short_side),
                    "aspect_ratio": float(aspect_ratio),
                }

            cv2.imshow(WINDOW_NAME, view)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord("q"), ord("Q")):
                break

            if key in (ord("s"), ord("S")):
                if result is None:
                    print("当前未检测到满足V5规则的产品")
                else:
                    cv2.imwrite(SAVE_IMG, view)
                    Path(SAVE_JSON).write_text(
                        json.dumps(result, ensure_ascii=False, indent=2),
                        encoding="utf-8",
                    )
                    print("保存成功")
                    print("center_pixel =", result["center_pixel"])
                    print("angle_deg =", result["angle_deg"])
                    print("long_side_px =", result["long_side_px"])
                    print("short_side_px =", result["short_side_px"])
                    print("aspect_ratio =", result["aspect_ratio"])

    finally:
        cv2.destroyAllWindows()
        try:
            camera.close_camera()
        except Exception:
            pass
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
