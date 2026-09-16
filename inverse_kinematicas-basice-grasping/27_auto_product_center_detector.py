#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
27_auto_product_center_detector.py
DRY_RUN ONLY

功能:
1. 读取RGB图片
2. OpenCV自动检测产品轮廓
3. 找最大轮廓
4. 计算产品中心(cx,cy)
5. 输出中心像素
6. 保存标注图
7. 不控制机器人

用法:
/usr/bin/python3.10 27_auto_product_center_detector.py image.jpg
"""

import sys
import json
from pathlib import Path
import cv2
import numpy as np

if len(sys.argv) < 2:
    print('用法: python3 27_auto_product_center_detector.py image.jpg')
    sys.exit(1)

img_path = Path(sys.argv[1])
img = cv2.imread(str(img_path))
if img is None:
    raise RuntimeError(f'无法读取图片: {img_path}')

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# 通用前景提取，可根据产品颜色后续调整
v = hsv[:,:,2]
_, mask = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

kernel = np.ones((5,5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
if not contours:
    raise RuntimeError('未检测到轮廓')

cnt = max(contours, key=cv2.contourArea)
area = cv2.contourArea(cnt)
M = cv2.moments(cnt)
if M['m00'] == 0:
    raise RuntimeError('轮廓面积为0')

cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])

out = img.copy()
cv2.drawContours(out,[cnt],-1,(0,255,0),2)
cv2.circle(out,(cx,cy),8,(0,0,255),-1)
cv2.line(out,(cx-20,cy),(cx+20,cy),(255,0,0),2)
cv2.line(out,(cx,cy-20),(cx,cy+20),(255,0,0),2)

png_file = img_path.with_name(img_path.stem + '_detected.png')
json_file = img_path.with_name(img_path.stem + '_center.json')

cv2.imwrite(str(png_file), out)

result = {
    'image': str(img_path),
    'center_pixel':[cx,cy],
    'contour_area': float(area),
    'annotated_image': str(png_file)
}

json_file.write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')

print('产品中心检测完成')
print('center_pixel =', [cx,cy])
print('contour_area =', area)
print('annotated =', png_file)
print('json =', json_file)
