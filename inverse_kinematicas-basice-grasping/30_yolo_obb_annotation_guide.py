#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
30_yolo_obb_annotation_guide.py
READ ONLY

生成 YOLO OBB 标注指南文件。
不会训练模型，不控制机器人。
"""
from pathlib import Path

content = '''
# YOLO OBB 标注指南（Product）

类别定义：

0 = product

## 数据集结构

dataset/
├── images/
│   ├── train/
│   └── val/
├── labels/
│   ├── train/
│   └── val/
└── dataset.yaml

## OBB 标注格式

每张图片对应一个 txt：

class x1 y1 x2 y2 x3 y3 x4 y4

说明：
- 四个点按顺序围绕产品外轮廓。
- 坐标需归一化到 0~1。
- 仅 1 类：product。

示例：
0 0.45 0.40 0.60 0.42 0.59 0.50 0.44 0.48

## 标注原则

1. 只框产品。
2. 不包含桌面。
3. 不包含机械臂。
4. 不包含线缆。
5. 产品倾斜时使用旋转框。
6. 无产品图片保留空标签文件。

## dataset.yaml

path: dataset
train: images/train
val: images/val
names:
  0: product

## 推荐训练命令

yolo obb train model=yolo11n-obb.pt data=dataset/dataset.yaml imgsz=640 epochs=100 batch=8

训练完成后获得：

runs/obb/train/weights/best.pt

后续步骤：
31_yolo_obb_train.py
32_yolo_obb_product_detector_live.py
'''

out = Path('30_yolo_obb_annotation_guide.md')
out.write_text(content, encoding='utf-8')
print(out)
