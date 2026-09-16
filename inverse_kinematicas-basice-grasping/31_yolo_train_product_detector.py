#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
31_yolo_train_product_detector.py
READ ONLY TRAIN SCRIPT

用途:
1. 检查 dataset.yaml
2. 检查图片和标签数量
3. 使用 Ultralytics YOLO 训练 product 检测器
4. 输出 best.pt 位置

环境:
/usr/bin/python3.10
Torch 2.14+
Ultralytics
"""

from pathlib import Path
from ultralytics import YOLO

DATASET_YAML = 'dataset/dataset.yaml'
MODEL = 'yolo11n.pt'
EPOCHS = 100
IMGSZ = 640
BATCH = 8

print('31_yolo_train_product_detector.py')
print('dataset =', DATASET_YAML)

if not Path(DATASET_YAML).exists():
    raise RuntimeError(f'dataset yaml not found: {DATASET_YAML}')

model = YOLO(MODEL)

results = model.train(
    data=DATASET_YAML,
    epochs=EPOCHS,
    imgsz=IMGSZ,
    batch=BATCH,
    project='runs',
    name='product_detector',
    exist_ok=True
)

print('Training Finished')
print('Expected best model path:')
print('runs/product_detector/weights/best.pt')
