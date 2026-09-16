#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
29_yolo_obb_dataset_prepare.py
READ ONLY

用途:
1. 检查 dataset/images 中图片数量
2. 自动生成 train/val 划分(80/20)
3. 创建 YOLO OBB 数据集目录结构
4. 创建空 labels/train 与 labels/val
5. 生成 dataset.yaml
6. 不训练模型
7. 不控制机器人

运行:
/usr/bin/python3.10 29_yolo_obb_dataset_prepare.py
"""

from pathlib import Path
import random
import shutil
import yaml

ROOT = Path('dataset')
IMAGES = ROOT / 'images'
TRAIN_IMG = ROOT / 'images' / 'train'
VAL_IMG = ROOT / 'images' / 'val'
TRAIN_LBL = ROOT / 'labels' / 'train'
VAL_LBL = ROOT / 'labels' / 'val'

for d in [TRAIN_IMG, VAL_IMG, TRAIN_LBL, VAL_LBL]:
    d.mkdir(parents=True, exist_ok=True)

imgs = []
for ext in ('*.jpg','*.png','*.jpeg'):
    imgs.extend(IMAGES.glob(ext))

imgs = sorted(set(imgs))
if len(imgs)==0:
    raise RuntimeError('dataset/images 下没有图片')

random.seed(42)
random.shuffle(imgs)

split = max(1, int(len(imgs)*0.8))
train_imgs = imgs[:split]
val_imgs = imgs[split:]
if len(val_imgs)==0:
    val_imgs = train_imgs[-1:]

for f in train_imgs:
    target = TRAIN_IMG / f.name
    if not target.exists():
        shutil.copy2(f, target)

for f in val_imgs:
    target = VAL_IMG / f.name
    if not target.exists():
        shutil.copy2(f, target)

cfg = {
    'path': str(ROOT.resolve()),
    'train': 'images/train',
    'val': 'images/val',
    'names': {0:'product'}
}

yaml_file = ROOT / 'dataset.yaml'
with open(yaml_file,'w',encoding='utf-8') as fp:
    yaml.safe_dump(cfg, fp, allow_unicode=True, sort_keys=False)

print('YOLO OBB 数据集准备完成')
print('total_images =', len(imgs))
print('train_images =', len(train_imgs))
print('val_images =', len(val_imgs))
print('dataset_yaml =', yaml_file)
print('下一步: 使用 labelImg/roboflow 对 product 做 OBB 标注')
