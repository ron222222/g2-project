#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
将 83_g2_pick_autorun_height40.py 升级为
84_g2_pick_autorun_fast_yolo.py

修改：
1. REQUIRE_WAIST_CONFIRMATION=False
2. 提高速度
3. 全流程YOLO窗口持续刷新(保留最后识别画面)
4. 显示详细状态信息
5. 不修改抓取高度逻辑
"""
from pathlib import Path
import re

src=Path('83_g2_pick_autorun_height40.py')
dst=Path('84_g2_pick_autorun_fast_yolo.py')
if not src.exists():
    raise RuntimeError(f'缺少源文件: {src}')

s=src.read_text(encoding='utf-8')

s=s.replace('REQUIRE_WAIST_CONFIRMATION = True','REQUIRE_WAIST_CONFIRMATION = False')
s=s.replace('DIRECT_ARM_SPEED_RAD_S = 0.20','DIRECT_ARM_SPEED_RAD_S = 0.30')
s=s.replace('HEAD_SPEED_RAD_S = 0.20','HEAD_SPEED_RAD_S = 0.35')
s=s.replace('WAIST_SPEED_RAD_S = 0.20','WAIST_SPEED_RAD_S = 0.35')
s=s.replace('SETTLE_S = 1.0','SETTLE_S = 0.40')
s=s.replace('ROTATION_FINAL_HOLD_S = 0.45','ROTATION_FINAL_HOLD_S = 0.15')

if 'LAST_YOLO_FRAME = None' not in s:
    s=s.replace('SHOW_YOLO_WINDOW = True',
                'SHOW_YOLO_WINDOW = True\nLAST_YOLO_FRAME = None\nLAST_YOLO_TEXT = []',1)

show_old='''def show_frame(frame,lines):'''
if show_old in s and 'def refresh_live_window(' not in s:
    insert='''\n\ndef refresh_live_window(stage):\n    global LAST_YOLO_FRAME,LAST_YOLO_TEXT\n    if LAST_YOLO_FRAME is None or not SHOW_YOLO_WINDOW:\n        return\n    canvas=LAST_YOLO_FRAME.copy()\n    text=list(LAST_YOLO_TEXT)\n    text.append(f"stage={stage}")\n    y=28\n    for line in text:\n        import cv2\n        cv2.putText(canvas,str(line),(12,y),cv2.FONT_HERSHEY_SIMPLEX,0.65,(0,255,255),2)\n        y+=27\n    try:\n        cv2.imshow(WINDOW_NAME,canvas)\n        cv2.waitKey(1)\n    except Exception:\n        pass\n'''
    pos=s.find('def detect_product(')
    if pos>0:
        s=s[:pos]+insert+s[pos:]
    s=s.replace('canvas=frame.copy(); y=28','global LAST_YOLO_FRAME,LAST_YOLO_TEXT\n    LAST_YOLO_FRAME=frame.copy()\n    LAST_YOLO_TEXT=list(lines)\n    canvas=frame.copy(); y=28',1)

# richer display
s=s.replace('show_frame(annotated,[f"conf={conf:.3f}",f"position={len(pos)}/8",f"angle={len(angles)}"])',
'''show_frame(annotated,[
            f"YOLO conf={conf:.3f}",
            f"position={len(pos)}/{DETECTION_SAMPLES}",
            f"angle={len(angles)}/{MIN_ANGLE_SAMPLES}",
            f"depth={z:.3f}m",
            f"base=({base[0]:.3f},{base[1]:.3f},{base[2]:.3f})"
        ])''')

for token in ['coarse_align_xy_before_orientation','fine_align_xy_after_orientation','descend_pregrasp','descend_grasp','lift_10cm']:
    s=s.replace(f'"{token}"',f'"{token}"')

s+='\n# 84版: Faster motion + Persistent detailed YOLO window\n'

dst.write_text(s,encoding='utf-8')
print(f'Generated: {dst}')
