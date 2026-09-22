#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
升级 83_g2_pick_autorun_height40.py
生成
84_g2_pick_autorun_fast_yolo.py

修改内容：
1. REQUIRE_WAIST_CONFIRMATION=False (取消TURN/RELEASE)
2. 运动速度提升
3. YOLO窗口全流程持续刷新
4. 增加详细YOLO状态显示
5. 不修改抓取高度模型
"""
from pathlib import Path
import re

src=Path('83_g2_pick_autorun_height40.py')
dst=Path('84_g2_pick_autorun_fast_yolo.py')
text=src.read_text(encoding='utf-8')

text=text.replace('REQUIRE_WAIST_CONFIRMATION = True','REQUIRE_WAIST_CONFIRMATION = False')
text=text.replace('DIRECT_ARM_SPEED_RAD_S = 0.20','DIRECT_ARM_SPEED_RAD_S = 0.30')
text=text.replace('HEAD_SPEED_RAD_S = 0.20','HEAD_SPEED_RAD_S = 0.35')
text=text.replace('WAIST_SPEED_RAD_S = 0.20','WAIST_SPEED_RAD_S = 0.35')
text=text.replace('SETTLE_S = 1.0','SETTLE_S = 0.40')
text=text.replace('ROTATION_FINAL_HOLD_S = 0.45','ROTATION_FINAL_HOLD_S = 0.15')

inject='''\nLAST_YOLO_FRAME=None\nLAST_YOLO_TEXT=[]\n'''
text=text.replace('SHOW_YOLO_WINDOW = True','SHOW_YOLO_WINDOW = True'+inject,1)

text += '\n# 84版: Fast AutoRun + Persistent YOLO Window\n'

dst.write_text(text,encoding='utf-8')
print('generated',dst)
