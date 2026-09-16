#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
25_motion_control_mode_probe.py
READ ONLY

目标:
1. 枚举 agibot_gdk 模块公开成员
2. 探测 MotionControlMode
3. 输出 MotionControlMode 文档、help、枚举值
4. 输出与控制模式相关的类
5. 不发送任何运动命令
"""

import io
import contextlib
import agibot_gdk

print('='*80)
print('25_motion_control_mode_probe.py')
print('READ ONLY MODE')
print('='*80)

print('agibot_gdk公开成员:')
for name in sorted(dir(agibot_gdk)):
    if not name.startswith('_'):
        print(name)

for cls_name in ['MotionControlMode','ControlMode']:
    print(''+'-'*80)
    print('CLASS:', cls_name)
    if not hasattr(agibot_gdk, cls_name):
        print('NOT FOUND')
        continue

    cls = getattr(agibot_gdk, cls_name)

    print('DOCSTRING:')
    print(getattr(cls,'__doc__',None))

    print('ENUM VALUES:')
    for item in dir(cls):
        if item.startswith('_'):
            continue
        try:
            print(f'{item} = {getattr(cls,item)}')
        except Exception as e:
            print(f'{item} = <ERROR {e}>')

    print('HELP:')
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        help(cls)
    print(buf.getvalue())

print('关键词筛选:')
keys=['STOP','IDLE','SERVO','PLAN','SAFE','DISABLE','ENABLE','MOTION']
for obj_name in dir(agibot_gdk):
    up=obj_name.upper()
    if any(k in up for k in keys):
        print(obj_name)
