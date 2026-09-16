#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
25_safe_mode_probe.py
READ ONLY

目标:
1. 探测 SafeMode
2. 探测 SAFE_NORMAL / SAFE_REDUCED / SAFE_STOP
3. 探测 MotionControlMode对象结构
4. 验证 MotionControlMode.safe_mode 字段可写性
5. 不发送任何运动命令
"""

import io
import contextlib
import agibot_gdk

print('='*80)
print('25_safe_mode_probe.py')
print('READ ONLY MODE')
print('='*80)

for name in ['SafeMode']:
    print('' + '-'*80)
    print('CLASS:', name)
    if not hasattr(agibot_gdk, name):
        print('NOT FOUND')
        continue

    cls = getattr(agibot_gdk, name)

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
    buf=io.StringIO()
    with contextlib.redirect_stdout(buf):
        help(cls)
    print(buf.getvalue())

print('' + '-'*80)
print('SAFE 常量')
for item in ['SAFE_NORMAL','SAFE_REDUCED','SAFE_STOP']:
    if hasattr(agibot_gdk,item):
        print(item,'=',getattr(agibot_gdk,item))

print('' + '-'*80)
print('MotionControlMode 结构验证')
m=agibot_gdk.MotionControlMode()
print('control_mode =', getattr(m,'control_mode',None))
print('input_source =', getattr(m,'input_source',None))
print('priority =', getattr(m,'priority',None))
print('safe_mode =', getattr(m,'safe_mode',None))
print('target =', getattr(m,'target',None))

for item in ['SAFE_NORMAL','SAFE_REDUCED','SAFE_STOP']:
    if hasattr(agibot_gdk,item):
        try:
            m.safe_mode = getattr(agibot_gdk,item)
            print('SET OK:', item, '=>', m.safe_mode)
        except Exception as e:
            print('SET FAIL:', item, e)
