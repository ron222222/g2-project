#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
25_motion_control_mode_dump.py
READ ONLY

目标:
1. 深度解码 MotionControlMode
2. 解码 ControlMode
3. 解码 InputSource
4. 解码 Target
5. 解码 SafeMode
6. 构造 MotionControlMode 并测试字段赋值
7. 不调用 set_control_mode()
8. 不发送任何运动命令
"""

import io
import contextlib
import agibot_gdk


def dump_enum(enum_name):
    print()
    print('=' * 70)
    print('ENUM:', enum_name)
    print('=' * 70)

    if not hasattr(agibot_gdk, enum_name):
        print('NOT FOUND')
        return

    cls = getattr(agibot_gdk, enum_name)

    print('DOCSTRING:')
    print(getattr(cls, '__doc__', None))

    print()
    print('VALUES:')
    for item in dir(cls):
        if item.startswith('_'):
            continue
        try:
            value = getattr(cls, item)
            print(f'{item} = {value}')
        except Exception as e:
            print(f'{item} = ERROR: {e}')

    print()
    print('HELP:')
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        help(cls)
    print(buf.getvalue())


print('============================================================')
print('25_motion_control_mode_dump.py')
print('READ ONLY MODE')
print('============================================================')

dump_enum('ControlMode')
dump_enum('InputSource')
dump_enum('Target')
dump_enum('SafeMode')

print()
print('============================================================')
print('MotionControlMode Instance Dump')
print('============================================================')

m = agibot_gdk.MotionControlMode()

print('control_mode =', m.control_mode)
print('input_source =', m.input_source)
print('priority =', m.priority)
print('safe_mode =', m.safe_mode)
print('target =', m.target)

print()
print('Field Assignment Test')

for field,value in [
    ('safe_mode', agibot_gdk.SAFE_STOP),
    ('safe_mode', agibot_gdk.SAFE_REDUCED),
    ('safe_mode', agibot_gdk.SAFE_NORMAL),
]:
    try:
        setattr(m, field, value)
        print('OK', field, '=', getattr(m, field))
    except Exception as e:
        print('FAIL', field, e)
