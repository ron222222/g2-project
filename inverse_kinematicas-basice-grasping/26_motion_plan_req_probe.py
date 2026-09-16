#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
26_motion_plan_req_probe.py
READ ONLY

目标:
1. 解码 MotionPlanReq
2. 解码 MotionPlanType
3. 解码 PlanningGroup
4. 解码 PlanningTarget
5. 解码 PlanningTargetType
6. 尝试构造 MotionPlanReq()
7. 不调用 motion_plan_request()
8. 不发送任何运动命令
"""

import io
import contextlib
import agibot_gdk

def dump_type(type_name):
    print()
    print('='*70)
    print('TYPE:', type_name)
    print('='*70)

    if not hasattr(agibot_gdk, type_name):
        print('NOT FOUND')
        return

    cls = getattr(agibot_gdk, type_name)

    print('DOCSTRING:')
    print(getattr(cls,'__doc__',None))

    print()
    print('MEMBERS:')
    for item in dir(cls):
        if item.startswith('_'):
            continue
        try:
            print(f'{item} = {getattr(cls,item)}')
        except Exception as e:
            print(f'{item} = ERROR: {e}')

    print()
    print('HELP:')
    buf=io.StringIO()
    with contextlib.redirect_stdout(buf):
        help(cls)
    print(buf.getvalue())

print('============================================================')
print('26_motion_plan_req_probe.py')
print('READ ONLY MODE')
print('============================================================')

for name in [
    'MotionPlanReq',
    'MotionPlanType',
    'PlanningGroup',
    'PlanningTarget',
    'PlanningTargetType'
]:
    dump_type(name)

print()
print('============================================================')
print('MotionPlanReq Construction Test')
print('============================================================')

if hasattr(agibot_gdk,'MotionPlanReq'):
    try:
        req = agibot_gdk.MotionPlanReq()
        print('CONSTRUCT OK')
        for item in dir(req):
            if item.startswith('_'):
                continue
            try:
                print(f'{item} = {getattr(req,item)}')
            except Exception:
                pass
    except Exception as e:
        print('CONSTRUCT FAIL:', e)
