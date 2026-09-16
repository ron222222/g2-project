#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
25_robot_api_probe.py
READ ONLY

用途：
1. 枚举 agibot_gdk.Robot() 所有公开接口
2. 重点查找 stop / halt / abort / cancel / servo / motion 相关接口
3. 不发送任何运动命令
4. 不控制机械臂
5. 不控制夹爪
"""

import inspect
import time
import agibot_gdk

KEYWORDS = [
    'stop','halt','abort','cancel','servo','motion',
    'enable','disable','arm','end_effector','gripper',
    'pause','resume','emergency'
]

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    raise RuntimeError('GDK初始化失败')

try:
    robot = agibot_gdk.Robot()
    time.sleep(2)

    print('='*80)
    print('25_robot_api_probe.py')
    print('READ ONLY MODE')
    print('='*80)

    methods=[]
    for name in dir(robot):
        if name.startswith('_'):
            continue
        try:
            obj=getattr(robot,name)
            methods.append((name, callable(obj)))
        except Exception:
            pass

    print('全部公开成员:')
    for name,is_callable in methods:
        print(f'[{"FUNC" if is_callable else "ATTR"}] {name}')

    print('重点接口筛选:')
    for name,is_callable in methods:
        lower=name.lower()
        if any(k in lower for k in KEYWORDS):
            print(f'[{"FUNC" if is_callable else "ATTR"}] {name}')
            try:
                obj=getattr(robot,name)
                if callable(obj):
                    sig='signature unavailable'
                    try:
                        sig=str(inspect.signature(obj))
                    except Exception:
                        pass
                    print('   ',sig)
            except Exception:
                pass
finally:
    agibot_gdk.gdk_release()
