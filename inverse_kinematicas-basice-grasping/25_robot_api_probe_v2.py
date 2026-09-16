#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
25_robot_api_probe_v2.py
READ ONLY

目标：
1. 枚举 Robot 全部公开接口
2. 输出 __doc__
3. 输出 help() 文本
4. 重点分析运动相关接口
5. 不发送任何运动命令
"""

import io
import time
import inspect
import contextlib
import agibot_gdk

KEY_APIS = [
    'end_effector_pose_control',
    'move_ee_pos',
    'move_arm_joint',
    'move_arm_joint_servo',
    'joint_servo_control',
    'servo_control_arm_pos',
    'trajectory_tracking_control',
    'motion_plan_request',
    'set_control_mode',
    'force_position_control',
]

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    raise RuntimeError('GDK初始化失败')

try:
    robot = agibot_gdk.Robot()
    time.sleep(2)

    print('='*80)
    print('25_robot_api_probe_v2.py')
    print('READ ONLY MODE')
    print('='*80)

    for api_name in KEY_APIS:
        print() 
        print('-'*80)
        print(f'API: {api_name}')

        if not hasattr(robot, api_name):
            print('NOT FOUND')
            continue

        obj = getattr(robot, api_name)

        try:
            print('CALLABLE =', callable(obj))
        except Exception:
            pass

        try:
            print('DOCSTRING:')
            print(obj.__doc__)
        except Exception as e:
            print('DOCSTRING ERROR:', e)

        try:
            print('SIGNATURE:')
            print(inspect.signature(obj))
        except Exception as e:
            print('SIGNATURE ERROR:', e)

        try:
            print('HELP:')
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                help(obj)
            print(buf.getvalue())
        except Exception as e:
            print('HELP ERROR:', e)

finally:
    agibot_gdk.gdk_release()
