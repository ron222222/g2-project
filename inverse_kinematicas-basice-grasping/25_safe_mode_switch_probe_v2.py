#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
25_safe_mode_switch_probe_v2.py
READ ONLY

目的:
1. 读取当前 MotionControlStatus
2. 构造 MotionControlMode
3. 尝试切换 SAFE_STOP
4. 读取状态变化
5. 恢复 SAFE_NORMAL
6. 不发送任何关节/末端运动命令

注意:
仅测试 set_control_mode() 行为。
"""

import time
import agibot_gdk


def dump_status(robot, title):
    print()
    print(title)
    try:
        s = robot.get_motion_control_status()
        print(f'mode={getattr(s, "mode", None)}')
        print(f'error_code={getattr(s, "error_code", None)}')
        print(f'error_msg={getattr(s, "error_msg", None)}')
    except Exception as e:
        print('status read failed:', e)


if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    raise RuntimeError('GDK初始化失败')

try:
    robot = agibot_gdk.Robot()
    time.sleep(2)

    print('============================================================')
    print('25_safe_mode_switch_probe_v2.py')
    print('READ ONLY TEST')
    print('============================================================')

    dump_status(robot, '当前状态')

    mode = agibot_gdk.MotionControlMode()

    print()
    print('默认MotionControlMode')
    print('control_mode =', mode.control_mode)
    print('input_source =', mode.input_source)
    print('priority =', mode.priority)
    print('safe_mode =', mode.safe_mode)
    print('target =', mode.target)

    print()
    print('切换 SAFE_STOP')
    mode.safe_mode = agibot_gdk.SAFE_STOP
    ret = robot.set_control_mode(mode)
    print('set_control_mode return =', ret)

    time.sleep(1.0)
    dump_status(robot, 'SAFE_STOP后状态')

    print()
    print('恢复 SAFE_NORMAL')
    mode.safe_mode = agibot_gdk.SAFE_NORMAL
    ret = robot.set_control_mode(mode)
    print('set_control_mode return =', ret)

    time.sleep(1.0)
    dump_status(robot, '恢复后状态')

finally:
    try:
        agibot_gdk.gdk_release()
    except Exception:
        pass
