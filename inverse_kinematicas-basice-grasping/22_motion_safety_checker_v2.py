#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
22_motion_safety_checker_v2.py
READ ONLY
展开检查 status.wrenches 结构
不发送任何运动指令
"""
import time
import agibot_gdk

REFRESH_SEC = 1.0


def dump_wrench(idx, wrench):
    print()
    print(f'WRENCH[{idx}]')
    print(f'type = {type(wrench)}')

    attrs = [a for a in dir(wrench) if not a.startswith('_')]
    print('attributes:')
    print(attrs)

    for name in attrs:
        try:
            value = getattr(wrench, name)
            print(f'  {name} = {value}')
        except Exception as e:
            print(f'  {name} = <ERROR:{e}>')


def main():
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print('GDK初始化失败')
        return

    robot = agibot_gdk.Robot()
    time.sleep(2)

    print('=' * 70)
    print('22_motion_safety_checker_v2.py')
    print('READ ONLY MODE')
    print('展开 Wrench 结构')
    print('=' * 70)

    try:
        while True:
            status = robot.get_motion_control_status()

            print()
            print('-' * 70)
            print('Motion Status')
            print(f'mode = {status.mode}')
            print(f'error_code = {status.error_code}')
            print(f'error_msg = {status.error_msg}')
            print(f'frame_count = {len(status.frame_names)}')
            print(f'wrench_count = {len(status.wrenches)}')

            for i, frame in enumerate(status.frame_names):
                print(f'frame[{i}] = {frame}')

            for i, wrench in enumerate(status.wrenches):
                dump_wrench(i, wrench)

            time.sleep(REFRESH_SEC)

    except KeyboardInterrupt:
        print()
        print('用户退出')
    finally:
        try:
            agibot_gdk.gdk_release()
        except Exception:
            pass

if __name__ == '__main__':
    main()
