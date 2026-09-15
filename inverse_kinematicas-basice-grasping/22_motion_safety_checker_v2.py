#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
22_motion_safety_checker.py
READ ONLY

用途：
- 读取 G2 Motion Control Status
- 显示 mode
- 显示 error_code / error_msg
- 显示 collision_pairs
- 显示 wrenches 数量

不发送运动指令
不控制机械臂
不控制夹爪

Ctrl+C退出
"""

import time
import agibot_gdk

REFRESH_SEC = 1.0


def main():
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print('GDK初始化失败')
        return

    try:
        robot = agibot_gdk.Robot()
        time.sleep(2)

        print('=' * 70)
        print('22_motion_safety_checker.py')
        print('READ ONLY MODE')
        print('读取运动状态 / 力力矩信息 / 碰撞信息')
        print('=' * 70)

        while True:
            try:
                status = robot.get_motion_control_status()

                print()
                print('-' * 70)
                print('Motion Status')

                print(f'mode        : {status.mode}')
                print(f'error_code  : {status.error_code}')
                print(f'error_msg   : {status.error_msg}')

                collision_count = len(status.collision_pairs_1)
                print(f'collision_count : {collision_count}')

                if collision_count > 0:
                    print('Collision Pairs:')
                    for a, b in zip(status.collision_pairs_1,
                                    status.collision_pairs_2):
                        print(f'  {a} <-> {b}')

                print(f'frame_count : {len(status.frame_names)}')
                print(f'wrench_count: {len(status.wrenches)}')

                for i, frame in enumerate(status.frame_names[:10]):
                    print(f'frame[{i}] = {frame}')

            except Exception as e:
                print(f'读取 Motion Status 失败: {e}')

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
