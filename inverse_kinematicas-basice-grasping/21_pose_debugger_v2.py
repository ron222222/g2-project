#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
21_pose_debugger_v2.py
READ ONLY 调试器

目标：
显示和记录：
RGB Pixel -> Camera XYZ -> Base XYZ -> Right Hand XYZ -> PreGrasp XYZ

不发送任何运动指令。
不调用机械臂运动接口。

使用方法：
1. 先运行 19_camera_xyz_to_base_link_v3.py 验证视觉。
2. 将实际测得的 Base XYZ 填入 TARGET_BASE_XYZ。
3. 将 arm_r_end_link 最新 TF 填入 CURRENT_HAND_XYZ。
4. 观察 DX/DY/DZ 与 PreGrasp。
"""

import math

TARGET_BASE_XYZ = [0.830, -0.116, 0.717]
CURRENT_HAND_XYZ = [0.589, -0.243, 0.816]
PREGRASP_OFFSET_Z = 0.150


def main():
    tx, ty, tz = TARGET_BASE_XYZ
    hx, hy, hz = CURRENT_HAND_XYZ

    dx = tx - hx
    dy = ty - hy
    dz = tz - hz

    distance = math.sqrt(dx*dx + dy*dy + dz*dz)

    pregrasp = [tx, ty, tz + PREGRASP_OFFSET_Z]

    print('='*70)
    print('21_pose_debugger_v2.py')
    print('READ ONLY MODE')
    print('='*70)

    print('
视觉目标(Base XYZ)')
    print(f'X={tx:.3f} Y={ty:.3f} Z={tz:.3f}')

    print('
右手当前位置(arm_r_end_link)')
    print(f'X={hx:.3f} Y={hy:.3f} Z={hz:.3f}')

    print('
差值')
    print(f'DX={dx:.3f}')
    print(f'DY={dy:.3f}')
    print(f'DZ={dz:.3f}')
    print(f'Distance={distance:.3f} m')

    print('
预抓取点 PreGrasp')
    print(f'X={pregrasp[0]:.3f}')
    print(f'Y={pregrasp[1]:.3f}')
    print(f'Z={pregrasp[2]:.3f}')

    print('
后续阶段：')
    print('22_motion_safety_checker.py')
    print('读取 collision_pairs / wrenches / error_code')
    print('验证力矩保护后再进入运动控制。')

if __name__ == '__main__':
    main()
