#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
21_pose_debugger.py
G2 位姿调试器（READ ONLY）

功能：
- 点击目标
- 显示 Camera XYZ
- 显示 Base XYZ
- 显示 arm_r_end_link 当前坐标
- 显示目标与右手当前位置差值
- 生成 PreGrasp（目标上方150mm）

不发送任何运动指令
不控制夹爪
不控制机械臂
"""
import json
import numpy as np

print('='*70)
print('21_pose_debugger.py')
print('READ ONLY MODE')
print('用于分析 Base XYZ 与右手当前位置差值')
print('='*70)

# 使用方式说明
example = {
    'Target_Base_XYZ':[0.830,-0.116,0.717],
    'Current_Right_Hand':[0.589,-0.243,0.816]
}

T=np.array(example['Target_Base_XYZ'])
H=np.array(example['Current_Right_Hand'])
D=T-H
pregrasp=T.copy()
pregrasp[2]+=0.150

print('
示例计算：')
print('Target Base XYZ =',T.tolist())
print('Current Hand XYZ =',H.tolist())
print('DX,DY,DZ =',D.tolist())
print('Distance = %.3f m' % np.linalg.norm(D))
print('PreGrasp XYZ =',pregrasp.tolist())

print('
后续将把已验证的19_v3视觉链路接入此调试器：')
print('RGB Pixel -> Camera XYZ -> Base XYZ -> PreGrasp XYZ')
