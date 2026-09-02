#!/usr/bin/env python3
import agibot_gdk
import time

agibot_gdk.gdk_init()
robot = agibot_gdk.Robot()
time.sleep(2)

# 测试1: 查看 joint_states 中是否有 effort 数据
joint_states = robot.get_joint_states()
print("=== joint_states 中的力矩数据 ===")
for state in joint_states['states']:
    name = state.get('name', 'unknown')
    effort = state.get('effort', 'N/A')
    print(f"  {name}: effort={effort}")

# 测试2: 查看 motion_control_status 中是否有 wrenches
try:
    status = robot.get_motion_control_status()
    print("\n=== motion_control_status 中的力矩数据 ===")
    if hasattr(status, 'wrenches'):
        print(f"  wrenches: {status.wrenches}")
    else:
        print("  status 对象没有 wrenches 属性")
    print(f"  status 所有属性: {dir(status)}")
except Exception as e:
    print(f"  get_motion_control_status 失败: {e}")

agibot_gdk.gdk_release()