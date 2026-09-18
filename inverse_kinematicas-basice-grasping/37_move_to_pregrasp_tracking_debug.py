#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
37_move_to_pregrasp_tracking_debug.py
READ ONLY

用于分析 end_effector_pose_control 跟踪情况。
不发送运动命令。
每秒打印:
- Target XYZ
- Current XYZ
- Error
- Error变化
"""
import json,time,math
import agibot_gdk

TARGET_FILE='pregrasp_plan.json'
RIGHT_FRAME='arm_r_end_link'

with open(TARGET_FILE,'r',encoding='utf-8') as f:
    target=json.load(f)['pregrasp_xyz']

def dist(a,b):
    return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))

agibot_gdk.gdk_init()
tf=agibot_gdk.TF()

last=None
print('Tracking Debug Start')
print('Target=',target)

try:
    step=0
    while True:
        t=tf.get_tf_from_base_link(RIGHT_FRAME)
        cur=[float(t.translation.x),float(t.translation.y),float(t.translation.z)]
        err=dist(cur,target)

        if last is None:
            derr=0.0
        else:
            derr=last-err

        print(f'[STEP {step}]')
        print('Current=',[round(v,4) for v in cur])
        print('Target =',[round(v,4) for v in target])
        print(f'Error={err:.4f} m')
        print(f'DeltaError={derr:.4f} m')
        print('-'*60)

        last=err
        step+=1
        time.sleep(1.0)
except KeyboardInterrupt:
    pass
finally:
    agibot_gdk.gdk_release()
