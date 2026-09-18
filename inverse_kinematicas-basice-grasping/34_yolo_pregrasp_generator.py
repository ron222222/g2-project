#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
34_yolo_pregrasp_generator.py
READ ONLY

读取 product_xyz_v3.json 或 product_xyz_v2.json
读取右臂当前位姿(arm_r_end_link)
生成 PreGrasp XYZ (目标上方150mm)
计算 Delta XYZ 与 Distance
保存 pregrasp_plan.json
不发送任何机器人运动命令
"""
import json,time
from pathlib import Path
import numpy as np
import agibot_gdk

INPUTS=['product_xyz_v3.json','product_xyz_v2.json']
OUTPUT='pregrasp_plan.json'
PREGRASP_OFFSET_Z=0.150
FRAME='arm_r_end_link'

src=None
for f in INPUTS:
    if Path(f).exists():
        src=f
        break
if src is None:
    raise RuntimeError('未找到 product_xyz_v3.json 或 product_xyz_v2.json')

product=json.loads(Path(src).read_text(encoding='utf-8'))
base_xyz=np.array(product['base_xyz'],dtype=float)
pregrasp=base_xyz.copy()
pregrasp[2]+=PREGRASP_OFFSET_Z

if agibot_gdk.gdk_init()!=agibot_gdk.GDKRes.kSuccess:
    raise RuntimeError('GDK init failed')

try:
    tf_api=agibot_gdk.TF()
    time.sleep(2)
    t=tf_api.get_tf_from_base_link(FRAME)
    arm=np.array([t.translation.x,t.translation.y,t.translation.z],dtype=float)
    delta=pregrasp-arm
    dist=float(np.linalg.norm(delta))

    result={
        'source_file':src,
        'product_base_xyz':[round(float(v),4) for v in base_xyz],
        'pregrasp_xyz':[round(float(v),4) for v in pregrasp],
        'arm_r_xyz':[round(float(v),4) for v in arm],
        'delta_xyz':[round(float(v),4) for v in delta],
        'distance_m':round(dist,4),
        'offset_z_m':PREGRASP_OFFSET_Z,
        'read_only':True
    }
    Path(OUTPUT).write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))
finally:
    try:
        agibot_gdk.gdk_release()
    except Exception:
        pass
