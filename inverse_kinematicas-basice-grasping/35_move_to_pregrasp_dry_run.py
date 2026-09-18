#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
35_move_to_pregrasp_dry_run.py

READ ONLY
不发送任何运动命令

读取 pregrasp_plan.json
检查目标距离、工作空间范围
输出 Dry Run 结果
"""
import json
from pathlib import Path
import numpy as np

PLAN='pregrasp_plan.json'

if not Path(PLAN).exists():
    raise RuntimeError('未找到 pregrasp_plan.json，请先运行34_yolo_pregrasp_generator.py')

plan=json.loads(Path(PLAN).read_text(encoding='utf-8'))

pre=np.array(plan['pregrasp_xyz'],dtype=float)
arm=np.array(plan['arm_r_xyz'],dtype=float)
delta=np.array(plan['delta_xyz'],dtype=float)
dist=float(plan['distance_m'])

# 保守工作空间检查(仅提示)
workspace_ok=(0.20<pre[0]<1.50 and -0.80<pre[1]<0.80 and 0.20<pre[2]<1.50)
reach_ok=dist<1.0

result={
 'dry_run':True,
 'workspace_check':'PASS' if workspace_ok else 'FAIL',
 'reachability_check':'PASS' if reach_ok else 'FAIL',
 'target_pregrasp_xyz':plan['pregrasp_xyz'],
 'current_arm_xyz':plan['arm_r_xyz'],
 'delta_xyz':plan['delta_xyz'],
 'distance_m':dist,
 'motion_command_sent':False
}

Path('dry_run_report.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
