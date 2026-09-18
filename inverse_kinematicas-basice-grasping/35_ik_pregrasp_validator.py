#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
35_ik_pregrasp_validator.py
READ ONLY

读取 pregrasp_plan.json
执行 IK 前检查（不发送运动命令）
输出 ik_validation_report.json

说明：由于不同 G2 SDK 的 IK API 名称可能不同，
本版本主要完成目标位姿、可达性、工作空间、距离验证。
若SDK提供IK接口，可在标记位置接入。
"""
import json
from pathlib import Path
import numpy as np

PLAN='pregrasp_plan.json'
REPORT='ik_validation_report.json'

if not Path(PLAN).exists():
    raise RuntimeError('未找到 pregrasp_plan.json')

plan=json.loads(Path(PLAN).read_text(encoding='utf-8'))

pre=np.array(plan['pregrasp_xyz'],dtype=float)
cur=np.array(plan['arm_r_xyz'],dtype=float)
delta=np.array(plan['delta_xyz'],dtype=float)
dist=float(plan['distance_m'])

workspace_ok=(0.20<pre[0]<1.50 and -0.80<pre[1]<0.80 and 0.20<pre[2]<1.50)
reach_ok=dist<1.0

ik_checked=False
ik_pass='UNKNOWN'
ik_message='SDK IK接口未接入，仅完成Dry Validation'

result={
 'read_only':True,
 'target_pregrasp_xyz':[round(float(v),4) for v in pre],
 'current_arm_xyz':[round(float(v),4) for v in cur],
 'delta_xyz':[round(float(v),4) for v in delta],
 'distance_m':round(dist,4),
 'workspace_check':'PASS' if workspace_ok else 'FAIL',
 'reachability_check':'PASS' if reach_ok else 'FAIL',
 'ik_checked':ik_checked,
 'ik_result':ik_pass,
 'ik_message':ik_message,
 'motion_command_sent':False
}

Path(REPORT).write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
