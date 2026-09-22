#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
基于75_g2_pick_waist_right80_angle_fix.py生成：
76_g2_pick_adaptive_vertical_pose.py

修正垂直姿态阶段频繁因轻微超限而停止：
- 阶段容差：TCP 25->35 mm，姿态 6->8 deg
- 最终垂直误差：4->8 deg
- 每段变化：7.5->5 deg
- 单次轻微超限不立即退出；连续2段超限才停止
- 硬限制：TCP 50 mm、姿态12 deg，触发立即停止
- 记录最佳垂直姿态和工作区诊断
本生成器不控制机器人。
"""
from pathlib import Path

SOURCE='75_g2_pick_waist_right80_angle_fix.py'
OUTPUT='76_g2_pick_adaptive_vertical_pose.py'
root=Path(__file__).resolve().parent
src=root/SOURCE
dst=root/OUTPUT
if not src.exists():
    raise RuntimeError(f'缺少源程序: {src}')
s=src.read_text(encoding='utf-8')
s=s.replace(SOURCE,OUTPUT)
s=s.replace('g2_pick_waist_right80_angle_fix_report.json','g2_pick_adaptive_vertical_pose_report.json')
s=s.replace('g2_pick_waist_right80_angle_fix_live.jpg','g2_pick_adaptive_vertical_pose_live.jpg')
s=s.replace('g2_pick_waist_right80_angle_fix_angle.jpg','g2_pick_adaptive_vertical_pose_angle.jpg')

# Existing constants.
for old,new in [
    ('MAX_YAW_PER_STAGE_DEG = 7.5','MAX_YAW_PER_STAGE_DEG = 5.0'),
    ('MAX_ROTATION_STAGE_DRIFT_M = 0.025','MAX_ROTATION_STAGE_DRIFT_M = 0.035'),
    ('VERTICAL_TOOL_TILT_TOLERANCE_DEG = 4.0','VERTICAL_TOOL_TILT_TOLERANCE_DEG = 8.0'),
]:
    if old not in s: raise RuntimeError(f'未找到参数: {old}')
    s=s.replace(old,new,1)

# Add adaptive constants after stage drift.
anchor='MAX_ROTATION_STAGE_DRIFT_M = 0.035\n'
insert='''MAX_ROTATION_STAGE_DRIFT_M = 0.035
VERTICAL_STAGE_ORIENTATION_TOLERANCE_DEG = 8.0
VERTICAL_HARD_TCP_DRIFT_M = 0.050
VERTICAL_HARD_ORIENTATION_ERROR_DEG = 12.0
VERTICAL_MAX_CONSECUTIVE_SOFT_FAILURES = 2
VERTICAL_WORKSPACE_XY_RADIUS_M = 0.245
'''
if anchor not in s: raise RuntimeError('未找到垂直参数插入点')
s=s.replace(anchor,insert,1)

# Only in align_orientation_target: initialize counters.
old='''    records=[]; initial_q=start_end.orientation.copy()
    for si in range(1,stages+1):'''
new='''    records=[]; initial_q=start_end.orientation.copy()
    consecutive_soft_failures=0
    best_vertical_error=float("inf")
    best_stage=0
    for si in range(1,stages+1):'''
if old not in s: raise RuntimeError('未找到垂直姿态循环初始化')
s=s.replace(old,new,1)

# Replace per-stage record and immediate fail block.
old='''        records.append({"stage":si,"drift":drift,"orientation_error":qe,"vertical_error_deg":ve})
        print(f"[垂直姿态 {si}/{stages}] TCP漂移={drift:.4f}m, 姿态误差={qe:.2f}deg, 垂直误差={ve:.2f}deg")
        if drift>MAX_ROTATION_STAGE_DRIFT_M or qe>ORIENTATION_TOLERANCE_DEG:
            raise RuntimeError(f"垂直姿态第{si}段超限")'''
new='''        if ve<best_vertical_error:
            best_vertical_error=ve
            best_stage=si
        soft_failed=(
            drift>MAX_ROTATION_STAGE_DRIFT_M
            or qe>VERTICAL_STAGE_ORIENTATION_TOLERANCE_DEG
        )
        hard_failed=(
            drift>VERTICAL_HARD_TCP_DRIFT_M
            or qe>VERTICAL_HARD_ORIENTATION_ERROR_DEG
        )
        records.append({
            "stage":si,"drift":drift,"orientation_error":qe,
            "vertical_error_deg":ve,"soft_failed":soft_failed,
            "hard_failed":hard_failed,
        })
        print(f"[垂直姿态 {si}/{stages}] TCP漂移={drift:.4f}m, 姿态误差={qe:.2f}deg, 垂直误差={ve:.2f}deg")
        if hard_failed:
            report["vertical_orientation_stages"]=records
            report["vertical_failure"]={
                "type":"hard_limit","stage":si,"best_stage":best_stage,
                "best_vertical_error_deg":best_vertical_error,
            }
            save(report)
            raise RuntimeError(
                f"垂直姿态第{si}段达到硬限制: "
                f"TCP漂移={drift:.4f}m, 姿态误差={qe:.2f}deg"
            )
        if soft_failed:
            consecutive_soft_failures+=1
            print(
                f"[垂直姿态警告] 轻微超限 {consecutive_soft_failures}/"
                f"{VERTICAL_MAX_CONSECUTIVE_SOFT_FAILURES}，继续观察下一小段"
            )
        else:
            consecutive_soft_failures=0
        if consecutive_soft_failures>=VERTICAL_MAX_CONSECUTIVE_SOFT_FAILURES:
            report["vertical_orientation_stages"]=records
            report["vertical_failure"]={
                "type":"consecutive_soft_limit","stage":si,
                "best_stage":best_stage,
                "best_vertical_error_deg":best_vertical_error,
            }
            save(report)
            raise RuntimeError(
                f"垂直姿态连续{consecutive_soft_failures}段轻微超限，"
                "判断当前机械臂构型接近可达边界，禁止继续强制旋转"
            )'''
if old not in s: raise RuntimeError('未找到垂直姿态阶段验证块')
s=s.replace(old,new,1)

# Add workspace diagnostic before target orientation execution.
anchor='''        target_q=vertical_tool_orientation(yaw+math.radians(GRIPPER_YAW_OFFSET_DEG))
        total_orientation_change=q_error_deg(wp2end.orientation,target_q)'''
new='''        target_q=vertical_tool_orientation(yaw+math.radians(GRIPPER_YAW_OFFSET_DEG))
        total_orientation_change=q_error_deg(wp2end.orientation,target_q)
        workspace_xy_radius=math.hypot(
            product[0]-wp2tcp.position[0],
            product[1]-wp2tcp.position[1],
        )
        print(f"产品相对WP2的XY半径={workspace_xy_radius:.4f}m")
        if workspace_xy_radius>VERTICAL_WORKSPACE_XY_RADIUS_M:
            raise RuntimeError(
                f"产品相对WP2横向距离{workspace_xy_radius:.4f}m超过"
                f"垂直姿态经验工作区{VERTICAL_WORKSPACE_XY_RADIUS_M:.3f}m，"
                "禁止在边界位置强制垂直旋转"
            )'''
if anchor not in s: raise RuntimeError('未找到垂直目标姿态主流程')
s=s.replace(anchor,new,1)

# Startup output.
anchor='''        print(f"角度采样: 位置目标={DETECTION_SAMPLES}, 角度最低={MIN_ANGLE_SAMPLES}, 超时={DETECTION_TIMEOUT_S:.1f}s")'''
new='''        print(f"角度采样: 位置目标={DETECTION_SAMPLES}, 角度最低={MIN_ANGLE_SAMPLES}, 超时={DETECTION_TIMEOUT_S:.1f}s")
        print(f"自适应垂直姿态: 每段{MAX_YAW_PER_STAGE_DEG:.1f}deg, 软限制TCP={MAX_ROTATION_STAGE_DRIFT_M:.3f}m/姿态={VERTICAL_STAGE_ORIENTATION_TOLERANCE_DEG:.1f}deg, 硬限制TCP={VERTICAL_HARD_TCP_DRIFT_M:.3f}m/姿态={VERTICAL_HARD_ORIENTATION_ERROR_DEG:.1f}deg")'''
if anchor not in s: raise RuntimeError('未找到启动输出点')
s=s.replace(anchor,new,1)

dst.write_text(s,encoding='utf-8')
print(f'[OK] generated: {dst}')
