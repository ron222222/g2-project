#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
65_build_g2_pregrasp_rotate90.py

在机器人项目目录中读取：
    63_g2_yolo_pick_closed_loop_complete.py
生成完整独立程序：
    65_g2_yolo_pick_pregrasp_rotate90.py

修改内容：
1. 最终夹爪方向相对63版增加 +90度工具角度补偿。
2. 角度对齐后重新读取真实TCP，再重新计算PreGrasp三维误差。
3. PreGrasp先做XY闭环，再做Z闭环，最后增加一次三维总误差复核。
4. 若PreGrasp总误差在25~50mm之间，执行最多2次三维闭环修正。
5. 误差超过50mm立即停止。
6. 其余YOLO、深度、头部、Waypoint、夹爪高度和抬升逻辑保持63版。

本生成器不控制机器人，只生成65号完整程序。
"""

from pathlib import Path

SOURCE = "63_g2_yolo_pick_closed_loop_complete.py"
OUTPUT = "65_g2_yolo_pick_pregrasp_rotate90.py"

root = Path(__file__).resolve().parent
src = root / SOURCE
dst = root / OUTPUT
if not src.exists():
    raise RuntimeError(f"缺少源程序: {src}")

s = src.read_text(encoding="utf-8")
s = s.replace(
    "63_g2_yolo_pick_closed_loop_complete.py",
    "65_g2_yolo_pick_pregrasp_rotate90.py",
)
s = s.replace(
    "g2_yolo_pick_closed_loop_complete_report.json",
    "g2_yolo_pick_pregrasp_rotate90_report.json",
)
s = s.replace(
    "g2_yolo_closed_loop_live.jpg",
    "g2_yolo_pregrasp_rotate90_live.jpg",
)
s = s.replace(
    "g2_yolo_closed_loop_angle.jpg",
    "g2_yolo_pregrasp_rotate90_angle.jpg",
)

# 工具方向在现有结果基础上固定增加90度。
old = 'GRIPPER_YAW_OFFSET_DEG = 0.0'
new = '''GRIPPER_YAW_OFFSET_DEG = 90.0
# 对无方向长轴而言，最终仍通过wrap_axis()选择等效的最短旋转。
# 该参数表示夹爪末端目标相对63版工具方向旋转90度。'''
if old not in s:
    raise RuntimeError("未找到GRIPPER_YAW_OFFSET_DEG")
s = s.replace(old, new, 1)

# 允许90度工具补偿经过wrap后产生的最大等效修正。
s = s.replace(
    "MAX_YAW_CORRECTION_DEG = 60.0",
    "MAX_YAW_CORRECTION_DEG = 90.0",
    1,
)

# 增加PreGrasp最终三维检查配置。
anchor = 'MAX_CORRECTION_ATTEMPTS = 2\nCORRECTION_STEP_M = 0.001\n'
replacement = '''MAX_CORRECTION_ATTEMPTS = 2
CORRECTION_STEP_M = 0.001
PREGRASP_FINAL_TOLERANCE_M = 0.025
PREGRASP_HARD_ERROR_LIMIT_M = 0.050
PREGRASP_FINAL_CORRECTION_ATTEMPTS = 2
'''
if anchor not in s:
    raise RuntimeError("未找到闭环参数插入点")
s = s.replace(anchor, replacement, 1)

# 在set_gripper之前插入PreGrasp总误差复核函数。
anchor = '\ndef set_gripper(robot,value):\n'
helper = r'''
def verify_and_correct_pregrasp_3d(
    robot, tf_api, left, target_pregrasp, q, base, report
):
    """角度/XY/Z动作后，按真实TCP复核PreGrasp三维位置。"""
    records = []
    for attempt in range(PREGRASP_FINAL_CORRECTION_ATTEMPTS + 1):
        actual_tcp = wait_pose(tf_api, TCP_FRAME)
        actual_end = wait_pose(tf_api, RIGHT_FRAME)
        error_xyz = [
            target_pregrasp[i] - actual_tcp.position[i]
            for i in range(3)
        ]
        error_norm = math.sqrt(sum(v*v for v in error_xyz))
        orientation_error = q_error_deg(actual_end.orientation, q)
        records.append({
            "attempt": attempt,
            "actual_tcp": actual_tcp.__dict__,
            "error_xyz": error_xyz,
            "error_norm_m": error_norm,
            "orientation_error_deg": orientation_error,
        })
        print(
            f"[PreGrasp三维复核 {attempt}] "
            f"误差XYZ={[round(v,5) for v in error_xyz]}, "
            f"总误差={error_norm:.4f}m, "
            f"姿态误差={orientation_error:.2f}deg"
        )
        if (
            error_norm <= PREGRASP_FINAL_TOLERANCE_M
            and orientation_error <= ORIENTATION_TOLERANCE_DEG
        ):
            report["pregrasp_final_verification"] = records
            save(report)
            return actual_tcp, actual_end
        if error_norm > PREGRASP_HARD_ERROR_LIMIT_M:
            report["pregrasp_final_verification"] = records
            save(report)
            raise RuntimeError(
                f"PreGrasp总误差{error_norm:.4f}m超过50mm硬限制"
            )
        if attempt >= PREGRASP_FINAL_CORRECTION_ATTEMPTS:
            report["pregrasp_final_verification"] = records
            save(report)
            raise RuntimeError("PreGrasp三维闭环修正后仍未到位")

        # 将真实TCP剩余误差转换为End的平移修正，姿态保持不变。
        correction_target_end = [
            actual_end.position[i] + error_xyz[i]
            for i in range(3)
        ]
        print(
            f"[PreGrasp] 执行三维自动修正{attempt+1}: "
            f"{[round(v,5) for v in error_xyz]}"
        )
        command_cart_path(
            robot,
            left,
            actual_end.position,
            correction_target_end,
            q,
            base,
            report,
            "pregrasp_final_correction",
            CORRECTION_STEP_M,
        )
        time.sleep(SETTLE_S)

    raise RuntimeError("PreGrasp三维复核异常退出")

'''
if anchor not in s:
    raise RuntimeError("未找到set_gripper插入点")
s = s.replace(anchor, '\n' + helper + 'def set_gripper(robot,value):\n', 1)

# 替换Main中的PreGrasp动作段：旋转后以真实TCP重新计算XY和Z，并最终3D复核。
old = '''        aligned_tcp,_=move_tcp_closed_loop(robot,tf_api,left,
            [pre[0]-aligned_tcp.position[0],pre[1]-aligned_tcp.position[1],0.0],q,base,report,"align_xy")
        cur=wait_pose(tf_api,TCP_FRAME)
        move_tcp_closed_loop(robot,tf_api,left,[0.0,0.0,pre[2]-cur.position[2]],q,base,report,"descend_pregrasp")
        if not ENABLE_GRIPPER_AND_PICK:
            report["status"]="PREGRASP_REACHED";save(report);return
'''
new = '''        # 角度对齐可能产生TCP漂移，必须使用旋转后的真实TCP重新计算PreGrasp。
        aligned_tcp = wait_pose(tf_api, TCP_FRAME)
        aligned_tcp,_=move_tcp_closed_loop(robot,tf_api,left,
            [pre[0]-aligned_tcp.position[0],pre[1]-aligned_tcp.position[1],0.0],q,base,report,"align_xy")
        cur=wait_pose(tf_api,TCP_FRAME)
        move_tcp_closed_loop(robot,tf_api,left,
            [0.0,0.0,pre[2]-cur.position[2]],q,base,report,"descend_pregrasp")

        # XY与Z分别闭环后，再按三维欧氏误差复核并做小范围修正。
        verify_and_correct_pregrasp_3d(
            robot, tf_api, left, pre, q, base, report
        )
        if not ENABLE_GRIPPER_AND_PICK:
            report["status"]="PREGRASP_REACHED";save(report);return
'''
if old not in s:
    raise RuntimeError("未找到Main中的PreGrasp动作段")
s = s.replace(old, new, 1)

# 报告和启动输出写入新标定信息。
old = '''"qt_font":QT_FONT_RESULT,
            "joint_stages":[],"cartesian_stages":[],"status":"INITIALIZED"}'''
new = '''"qt_font":QT_FONT_RESULT,
            "tool_orientation_change": {
                "relative_to_63_deg": 90.0,
                "gripper_yaw_offset_deg": GRIPPER_YAW_OFFSET_DEG,
            },
            "pregrasp_control": {
                "final_tolerance_m": PREGRASP_FINAL_TOLERANCE_M,
                "hard_error_limit_m": PREGRASP_HARD_ERROR_LIMIT_M,
                "max_final_corrections": PREGRASP_FINAL_CORRECTION_ATTEMPTS,
            },
            "joint_stages":[],"cartesian_stages":[],"status":"INITIALIZED"}'''
if old not in s:
    raise RuntimeError("未找到report初始化段")
s = s.replace(old, new, 1)

old = '''        print(f"起始姿态={name}, 夹爪长轴={GRIPPER_LONG_AXIS_LOCAL.upper()}, Grasp Z偏移={GRASP_ABOVE_PRODUCT_M:.3f}m")
        print(f"XY闭环: 容差25mm, 硬限制50mm, 最多修正{MAX_CORRECTION_ATTEMPTS}次")'''
new = '''        print(f"起始姿态={name}, 夹爪长轴={GRIPPER_LONG_AXIS_LOCAL.upper()}, Grasp Z偏移={GRASP_ABOVE_PRODUCT_M:.3f}m")
        print(f"夹爪末端方向相对63版旋转={GRIPPER_YAW_OFFSET_DEG:.1f}deg")
        print(f"XY闭环: 容差25mm, 硬限制50mm, 最多修正{MAX_CORRECTION_ATTEMPTS}次")
        print(f"PreGrasp三维复核: 容差25mm, 硬限制50mm, 最多修正{PREGRASP_FINAL_CORRECTION_ATTEMPTS}次")'''
if old not in s:
    raise RuntimeError("未找到启动输出段")
s = s.replace(old, new, 1)

dst.write_text(s, encoding="utf-8")
print(f"[OK] generated complete program: {dst}")
