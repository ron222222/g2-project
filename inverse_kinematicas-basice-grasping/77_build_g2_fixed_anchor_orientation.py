#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
基于 76_g2_pick_adaptive_vertical_pose.py 生成：
77_g2_pick_fixed_anchor_orientation.py

核心修复：
1. 先在Waypoint_2高度执行产品XY粗对准，再调整垂直姿态。
2. 整个垂直姿态过程使用同一个固定TCP锚点，不再每段把漂移后的TCP当新锚点。
3. 每段姿态完成后检查相对固定锚点的累计漂移，而不是只看单段漂移。
4. 姿态完成后仅允许小范围XY精修；若残差超过80mm立即停止。
5. 增加工作区和累计漂移报告，避免36段后累计漂移到30cm。
6. 保留76版的YOLO角度修正、抓取高度、分段腰部右转及松爪逻辑。

本生成器不控制机器人。
"""
from pathlib import Path

SOURCE = "76_g2_pick_adaptive_vertical_pose.py"
OUTPUT = "77_g2_pick_fixed_anchor_orientation.py"
root = Path(__file__).resolve().parent
src = root / SOURCE
dst = root / OUTPUT
if not src.exists():
    raise RuntimeError(f"缺少源程序: {src}")

s = src.read_text(encoding="utf-8")
s = s.replace(SOURCE, OUTPUT)
s = s.replace(
    "g2_pick_adaptive_vertical_pose_report.json",
    "g2_pick_fixed_anchor_orientation_report.json",
)
s = s.replace(
    "g2_pick_adaptive_vertical_pose_live.jpg",
    "g2_pick_fixed_anchor_orientation_live.jpg",
)
s = s.replace(
    "g2_pick_adaptive_vertical_pose_angle.jpg",
    "g2_pick_fixed_anchor_orientation_angle.jpg",
)

# 新增固定锚点与残差限制。
anchor = "VERTICAL_WORKSPACE_XY_RADIUS_M = 0.245\n"
insert = '''VERTICAL_WORKSPACE_XY_RADIUS_M = 0.245
VERTICAL_FIXED_ANCHOR_SOFT_DRIFT_M = 0.030
VERTICAL_FIXED_ANCHOR_HARD_DRIFT_M = 0.050
POST_ORIENTATION_MAX_XY_RESIDUAL_M = 0.080
PRE_ORIENTATION_MAX_XY_MOVE_M = 0.240
'''
if anchor not in s:
    raise RuntimeError("未找到垂直工作区参数")
s = s.replace(anchor, insert, 1)

# 整体替换垂直姿态函数，实现全程固定TCP锚点。
start = s.index("def align_orientation_target(robot,tf_api,left,target_q,base,report):")
end = s.index("\n\ndef rotate_yaw(", start)
new_function = r'''def align_orientation_target(robot,tf_api,left,target_q,base,report):
    """绕同一个固定TCP锚点分段调整姿态，禁止累计漂移。"""
    initial_end=wait_pose(tf_api,RIGHT_FRAME)
    anchor_tcp=wait_pose(tf_api,TCP_FRAME)
    initial_q=initial_end.orientation.copy()
    total=q_error_deg(initial_q,target_q)
    stages=max(1,int(math.ceil(total/MAX_YAW_PER_STAGE_DEG)))

    if REQUIRE_CONFIRMATION:
        command=input(
            f"准备固定TCP锚点垂直姿态对齐{total:.1f}deg/{stages}段，输入 NEXT："
        ).strip()
        if command!="NEXT":
            raise RuntimeError("用户取消固定锚点垂直姿态对齐")

    records=[]
    consecutive_soft_failures=0
    best_vertical_error=float("inf")
    best_stage=0

    for si in range(1,stages+1):
        stage_target=slerp(initial_q,target_q,si/stages)
        actual_end_before=wait_pose(tf_api,RIGHT_FRAME)
        orientation_change=q_error_deg(actual_end_before.orientation,stage_target)
        n=max(2,int(math.ceil(orientation_change/ROTATION_STEP_DEG)))

        for i in range(1,n+1):
            qq=slerp(actual_end_before.orientation,stage_target,i/n)
            # 关键：每个插值点都使用最初的固定anchor_tcp，而不是本段实际TCP。
            pp=end_from_tcp(anchor_tcp.position,qq)
            set_pose(robot,left,PoseData(pp,qq))
            time.sleep(DT)

        hold=max(1,int(ROTATION_FINAL_HOLD_S*RATE_HZ))
        final_end_position=end_from_tcp(anchor_tcp.position,stage_target)
        for _ in range(hold):
            set_pose(robot,left,PoseData(final_end_position,stage_target))
            time.sleep(DT)

        time.sleep(SETTLE_S)
        actual_tcp=wait_pose(tf_api,TCP_FRAME)
        actual_end=wait_pose(tf_api,RIGHT_FRAME)
        cumulative_drift=dist(actual_tcp.position,anchor_tcp.position)
        orientation_error=q_error_deg(actual_end.orientation,stage_target)
        vertical_error=tool_vertical_error_deg(actual_end.orientation)

        if vertical_error<best_vertical_error:
            best_vertical_error=vertical_error
            best_stage=si

        soft_failed=(
            cumulative_drift>VERTICAL_FIXED_ANCHOR_SOFT_DRIFT_M
            or orientation_error>VERTICAL_STAGE_ORIENTATION_TOLERANCE_DEG
        )
        hard_failed=(
            cumulative_drift>VERTICAL_FIXED_ANCHOR_HARD_DRIFT_M
            or orientation_error>VERTICAL_HARD_ORIENTATION_ERROR_DEG
        )
        records.append({
            "stage":si,
            "stage_count":stages,
            "anchor_tcp":anchor_tcp.__dict__,
            "actual_tcp":actual_tcp.__dict__,
            "cumulative_anchor_drift_m":cumulative_drift,
            "orientation_error_deg":orientation_error,
            "vertical_error_deg":vertical_error,
            "soft_failed":soft_failed,
            "hard_failed":hard_failed,
        })
        report["vertical_orientation_stages"]=records
        save(report)

        print(
            f"[固定锚点垂直姿态 {si}/{stages}] "
            f"累计TCP漂移={cumulative_drift:.4f}m, "
            f"姿态误差={orientation_error:.2f}deg, "
            f"垂直误差={vertical_error:.2f}deg"
        )

        if hard_failed:
            report["vertical_failure"]={
                "type":"fixed_anchor_hard_limit",
                "stage":si,
                "best_stage":best_stage,
                "best_vertical_error_deg":best_vertical_error,
            }
            save(report)
            raise RuntimeError(
                f"固定锚点垂直姿态第{si}段达到硬限制: "
                f"累计TCP漂移={cumulative_drift:.4f}m, "
                f"姿态误差={orientation_error:.2f}deg"
            )

        if soft_failed:
            consecutive_soft_failures+=1
            print(
                f"[固定锚点姿态警告] 轻微超限 "
                f"{consecutive_soft_failures}/"
                f"{VERTICAL_MAX_CONSECUTIVE_SOFT_FAILURES}"
            )
        else:
            consecutive_soft_failures=0

        if consecutive_soft_failures>=VERTICAL_MAX_CONSECUTIVE_SOFT_FAILURES:
            report["vertical_failure"]={
                "type":"fixed_anchor_consecutive_soft_limit",
                "stage":si,
                "best_stage":best_stage,
                "best_vertical_error_deg":best_vertical_error,
            }
            save(report)
            raise RuntimeError(
                "固定锚点垂直姿态连续两段轻微超限，"
                "当前构型接近可达边界，禁止继续"
            )

    final_tcp=wait_pose(tf_api,TCP_FRAME)
    final_end=wait_pose(tf_api,RIGHT_FRAME)
    final_anchor_drift=dist(final_tcp.position,anchor_tcp.position)
    final_vertical_error=tool_vertical_error_deg(final_end.orientation)
    report["vertical_orientation_summary"]={
        "anchor_tcp":anchor_tcp.__dict__,
        "final_tcp":final_tcp.__dict__,
        "final_anchor_drift_m":final_anchor_drift,
        "final_vertical_error_deg":final_vertical_error,
        "best_stage":best_stage,
        "best_vertical_error_deg":best_vertical_error,
    }
    save(report)

    if final_anchor_drift>VERTICAL_FIXED_ANCHOR_HARD_DRIFT_M:
        raise RuntimeError(
            f"垂直姿态最终累计TCP漂移{final_anchor_drift:.4f}m超过50mm"
        )
    if final_vertical_error>VERTICAL_TOOL_TILT_TOLERANCE_DEG:
        raise RuntimeError(
            f"夹爪末端未达到垂直容差: {final_vertical_error:.2f}deg"
        )
    return final_tcp,final_end
'''
s = s[:start] + new_function + s[end:]

# 重排主流程：先高位XY粗对准，再固定锚点姿态，再小范围XY精修。
old = '''        target_q=vertical_tool_orientation(yaw+math.radians(GRIPPER_YAW_OFFSET_DEG))
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
            )
        print(f"产品长轴={math.degrees(yaw):.1f}deg, 当前夹爪长轴={math.degrees(gy):.1f}deg")
        print(f"垂直目标姿态总变化={total_orientation_change:.1f}deg, 目标工具Z轴垂直向下")
        if not ENABLE_ANGLE_ALIGNMENT:
            report["status"]="VERTICAL_ORIENTATION_PLAN_READY";save(report);return
        aligned_tcp,aligned_end=align_orientation_target(robot,tf_api,left,target_q,base,report);q=aligned_end.orientation
        if not ENABLE_XY_AND_PREGRASP:
            report["status"]="ANGLE_ALIGNED";save(report);return
        aligned_tcp,_=move_tcp_closed_loop(robot,tf_api,left,
            [pre[0]-aligned_tcp.position[0],pre[1]-aligned_tcp.position[1],0.0],q,base,report,"align_xy")'''
new = '''        target_q=vertical_tool_orientation(yaw+math.radians(GRIPPER_YAW_OFFSET_DEG))
        total_orientation_change=q_error_deg(wp2end.orientation,target_q)
        workspace_xy_radius=math.hypot(
            product[0]-wp2tcp.position[0],
            product[1]-wp2tcp.position[1],
        )
        print(f"产品相对WP2的XY半径={workspace_xy_radius:.4f}m")
        if workspace_xy_radius>PRE_ORIENTATION_MAX_XY_MOVE_M:
            raise RuntimeError(
                f"产品相对WP2横向距离{workspace_xy_radius:.4f}m超过"
                f"高位粗对准限制{PRE_ORIENTATION_MAX_XY_MOVE_M:.3f}m，"
                "当前产品位置超出可靠抓取范围"
            )
        print(f"产品长轴={math.degrees(yaw):.1f}deg, 当前夹爪长轴={math.degrees(gy):.1f}deg")
        print(f"垂直目标姿态总变化={total_orientation_change:.1f}deg, 目标工具Z轴垂直向下")

        if not ENABLE_XY_AND_PREGRASP:
            report["status"]="XY_AND_VERTICAL_PLAN_READY";save(report);return

        # 第一步：保持Waypoint_2当前姿态，先在高位移动到产品XY。
        coarse_tcp,_=move_tcp_closed_loop(
            robot,tf_api,left,
            [pre[0]-wp2tcp.position[0],pre[1]-wp2tcp.position[1],0.0],
            wp2end.orientation,base,report,"coarse_align_xy_before_orientation"
        )

        if not ENABLE_ANGLE_ALIGNMENT:
            report["status"]="COARSE_XY_ALIGNED_VERTICAL_DISABLED";save(report);return

        # 第二步：在产品XY上方，绕固定TCP锚点调整完整垂直姿态。
        aligned_tcp,aligned_end=align_orientation_target(
            robot,tf_api,left,target_q,base,report
        )
        q=aligned_end.orientation

        # 第三步：姿态完成后只允许小范围XY精修。
        residual_xy=math.hypot(
            pre[0]-aligned_tcp.position[0],
            pre[1]-aligned_tcp.position[1],
        )
        print(f"垂直姿态后XY残差={residual_xy:.4f}m")
        if residual_xy>POST_ORIENTATION_MAX_XY_RESIDUAL_M:
            raise RuntimeError(
                f"垂直姿态后XY残差{residual_xy:.4f}m超过"
                f"{POST_ORIENTATION_MAX_XY_RESIDUAL_M:.3f}m，"
                "说明姿态过程中发生不可接受的累计漂移"
            )
        aligned_tcp,_=move_tcp_closed_loop(
            robot,tf_api,left,
            [pre[0]-aligned_tcp.position[0],pre[1]-aligned_tcp.position[1],0.0],
            q,base,report,"fine_align_xy_after_orientation"
        )'''
if old not in s:
    raise RuntimeError("未找到76版主流程姿态/XY代码块")
s = s.replace(old, new, 1)

# 启动输出。
anchor = '''        print(f"自适应垂直姿态: 每段{MAX_YAW_PER_STAGE_DEG:.1f}deg, 软限制TCP={MAX_ROTATION_STAGE_DRIFT_M:.3f}m/姿态={VERTICAL_STAGE_ORIENTATION_TOLERANCE_DEG:.1f}deg, 硬限制TCP={VERTICAL_HARD_TCP_DRIFT_M:.3f}m/姿态={VERTICAL_HARD_ORIENTATION_ERROR_DEG:.1f}deg")'''
new = '''        print(f"固定锚点垂直姿态: 每段{MAX_YAW_PER_STAGE_DEG:.1f}deg, 累计漂移软限制={VERTICAL_FIXED_ANCHOR_SOFT_DRIFT_M:.3f}m, 硬限制={VERTICAL_FIXED_ANCHOR_HARD_DRIFT_M:.3f}m")
        print(f"可靠顺序: 高位XY粗对准 -> 固定TCP垂直姿态 -> XY小范围精修；姿态后XY残差限制={POST_ORIENTATION_MAX_XY_RESIDUAL_M:.3f}m")'''
if anchor not in s:
    raise RuntimeError("未找到76版启动姿态输出")
s = s.replace(anchor, new, 1)

dst.write_text(s, encoding="utf-8")
print(f"[OK] generated: {dst}")
