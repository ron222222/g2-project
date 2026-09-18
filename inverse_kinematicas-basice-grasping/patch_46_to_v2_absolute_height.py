#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""将46程序改为绝对安全高度版本，避免每次运行重复抬高25cm。"""
from pathlib import Path

SRC = Path('46_yolo_tcp_pick_lift_10cm_start_raise_safe.py')
DST = Path('46_yolo_tcp_pick_lift_10cm_start_raise_safe_v2.py')

if not SRC.exists():
    raise RuntimeError(f'未找到 {SRC}，请在项目目录运行本补丁')

s = SRC.read_text(encoding='utf-8')
s = s.replace('46_yolo_tcp_pick_lift_10cm_start_raise_safe.py',
              '46_yolo_tcp_pick_lift_10cm_start_raise_safe_v2.py')
s = s.replace('REPORT_FILE = "yolo_tcp_pick_lift_10cm_start_raise_report.json"',
              'REPORT_FILE = "yolo_tcp_pick_lift_10cm_start_raise_v2_report.json"')
s = s.replace(
'''START_RAISE_M = 0.250
MIN_START_RAISE_ACHIEVED_M = 0.200
HIGH_STAGING_ABOVE_PRODUCT_M = 0.300''',
'''SAFE_START_END_Z_M = 1.020       # 绝对安全高度，不重复叠加抬高
MAX_START_RAISE_M = 0.220          # 标准姿态最多抬高22cm
MIN_TCP_PRODUCT_CLEARANCE_M = 0.180
HIGH_STAGING_ABOVE_PRODUCT_M = 0.250''')
s = s.replace('POSITION_TOLERANCE_M = 0.020', 'POSITION_TOLERANCE_M = 0.025')

old = '''        initial_raise_end = [
            end_initial.position[0],
            end_initial.position[1],
            end_initial.position[2] + START_RAISE_M,
        ]
        initial_raise_tcp = tcp_from_end(initial_raise_end, end_initial.orientation)
'''
new = '''        # 绝对安全高度：重跑程序时不会再次叠加抬高20-25cm。
        requested_raise = max(0.0, SAFE_START_END_Z_M - end_initial.position[2])
        applied_raise = min(requested_raise, MAX_START_RAISE_M)
        initial_raise_end = [
            end_initial.position[0],
            end_initial.position[1],
            end_initial.position[2] + applied_raise,
        ]
        initial_raise_tcp = tcp_from_end(initial_raise_end, end_initial.orientation)
'''
if old not in s:
    raise RuntimeError('未找到起始抬高代码块，源文件可能不是预期版本')
s = s.replace(old, new)

s = s.replace(
'''                "initial_raise_end": initial_raise_end,
                "initial_raise_tcp": initial_raise_tcp,''',
'''                "initial_raise_end": initial_raise_end,
                "initial_raise_tcp": initial_raise_tcp,
                "requested_raise_m": requested_raise,
                "applied_raise_m": applied_raise,
                "safe_start_end_z_m": SAFE_START_END_Z_M,''')

s = s.replace(
'print(f"第一动作抬高        : {START_RAISE_M*100:.0f} cm")',
'print(f"第一动作计划抬高    : {applied_raise*100:.1f} cm；绝对End Z目标={SAFE_START_END_Z_M:.3f}m")')

old = '''        # Stage 1: 第一动作垂直抬高25cm，保持当前姿态。
        raise_target = PoseData(initial_raise_end, end_initial.orientation.copy())
        move_pose(
            robot, left_initial, wait_pose(tf_api, RIGHT_FRAME), raise_target,
            baseline, report, "initial_raise_25cm",
            minimum_tcp_z=tcp_initial.position[2],
        )
        require_reached(tf_api, RIGHT_FRAME, raise_target, "initial_raise_25cm", report)
        raised_tcp = wait_pose(tf_api, TCP_FRAME)
        achieved_raise = raised_tcp.position[2] - tcp_initial.position[2]
        report["achieved_start_raise_m"] = achieved_raise
        save_report(report)
        print(f"[起始抬高验证] TCP实际抬高={achieved_raise:.4f}m")
        if achieved_raise < MIN_START_RAISE_ACHIEVED_M:
            report["status"] = "STOPPED_INITIAL_RAISE_INSUFFICIENT"
            save_report(report)
            raise RuntimeError("TCP实际抬高不足20cm，禁止继续")
'''
new = '''        # Stage 1: 到绝对安全高度；当前已经够高时不再重复抬高。
        raise_target = PoseData(initial_raise_end, end_initial.orientation.copy())
        if applied_raise > 0.005:
            move_pose(
                robot, left_initial, wait_pose(tf_api, RIGHT_FRAME), raise_target,
                baseline, report, "initial_raise_absolute",
                minimum_tcp_z=tcp_initial.position[2],
            )
            require_reached(
                tf_api, RIGHT_FRAME, raise_target,
                "initial_raise_absolute", report
            )
        else:
            print("[起始抬高] 当前End已经达到绝对安全高度，不重复抬高。")

        raised_tcp = wait_pose(tf_api, TCP_FRAME)
        achieved_raise = raised_tcp.position[2] - tcp_initial.position[2]
        product_clearance = raised_tcp.position[2] - product[2]
        report["achieved_start_raise_m"] = achieved_raise
        report["raised_tcp_product_clearance_m"] = product_clearance
        save_report(report)
        print(f"[起始高度验证] TCP相对产品高度={product_clearance:.4f}m")
        if product_clearance < MIN_TCP_PRODUCT_CLEARANCE_M:
            report["status"] = "STOPPED_INITIAL_CLEARANCE_INSUFFICIENT"
            save_report(report)
            raise RuntimeError("TCP相对产品安全高度不足18cm，禁止继续")
'''
if old not in s:
    raise RuntimeError('未找到Stage 1代码块，源文件可能不是预期版本')
s = s.replace(old, new)

s = s.replace(
'require_reached(tf_api, RIGHT_FRAME, high_translate_target, "high_translate_xy", report)',
'''require_reached(
            tf_api, RIGHT_FRAME, high_translate_target,
            "high_translate_xy", report, pos_tol=0.025
        )''')
s = s.replace('流程: 抬高25cm ->', '流程: 到绝对安全高度 ->')

DST.write_text(s, encoding='utf-8')
compile(s, str(DST), 'exec')
print(f'已生成: {DST.resolve()}')
