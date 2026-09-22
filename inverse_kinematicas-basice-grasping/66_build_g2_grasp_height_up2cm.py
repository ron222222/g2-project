#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
66_build_g2_grasp_height_up2cm.py

在当前项目目录中读取：
    63_g2_yolo_pick_closed_loop_complete.py
生成完整独立程序：
    66_g2_yolo_pick_closed_loop_grasp_up2cm.py

唯一功能性修改：
    GRASP_ABOVE_PRODUCT_M: 0.025 m -> 0.045 m

即桌面抓取阶段的夹爪TCP目标，相对63版向上提高20 mm。
PreGrasp、YOLO、角度对齐、XY闭环、头部、Waypoint和抬升逻辑保持不变。

本生成器不控制机器人，只生成新程序。
"""

from pathlib import Path

SOURCE_NAME = "63_g2_yolo_pick_closed_loop_complete.py"
OUTPUT_NAME = "66_g2_yolo_pick_closed_loop_grasp_up2cm.py"

root = Path(__file__).resolve().parent
source = root / SOURCE_NAME
output = root / OUTPUT_NAME

if not source.exists():
    raise RuntimeError(
        f"缺少源程序: {source}\n"
        f"请将本生成器与 {SOURCE_NAME} 放在同一目录。"
    )

text = source.read_text(encoding="utf-8")

replacements = [
    (
        "63_g2_yolo_pick_closed_loop_complete.py",
        "66_g2_yolo_pick_closed_loop_grasp_up2cm.py",
    ),
    (
        "g2_yolo_pick_closed_loop_complete_report.json",
        "g2_yolo_pick_closed_loop_grasp_up2cm_report.json",
    ),
    (
        "g2_yolo_closed_loop_live.jpg",
        "g2_yolo_grasp_up2cm_live.jpg",
    ),
    (
        "g2_yolo_closed_loop_angle.jpg",
        "g2_yolo_grasp_up2cm_angle.jpg",
    ),
]

for old, new in replacements:
    text = text.replace(old, new)

old_height = "GRASP_ABOVE_PRODUCT_M = 0.025"
new_height = (
    "GRASP_ABOVE_PRODUCT_M = 0.045  "
    "# 相对63版向上提高20mm: 25mm -> 45mm"
)
if old_height not in text:
    raise RuntimeError(
        "未找到63版抓取高度参数。"
        "请确认源程序包含 GRASP_ABOVE_PRODUCT_M = 0.025"
    )
text = text.replace(old_height, new_height, 1)

# 在报告中记录本次高度标定，便于追溯。
report_anchor = '"qt_font":QT_FONT_RESULT,\n'
report_insert = '''"qt_font":QT_FONT_RESULT,
            "grasp_height_calibration": {
                "source_program": "63_g2_yolo_pick_closed_loop_complete.py",
                "previous_grasp_above_product_m": 0.025,
                "new_grasp_above_product_m": 0.045,
                "z_change_m": 0.020,
                "direction": "up",
            },
'''
if report_anchor not in text:
    raise RuntimeError("未找到报告配置插入位置")
text = text.replace(report_anchor, report_insert, 1)

# 启动时明确显示高度变化。
print_anchor = '''        print(f"起始姿态={name}, 夹爪长轴={GRIPPER_LONG_AXIS_LOCAL.upper()}, Grasp Z偏移={GRASP_ABOVE_PRODUCT_M:.3f}m")
'''
print_insert = '''        print(f"起始姿态={name}, 夹爪长轴={GRIPPER_LONG_AXIS_LOCAL.upper()}, Grasp Z偏移={GRASP_ABOVE_PRODUCT_M:.3f}m")
        print("桌面抓取高度相对63版向上调整=0.020m")
        print("63版偏移=0.025m, 66版偏移=0.045m")
'''
if print_anchor not in text:
    raise RuntimeError("未找到启动参数输出位置")
text = text.replace(print_anchor, print_insert, 1)

output.write_text(text, encoding="utf-8")
print(f"[OK] 已生成完整程序: {output}")
print("[OK] Grasp Z偏移: 0.025m -> 0.045m")
print("[OK] 相对63版向上提高: 0.020m")
