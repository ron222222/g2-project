#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
82_build_g2_autorun_product_height40.py

放在与 77_g2_pick_fixed_anchor_orientation.py 相同目录运行，生成：
82_g2_pick_autorun_product_height40.py

修改：
1. 取消 PICK/NEXT/TURN/RELEASE 等人工输入，保留全部自动安全检查。
2. 产品实际高度按 40 mm 建模。
3. 当视觉测得产品高度明显失真时，使用 40 mm 标称高度。
4. 最终抓取Z基于：桌面Z + 有效产品高度 + 抓取TCP相对产品顶部偏移。
5. 保留77版固定锚点垂直姿态、高位XY粗对准、XY精修、抓取、抬升、
   腰部动作和报告逻辑。

本生成器不控制机器人。
"""
from pathlib import Path
import re

SOURCE = "77_g2_pick_fixed_anchor_orientation.py"
OUTPUT = "82_g2_pick_autorun_product_height40.py"
root = Path(__file__).resolve().parent
src = root / SOURCE
dst = root / OUTPUT
if not src.exists():
    raise RuntimeError(f"缺少源程序: {src}")

s = src.read_text(encoding="utf-8")
s = s.replace(SOURCE, OUTPUT)
s = s.replace(
    "g2_pick_fixed_anchor_orientation_report.json",
    "g2_pick_autorun_product_height40_report.json",
)
s = s.replace(
    "g2_pick_fixed_anchor_orientation_live.jpg",
    "g2_pick_autorun_product_height40_live.jpg",
)
s = s.replace(
    "g2_pick_fixed_anchor_orientation_angle.jpg",
    "g2_pick_autorun_product_height40_angle.jpg",
)

# 自动运行：77版所有确认均受 REQUIRE_CONFIRMATION 控制。
if "REQUIRE_CONFIRMATION = True" not in s:
    raise RuntimeError("未找到 REQUIRE_CONFIRMATION = True")
s = s.replace(
    "REQUIRE_CONFIRMATION = True",
    "REQUIRE_CONFIRMATION = False  # 全自动运行，不等待PICK/NEXT/TURN/RELEASE",
    1,
)

# 统一抓取高度参数。偏移定义为相对产品顶部，而非相对错误product_z。
height_pattern = re.compile(
    r"PREGRASP_ABOVE_PRODUCT_M\s*=\s*[-+]?\d*\.?\d+.*?\n"
    r"GRASP_ABOVE_PRODUCT_M\s*=\s*[-+]?\d*\.?\d+.*?\n"
    r"MIN_TCP_ABOVE_TABLE_M\s*=\s*[-+]?\d*\.?\d+.*?\n"
    r"MAX_PRODUCT_Z_RANGE_M\s*=\s*[-+]?\d*\.?\d+.*?\n",
    re.MULTILINE,
)
height_block = '''PREGRASP_ABOVE_PRODUCT_M = 0.120
# 产品实际高度为40mm。以下偏移是TCP相对产品顶部的目标高度。
PRODUCT_NOMINAL_HEIGHT_M = 0.040
PRODUCT_HEIGHT_VALID_MIN_M = 0.025
PRODUCT_HEIGHT_VALID_MAX_M = 0.055
GRASP_ABOVE_PRODUCT_TOP_M = 0.000
# 保留旧变量名，避免其他显示或报告代码失效；不再直接用于Grasp Z计算。
GRASP_ABOVE_PRODUCT_M = GRASP_ABOVE_PRODUCT_TOP_M
MIN_TCP_ABOVE_TABLE_M = 0.010
MAX_PRODUCT_Z_RANGE_M = 0.012
'''
s, count = height_pattern.subn(height_block, s, count=1)
if count != 1:
    raise RuntimeError("未找到完整抓取高度参数块")

# 替换桌面与产品高度的计算核心。
old = '''        table_floor=det["table_z_m"]+MIN_TCP_ABOVE_TABLE_M
        nominal_grasp_z=product[2]+GRASP_ABOVE_PRODUCT_M
        safe_grasp_z=max(nominal_grasp_z,table_floor)
        grasp_z_source=("product_z_plus_45mm" if nominal_grasp_z>=table_floor
                        else "table_z_plus_40mm_safety_floor")
        pre=[product[0],product[1],max(product[2]+PREGRASP_ABOVE_PRODUCT_M,safe_grasp_z+0.060)]
        grasp=[product[0],product[1],safe_grasp_z]'''
new = '''        table_z=float(det["table_z_m"])
        measured_product_height=float(product[2]-table_z)
        if PRODUCT_HEIGHT_VALID_MIN_M <= measured_product_height <= PRODUCT_HEIGHT_VALID_MAX_M:
            effective_product_height=measured_product_height
            product_height_source="measured_product_height"
        else:
            effective_product_height=PRODUCT_NOMINAL_HEIGHT_M
            product_height_source="nominal_product_height_40mm"

        product_top_z=table_z+effective_product_height
        table_floor=table_z+MIN_TCP_ABOVE_TABLE_M
        nominal_grasp_z=product_top_z+GRASP_ABOVE_PRODUCT_TOP_M
        safe_grasp_z=max(nominal_grasp_z,table_floor)
        grasp_z_source=(
            "product_top_z_plus_offset"
            if nominal_grasp_z >= table_floor
            else "table_safety_floor"
        )
        # PreGrasp也以产品顶部为基准，避免错误product_z把轨迹整体拉低。
        pre=[
            product[0],
            product[1],
            max(product_top_z+PREGRASP_ABOVE_PRODUCT_M,safe_grasp_z+0.060),
        ]
        grasp=[product[0],product[1],safe_grasp_z]'''
if old not in s:
    # 兼容没有grasp_z_source的77变体。
    old2 = '''        table_floor=det["table_z_m"]+MIN_TCP_ABOVE_TABLE_M
        nominal_grasp_z=product[2]+GRASP_ABOVE_PRODUCT_M
        safe_grasp_z=max(nominal_grasp_z,table_floor)
        pre=[product[0],product[1],max(product[2]+PREGRASP_ABOVE_PRODUCT_M,safe_grasp_z+0.060)]
        grasp=[product[0],product[1],safe_grasp_z]'''
    if old2 not in s:
        raise RuntimeError("未找到77版Grasp Z计算段")
    s = s.replace(old2, new, 1)
else:
    s = s.replace(old, new, 1)

# 扩展报告字段。
old = '''                       "table_floor_tcp_z":table_floor,"nominal_grasp_z":nominal_grasp_z,
                       "safe_grasp_z":safe_grasp_z,"grasp_z_source":grasp_z_source});save(report)'''
new = '''                       "table_floor_tcp_z":table_floor,
                       "measured_product_height_m":measured_product_height,
                       "effective_product_height_m":effective_product_height,
                       "product_height_source":product_height_source,
                       "product_top_z":product_top_z,
                       "nominal_grasp_z":nominal_grasp_z,
                       "safe_grasp_z":safe_grasp_z,
                       "grasp_z_source":grasp_z_source});save(report)'''
if old in s:
    s = s.replace(old, new, 1)
else:
    old2 = '''                       "table_floor_tcp_z":table_floor,"nominal_grasp_z":nominal_grasp_z,
                       "safe_grasp_z":safe_grasp_z});save(report)'''
    if old2 not in s:
        raise RuntimeError("未找到77版高度报告字段")
    s = s.replace(old2, new, 1)

# 替换高度输出，明确数据来源。
old = '''        print(f"产品Z极差={det['product_z_range_m']:.4f}m, 桌面Z={det['table_z_m']:.5f}m")
        print(f"名义Grasp Z={nominal_grasp_z:.5f}m, 桌面安全下限={table_floor:.5f}m, 最终Grasp Z={safe_grasp_z:.5f}m")
        print(f"最终Grasp Z来源={grasp_z_source}")'''
new = '''        print(f"产品Z极差={det['product_z_range_m']:.4f}m, 桌面Z={table_z:.5f}m")
        print(f"视觉测量产品高度={measured_product_height:.5f}m")
        print(f"有效产品高度={effective_product_height:.5f}m, 来源={product_height_source}")
        print(f"产品顶部Z={product_top_z:.5f}m")
        print(f"名义Grasp Z={nominal_grasp_z:.5f}m, 桌面安全下限={table_floor:.5f}m, 最终Grasp Z={safe_grasp_z:.5f}m")
        print(f"最终Grasp Z来源={grasp_z_source}")'''
if old not in s:
    raise RuntimeError("未找到77版高度日志输出")
s = s.replace(old, new, 1)

# 报告配置中记录新模型。
anchor = '''            "safe_height_control": {'''
if anchor not in s:
    raise RuntimeError("未找到safe_height_control报告配置")
insert = '''            "product_height_model": {
                "nominal_height_m": PRODUCT_NOMINAL_HEIGHT_M,
                "valid_measured_min_m": PRODUCT_HEIGHT_VALID_MIN_M,
                "valid_measured_max_m": PRODUCT_HEIGHT_VALID_MAX_M,
                "grasp_offset_above_product_top_m": GRASP_ABOVE_PRODUCT_TOP_M,
                "fallback_when_depth_height_invalid": True,
            },
            "autorun": {
                "enabled": True,
                "manual_keywords_disabled": ["PICK", "NEXT", "TURN", "RELEASE"],
                "safety_checks_retained": True,
            },
            "safe_height_control": {'''
s = s.replace(anchor, insert, 1)

# 启动时明确显示全自动与高度模型。
startup_anchor = '''        print(f"REAL={ENABLE_REAL_MOTION}, ANGLE={ENABLE_ANGLE_ALIGNMENT}, XY={ENABLE_XY_AND_PREGRASP}, PICK={ENABLE_GRIPPER_AND_PICK}")'''
startup_new = '''        print(f"REAL={ENABLE_REAL_MOTION}, ANGLE={ENABLE_ANGLE_ALIGNMENT}, XY={ENABLE_XY_AND_PREGRASP}, PICK={ENABLE_GRIPPER_AND_PICK}")
        print("AUTO_RUN=True: 已取消PICK/NEXT/TURN/RELEASE人工输入")
        print(f"产品高度模型: 标称={PRODUCT_NOMINAL_HEIGHT_M:.3f}m, 有效测量范围={PRODUCT_HEIGHT_VALID_MIN_M:.3f}~{PRODUCT_HEIGHT_VALID_MAX_M:.3f}m")
        print(f"最终抓取TCP相对产品顶部偏移={GRASP_ABOVE_PRODUCT_TOP_M:.3f}m")'''
if startup_anchor not in s:
    raise RuntimeError("未找到启动状态输出")
s = s.replace(startup_anchor, startup_new, 1)

# 若主入口仍有未受控的PICK确认，改为自动提示。
pick_patterns = [
    '''        if input("确认急停可用，输入 PICK：").strip()!="PICK": return''',
    '''        if input("确认急停可用、路径无障碍后输入 PICK：").strip()!="PICK": return''',
]
for pattern in pick_patterns:
    if pattern in s:
        s = s.replace(
            pattern,
            '''        print("[AUTO] 急停、路径与工作区必须已在启动前确认；程序继续执行")''',
            1,
        )

# 如果还有直接input，防止生成一个伪全自动版本。
direct_inputs = [
    line.strip()
    for line in s.splitlines()
    if "input(" in line and not line.lstrip().startswith("#")
]
# REQUIRE_CONFIRMATION=False时分支内input不会执行，因此仅记录，不拒绝生成。

s += '''\n# 82版生成检查信息\n# REQUIRE_CONFIRMATION=False，因此确认分支中的input不会执行。\n'''
dst.write_text(s, encoding="utf-8")
print(f"[OK] generated: {dst}")
print(f"[INFO] remaining guarded input lines: {len(direct_inputs)}")
