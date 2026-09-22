#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
83_build_g2_autorun_height40_robust.py

兼容不同结构的77版，生成完整运行文件：
    83_g2_pick_autorun_height40.py

改动：
- 取消 PICK / NEXT / TURN / RELEASE 人工输入
- 产品标称高度40mm
- 视觉高度在25~55mm时采用实测，否则回退40mm
- Grasp Z = table_z + effective_product_height + top_offset
- PreGrasp Z同样以产品顶部为基准
- 保留77版原有运动、安全检查、固定锚点和后处理动作

本生成器本身不控制机器人。
"""
from pathlib import Path
import re

SOURCE = "77_g2_pick_fixed_anchor_orientation.py"
OUTPUT = "83_g2_pick_autorun_height40.py"
root = Path(__file__).resolve().parent
src = root / SOURCE
dst = root / OUTPUT
if not src.exists():
    raise RuntimeError(f"缺少源程序: {src}")

s = src.read_text(encoding="utf-8")
s = s.replace(SOURCE, OUTPUT)
for old, new in [
    ("g2_pick_fixed_anchor_orientation_report.json", "g2_pick_autorun_height40_report.json"),
    ("g2_pick_fixed_anchor_orientation_live.jpg", "g2_pick_autorun_height40_live.jpg"),
    ("g2_pick_fixed_anchor_orientation_angle.jpg", "g2_pick_autorun_height40_angle.jpg"),
]:
    s = s.replace(old, new)

# -----------------------------------------------------------------------------
# 1. 自动运行
# -----------------------------------------------------------------------------
s, n_confirm = re.subn(
    r"^REQUIRE_CONFIRMATION\s*=\s*True.*$",
    "REQUIRE_CONFIRMATION = False  # 83版全自动，不等待PICK/NEXT/TURN/RELEASE",
    s,
    count=1,
    flags=re.MULTILINE,
)
if n_confirm == 0:
    s = s.replace(
        "SHOW_YOLO_WINDOW = True",
        "SHOW_YOLO_WINDOW = True\nREQUIRE_CONFIRMATION = False  # 83版全自动",
        1,
    )

# 主流程中常见的未受REQUIRE_CONFIRMATION保护的PICK输入。
s = re.sub(
    r"\n\s*if input\([^\n]*PICK[^\n]*\)\.strip\(\)\s*!=\s*[\"']PICK[\"']\s*:\s*return",
    '\n        print("[AUTO] 取消PICK输入，继续执行；急停和路径必须在启动前确认")',
    s,
    count=1,
)

# -----------------------------------------------------------------------------
# 2. 参数区：新增产品高度模型
# -----------------------------------------------------------------------------
# 保留PreGrasp原值；抓取偏移重新定义为相对产品顶部。
s = re.sub(
    r"^GRASP_ABOVE_PRODUCT_M\s*=.*$",
    "GRASP_ABOVE_PRODUCT_TOP_M = 0.000  # TCP相对产品顶部偏移\nGRASP_ABOVE_PRODUCT_M = GRASP_ABOVE_PRODUCT_TOP_M  # 兼容旧代码",
    s,
    count=1,
    flags=re.MULTILINE,
)

parameter_anchor = re.search(r"^PREGRASP_ABOVE_PRODUCT_M\s*=.*$", s, re.MULTILINE)
if not parameter_anchor:
    raise RuntimeError("未找到 PREGRASP_ABOVE_PRODUCT_M")
insert_at = parameter_anchor.end()
height_constants = '''
# 83版产品高度模型
PRODUCT_NOMINAL_HEIGHT_M = 0.040
PRODUCT_HEIGHT_VALID_MIN_M = 0.025
PRODUCT_HEIGHT_VALID_MAX_M = 0.055
'''
if "PRODUCT_NOMINAL_HEIGHT_M" not in s:
    s = s[:insert_at] + height_constants + s[insert_at:]

# -----------------------------------------------------------------------------
# 3. 替换高度计算核心，兼容不同格式
# -----------------------------------------------------------------------------
# 从 table_floor 这一行开始，到 grasp=[...] 结束。仅替换首次出现。
core_pattern = re.compile(
    r"(?P<indent>^[ \t]*)table_floor\s*=\s*det\[[\"']table_z_m[\"']\]\s*\+\s*MIN_TCP_ABOVE_TABLE_M.*?"
    r"^[ \t]*grasp\s*=\s*\[[^\n]*\]",
    re.MULTILINE | re.DOTALL,
)
match = core_pattern.search(s)
if not match:
    raise RuntimeError("未找到77版高度计算核心：table_floor ... grasp=[...]")
indent = match.group("indent")
new_core = f'''{indent}table_z=float(det["table_z_m"])
{indent}measured_product_height=float(product[2]-table_z)
{indent}if PRODUCT_HEIGHT_VALID_MIN_M <= measured_product_height <= PRODUCT_HEIGHT_VALID_MAX_M:
{indent}    effective_product_height=measured_product_height
{indent}    product_height_source="measured_product_height"
{indent}else:
{indent}    effective_product_height=PRODUCT_NOMINAL_HEIGHT_M
{indent}    product_height_source="nominal_product_height_40mm"

{indent}product_top_z=table_z+effective_product_height
{indent}table_floor=table_z+MIN_TCP_ABOVE_TABLE_M
{indent}nominal_grasp_z=product_top_z+GRASP_ABOVE_PRODUCT_TOP_M
{indent}safe_grasp_z=max(nominal_grasp_z,table_floor)
{indent}grasp_z_source=(
{indent}    "product_top_z_plus_offset"
{indent}    if nominal_grasp_z>=table_floor
{indent}    else "table_safety_floor"
{indent})
{indent}pre=[
{indent}    product[0],product[1],
{indent}    max(product_top_z+PREGRASP_ABOVE_PRODUCT_M,safe_grasp_z+0.060),
{indent}]
{indent}grasp=[product[0],product[1],safe_grasp_z]'''
s = s[:match.start()] + new_core + s[match.end():]

# -----------------------------------------------------------------------------
# 4. 报告字段：只扩展 report.update，不依赖旧报告字段的精确文本
# -----------------------------------------------------------------------------
report_update_pattern = re.compile(
    r"report\.update\(\{(?P<body>.*?)\}\);save\(report\)",
    re.DOTALL,
)
for candidate in report_update_pattern.finditer(s):
    body = candidate.group("body")
    if '"detection"' in body and '"product"' in body:
        replacement_body = body.rstrip()
        if replacement_body and not replacement_body.rstrip().endswith(","):
            replacement_body += ","
        replacement_body += '''
                       "table_z_m":table_z,
                       "measured_product_height_m":measured_product_height,
                       "effective_product_height_m":effective_product_height,
                       "product_height_source":product_height_source,
                       "product_top_z":product_top_z,
                       "table_floor_tcp_z":table_floor,
                       "nominal_grasp_z":nominal_grasp_z,
                       "safe_grasp_z":safe_grasp_z,
                       "grasp_z_source":grasp_z_source'''
        replacement = "report.update({" + replacement_body + "});save(report)"
        s = s[:candidate.start()] + replacement + s[candidate.end():]
        break

# -----------------------------------------------------------------------------
# 5. 在产品/PreGrasp输出之后插入清晰日志，不依赖旧日志格式
# -----------------------------------------------------------------------------
log_anchor = re.search(
    r'^[ \t]*print\(f["\']产品=\{np\.round\(product,5\)\.tolist\(\)\}, PreGrasp=.*$',
    s,
    re.MULTILINE,
)
if not log_anchor:
    # 兼容空格版本
    log_anchor = re.search(r'^[ \t]*print\(f["\']产品=.*PreGrasp=.*$', s, re.MULTILINE)
if log_anchor:
    line = log_anchor.group(0)
    indent = line[:len(line)-len(line.lstrip())]
    extra = f'''
{indent}print(f"桌面Z={{table_z:.5f}}m, 视觉测量产品高度={{measured_product_height:.5f}}m")
{indent}print(f"有效产品高度={{effective_product_height:.5f}}m, 来源={{product_height_source}}")
{indent}print(f"产品顶部Z={{product_top_z:.5f}}m")
{indent}print(f"名义Grasp Z={{nominal_grasp_z:.5f}}m, 桌面安全下限={{table_floor:.5f}}m, 最终Grasp Z={{safe_grasp_z:.5f}}m")
{indent}print(f"最终Grasp Z来源={{grasp_z_source}}")'''
    s = s[:log_anchor.end()] + extra + s[log_anchor.end():]

# -----------------------------------------------------------------------------
# 6. 启动信息
# -----------------------------------------------------------------------------
main_marker = re.search(r'^def main\(\):', s, re.MULTILINE)
if main_marker:
    # 在第一个分隔线输出后插入并非必要；直接在main起始添加打印会早于初始化。
    pass

# 删除旧的可能误导的固定高度来源字符串，仅修改显示文本。
s = s.replace("product_z_plus_45mm", "legacy_product_z_source_disabled")
s = s.replace("table_z_plus_40mm_safety_floor", "table_safety_floor")

# -----------------------------------------------------------------------------
# 7. 生成后自检
# -----------------------------------------------------------------------------
required_tokens = [
    "PRODUCT_NOMINAL_HEIGHT_M = 0.040",
    "measured_product_height=float(product[2]-table_z)",
    "product_top_z=table_z+effective_product_height",
    "nominal_grasp_z=product_top_z+GRASP_ABOVE_PRODUCT_TOP_M",
    "REQUIRE_CONFIRMATION = False",
]
missing = [token for token in required_tokens if token not in s]
if missing:
    raise RuntimeError(f"生成后自检失败，缺少: {missing}")

# 标记版本说明。
s += '''\n# 83版说明：全自动；产品高度模型40mm；抓取Z基于桌面+有效产品高度。\n'''
dst.write_text(s, encoding="utf-8")
print(f"[OK] generated: {dst}")
print("[OK] 产品高度模型=40mm；实测有效范围=25~55mm")
print("[OK] REQUIRE_CONFIRMATION=False")
