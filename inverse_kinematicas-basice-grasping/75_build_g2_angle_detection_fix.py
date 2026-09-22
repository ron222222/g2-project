#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
75_build_g2_angle_detection_fix.py

放到与 74_g2_pick_waist_right80_segmented_release.py 相同目录运行，生成：
75_g2_pick_waist_right80_angle_fix.py

仅修正视觉角度识别：
1. ROI_PADDING_RATIO 真正用于扩展YOLO框。
2. 位置达到8帧后继续等待角度样本，直到角度达到3帧或超时。
3. 增加轮廓拒绝原因统计。
4. 放宽初始轮廓参数，优先恢复正确产品长轴。
5. 保留74版的高度、抓取、边缘显示、腰部右转分段及松爪逻辑。

本生成器不控制机器人。
"""
from pathlib import Path

SOURCE = "74_g2_pick_waist_right80_segmented_release.py"
OUTPUT = "75_g2_pick_waist_right80_angle_fix.py"
root = Path(__file__).resolve().parent
src = root / SOURCE
dst = root / OUTPUT
if not src.exists():
    raise RuntimeError(f"缺少源程序: {src}")

s = src.read_text(encoding="utf-8")
s = s.replace(SOURCE, OUTPUT)
s = s.replace(
    "g2_pick_waist_right80_segmented_release_report.json",
    "g2_pick_waist_right80_angle_fix_report.json",
)
s = s.replace(
    "g2_pick_waist_right80_segmented_release_live.jpg",
    "g2_pick_waist_right80_angle_fix_live.jpg",
)
s = s.replace(
    "g2_pick_waist_right80_segmented_release_angle.jpg",
    "g2_pick_waist_right80_angle_fix_angle.jpg",
)

# 统一替换视觉参数块。
parameter_pairs = {
    "MIN_CONFIDENCE = 0.1": "MIN_CONFIDENCE = 0.10",
    "DETECTION_TIMEOUT_S = 25.0": "DETECTION_TIMEOUT_S = 35.0",
    "MIN_ASPECT_RATIO = 1.40": "MIN_ASPECT_RATIO = 1.30",
    "MIN_CONTOUR_AREA_RATIO = 0.025": "MIN_CONTOUR_AREA_RATIO = 0.015",
    "MAX_CONTOUR_AREA_RATIO = 0.80": "MAX_CONTOUR_AREA_RATIO = 0.85",
    "MAX_CONTOUR_CENTER_DISTANCE_RATIO = 0.40": "MAX_CONTOUR_CENTER_DISTANCE_RATIO = 0.45",
    "CONTOUR_BORDER_MARGIN_PX = 1": "CONTOUR_BORDER_MARGIN_PX = 0",
    "ROI_PADDING_RATIO = 0.12": "ROI_PADDING_RATIO = 0.15",
}
for old, new in parameter_pairs.items():
    if old not in s:
        raise RuntimeError(f"未找到参数: {old}")
    s = s.replace(old, new, 1)

# 初始化角度拒绝统计。
old = '''pos=[]; angles=[]; table_z_samples=[]; rejects={"no_image":0,"no_detection":0,"no_depth":0,"no_angle":0,"no_table":0}'''
new = '''pos=[]; angles=[]; table_z_samples=[]
    rejects={"no_image":0,"no_detection":0,"no_depth":0,"no_angle":0,"no_table":0}
    angle_rejects={
        "area_too_small":0,
        "area_too_large":0,
        "touches_border":0,
        "aspect_too_small":0,
        "center_too_far":0,
        "no_valid_contour":0,
        "invalid_axis":0,
    }'''
if old not in s:
    raise RuntimeError("未找到视觉统计初始化段")
s = s.replace(old, new, 1)

# 等待位置和角度两类样本。
old = '''while time.time()<deadline and len(pos)<DETECTION_SAMPLES:'''
new = '''while time.time()<deadline and (
        len(pos)<DETECTION_SAMPLES or len(angles)<MIN_ANGLE_SAMPLES
    ):'''
if old not in s:
    raise RuntimeError("未找到视觉采样循环")
s = s.replace(old, new, 1)

# 让ROI_PADDING_RATIO真正生效。
old = '''x1,y1,x2,y2=map(int,box.xyxy[0].tolist()); h,w=color.shape[:2]
        x1=max(0,x1);y1=max(0,y1);x2=min(w-1,x2);y2=min(h-1,y2)'''
new = '''raw_x1,raw_y1,raw_x2,raw_y2=map(int,box.xyxy[0].tolist()); h,w=color.shape[:2]
        box_width=max(1,raw_x2-raw_x1)
        box_height=max(1,raw_y2-raw_y1)
        padding_x=int(round(box_width*ROI_PADDING_RATIO))
        padding_y=int(round(box_height*ROI_PADDING_RATIO))
        x1=max(0,raw_x1-padding_x)
        y1=max(0,raw_y1-padding_y)
        x2=min(w-1,raw_x2+padding_x)
        y2=min(h-1,raw_y2+padding_y)'''
if old not in s:
    raise RuntimeError("未找到YOLO框裁剪段")
s = s.replace(old, new, 1)

# 位置达到目标后不再无限追加，但继续尝试角度。
old = '''        pos.append(base.tolist())'''
new = '''        if len(pos)<DETECTION_SAMPLES:
            pos.append(base.tolist())'''
if old not in s:
    raise RuntimeError("未找到位置样本追加段")
s = s.replace(old, new, 1)

# 为各轮廓过滤条件增加拒绝统计。
replacements = [
    (
        'if area_ratio<MIN_CONTOUR_AREA_RATIO: continue',
        'if area_ratio<MIN_CONTOUR_AREA_RATIO:\n                    angle_rejects["area_too_small"]+=1\n                    continue',
    ),
    (
        'if area_ratio>MAX_CONTOUR_AREA_RATIO: continue',
        'if area_ratio>MAX_CONTOUR_AREA_RATIO:\n                    angle_rejects["area_too_large"]+=1\n                    continue',
    ),
    (
        'if touches_border: continue',
        'if touches_border:\n                    angle_rejects["touches_border"]+=1\n                    continue',
    ),
    (
        'if min(rw,rh)<3: continue',
        'if min(rw,rh)<3:\n                    angle_rejects["aspect_too_small"]+=1\n                    continue',
    ),
    (
        'if aspect_ratio<MIN_ASPECT_RATIO: continue',
        'if aspect_ratio<MIN_ASPECT_RATIO:\n                    angle_rejects["aspect_too_small"]+=1\n                    continue',
    ),
    (
        'if center_distance_ratio>MAX_CONTOUR_CENTER_DISTANCE_RATIO: continue',
        'if center_distance_ratio>MAX_CONTOUR_CENTER_DISTANCE_RATIO:\n                    angle_rejects["center_too_far"]+=1\n                    continue',
    ),
]
for old, new in replacements:
    if old not in s:
        raise RuntimeError(f"未找到轮廓条件: {old}")
    s = s.replace(old, new, 1)

# 记录无有效轮廓和无效轴。
old = '''            else: rejects["no_angle"]+=1
        else: rejects["no_angle"]+=1'''
new = '''            else:
                rejects["no_angle"]+=1
                angle_rejects["invalid_axis"]+=1
        else:
            rejects["no_angle"]+=1
            angle_rejects["no_valid_contour"]+=1'''
if old not in s:
    raise RuntimeError("未找到角度失败分支")
s = s.replace(old, new, 1)

# 报告中增加诊断。
old = '''report["vision"]={"position_samples":len(pos),"angle_samples":len(angles),"rejects":rejects};save(report)'''
new = '''report["vision"]={
        "position_samples":len(pos),
        "angle_samples":len(angles),
        "rejects":rejects,
        "angle_rejects":angle_rejects,
        "roi_padding_ratio":ROI_PADDING_RATIO,
    };save(report)'''
if old not in s:
    raise RuntimeError("未找到vision报告段")
s = s.replace(old, new, 1)

# 返回数据带诊断。
old = '''"table_z_m":table_z,"table_samples":len(table_z_samples)}'''
new = '''"table_z_m":table_z,"table_samples":len(table_z_samples),
          "rejects":rejects,"angle_rejects":angle_rejects}'''
if old not in s:
    raise RuntimeError("未找到视觉返回数据段")
s = s.replace(old, new, 1)

# 报错前打印详细诊断。
old = '''        if not det["angle_valid"]: raise RuntimeError("产品角度样本不足")'''
new = '''        if not det["angle_valid"]:
            print("角度识别诊断:")
            print(f"  位置样本={det.get('position_samples',0)}")
            print(f"  角度样本={det.get('angle_samples',0)}")
            print(f"  视觉拒绝统计={det.get('rejects',{})}")
            print(f"  轮廓拒绝统计={det.get('angle_rejects',{})}")
            raise RuntimeError(
                "产品位置和Z有效，但产品边缘角度样本不足，禁止执行抓取"
            )'''
if old not in s:
    raise RuntimeError("未找到angle_valid检查")
s = s.replace(old, new, 1)

# 启动打印参数。
anchor = '''        print(f"边缘过滤: aspect>={MIN_ASPECT_RATIO:.2f}, area={MIN_CONTOUR_AREA_RATIO:.3f}~{MAX_CONTOUR_AREA_RATIO:.2f}, center<={MAX_CONTOUR_CENTER_DISTANCE_RATIO:.2f}, border>{CONTOUR_BORDER_MARGIN_PX}px")'''
new = '''        print(f"角度采样: 位置目标={DETECTION_SAMPLES}, 角度最低={MIN_ANGLE_SAMPLES}, 超时={DETECTION_TIMEOUT_S:.1f}s")
        print(f"边缘过滤: aspect>={MIN_ASPECT_RATIO:.2f}, area={MIN_CONTOUR_AREA_RATIO:.3f}~{MAX_CONTOUR_AREA_RATIO:.2f}, center<={MAX_CONTOUR_CENTER_DISTANCE_RATIO:.2f}, border>{CONTOUR_BORDER_MARGIN_PX}px, ROI padding={ROI_PADDING_RATIO:.2f}")'''
if anchor not in s:
    raise RuntimeError("未找到启动边缘参数输出")
s = s.replace(anchor, new, 1)

dst.write_text(s, encoding="utf-8")
print(f"[OK] generated: {dst}")
