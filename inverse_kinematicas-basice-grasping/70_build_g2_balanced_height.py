#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""基于同目录69_g2_safe_height_vertical_gripper.py生成70平衡高度版。生成器不控制机器人。"""
from pathlib import Path
src=Path(__file__).resolve().with_name('69_g2_safe_height_vertical_gripper.py')
dst=Path(__file__).resolve().with_name('70_g2_balanced_height_vertical_gripper.py')
if not src.exists():
    raise RuntimeError(f'缺少源程序: {src}')
s=src.read_text(encoding='utf-8')
s=s.replace('69_g2_safe_height_vertical_gripper.py','70_g2_balanced_height_vertical_gripper.py')
s=s.replace('g2_safe_height_vertical_gripper_report.json','g2_balanced_height_vertical_gripper_report.json')
s=s.replace('g2_safe_height_vertical_live.jpg','g2_balanced_height_vertical_live.jpg')
s=s.replace('g2_safe_height_vertical_angle.jpg','g2_balanced_height_vertical_angle.jpg')
old='''GRASP_ABOVE_PRODUCT_M = 0.055  # 比68版提高20mm，避免指尖接近桌面
MIN_TCP_ABOVE_TABLE_M = 0.055    # 自动桌面估计后的TCP安全余量'''
new='''GRASP_ABOVE_PRODUCT_M = 0.045  # 35mm会碰桌，55mm偏高，取中间45mm
MIN_TCP_ABOVE_TABLE_M = 0.040    # 桌面安全下限改为桌面Z+40mm'''
if old not in s: raise RuntimeError('未找到69版高度参数')
s=s.replace(old,new,1)
s=s.replace('''        safe_grasp_z=max(nominal_grasp_z,table_floor)
        pre=[product[0],product[1],max(product[2]+PREGRASP_ABOVE_PRODUCT_M,safe_grasp_z+0.060)]''','''        safe_grasp_z=max(nominal_grasp_z,table_floor)
        grasp_z_source=("product_z_plus_45mm" if nominal_grasp_z>=table_floor
                        else "table_z_plus_40mm_safety_floor")
        pre=[product[0],product[1],max(product[2]+PREGRASP_ABOVE_PRODUCT_M,safe_grasp_z+0.060)]''',1)
s=s.replace('''                       "safe_grasp_z":safe_grasp_z});save(report)''','''                       "safe_grasp_z":safe_grasp_z,
                       "grasp_z_source":grasp_z_source});save(report)''',1)
s=s.replace('''        print(f"名义Grasp Z={nominal_grasp_z:.5f}m, 桌面安全下限={table_floor:.5f}m, 最终Grasp Z={safe_grasp_z:.5f}m")''','''        print(f"名义Grasp Z={nominal_grasp_z:.5f}m, 桌面安全下限={table_floor:.5f}m, 最终Grasp Z={safe_grasp_z:.5f}m")
        print(f"最终Grasp Z来源={grasp_z_source}")''',1)
s=s.replace('''        print("安全高度: 名义产品上方55mm，并受桌面Z+55mm硬下限保护")''','''        print("平衡高度: 名义产品上方45mm，桌面Z+40mm硬下限保护")
        print("现场区间: 35mm会碰桌，55mm偏高，当前选择45mm")''',1)
dst.write_text(s,encoding='utf-8')
print(f'[OK] generated: {dst}')
