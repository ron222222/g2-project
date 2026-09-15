#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
20_tf_viewer.py
G2 TF学习工具（只读）
用途：查看 base_link、head_link3、arm_r_end_link、arm_l_end_link 之间的关系。
不控制机器人。
"""
import time
import agibot_gdk

FRAMES=["head_link3","arm_r_end_link","arm_l_end_link"]


def show_tf(tf_api, frame):
    try:
        t=tf_api.get_tf_from_base_link(frame)
        print(f"
base_link -> {frame}")
        print(f"  XYZ(m): ({t.translation.x:.3f}, {t.translation.y:.3f}, {t.translation.z:.3f})")
        print(f"  Quaternion: ({t.rotation.x:.4f}, {t.rotation.y:.4f}, {t.rotation.z:.4f}, {t.rotation.w:.4f})")
    except Exception as e:
        print(f"读取 {frame} 失败: {e}")


def main():
    if agibot_gdk.gdk_init()!=agibot_gdk.GDKRes.kSuccess:
        print('GDK初始化失败')
        return

    tf=agibot_gdk.TF()
    time.sleep(2)

    print('='*60)
    print('G2 TF Viewer (READ ONLY)')
    print('base_link
 ↑
head_link3
 ↑
camera
 ↑
Camera XYZ
 ↑
RGB Pixel')
    print('='*60)

    while True:
        for f in FRAMES:
            show_tf(tf,f)
        print('
按 Ctrl+C 退出，尝试转头/低头观察head_link3变化')
        print('-'*60)
        time.sleep(3)

if __name__=='__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('
退出TF Viewer')
        agibot_gdk.gdk_release()
