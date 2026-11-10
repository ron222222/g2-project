#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
# ======================================================================================
# 90版 G2 抓取程序：超级教学中文注释版
# ======================================================================================
# 本文件面向 Python 初学者和 G2 机器人开发复盘。
# 本次只增加注释，不修改任何可执行语句、参数值、函数顺序或动作逻辑。
#
# 一、先记住5个Python符号
# 1. =    赋值：把右边结果保存到左边变量，不是数学等号证明。
# 2. ==   比较：判断左右是否相等，结果为True或False。
# 3. []   列表或索引：例如product[2]取产品Z。
# 4. {}   字典：用“键:值”保存报告，如report["status"]。
# 5. ()   函数调用或参数：例如move_arm_once(...)会执行函数。
#
# 二、4个核心数据概念
# 1. position=[x,y,z]：物体/工具在哪里，单位通常为米。
# 2. orientation=[x,y,z,w]：工具朝哪里，使用四元数，不是位置。
# 3. PoseData：把position和orientation放在一个对象里。
# 4. TF：不同坐标系之间的旋转和平移关系。
#
# 三、机器人总流程
# 启动检查 → HOME/WP1/WP2 → YOLO稳定门控 → 固定高度规划
# → XY粗对准 → 固定TCP锚点姿态 → XY精修 → PreGrasp
# → 打开夹爪 → Grasp → 闭合 → 抬升 → 腰部右转 → 释放
# → 腰部返回 → WP2 → WP1 → 下一轮。
#
# 四、阅读方法
# 第一步搜索“def main”，先理解总流程。
# 第二步看到函数调用，例如detect_product(...)，再搜索函数定义。
# 第三步看每个函数上方的【函数教学卡】，理解输入、输出和上下游。
# 第四步最后再看四元数、矩阵和插值公式，不要一开始陷入数学细节。
#
# 五、安全提醒
# 注释版仍保留原文件 ENABLE_REAL_MOTION=True 和 CYCLE_COUNT=10。
# 运行该文件会真实控制G2。注释不构成新的碰撞检测、急停或恢复功能。
# ======================================================================================
# ==================== 中文学习注释版阅读导航 ====================
# 本文件仅增加#注释；原参数、语句、字符串、函数和执行顺序保持不变。
# 阅读顺序：先看main总流程，再看参数区，最后按函数调用逐层展开。
# 单位：XYZ通常是米；关节位置是弧度；日志中deg是度；四元数顺序[x,y,z,w]。
# main -> move_arm_once/ensure_head -> detect_product -> detect_product_batch。
# main -> 高度计算/等价长轴选择 -> move_tcp_closed_loop -> command_cart_path -> set_pose。
# main -> align_orientation_target -> slerp/end_from_tcp/set_pose -> 实际TF验证。
# main -> set_gripper/抬升 -> turn_release_and_return_waist -> move_waist_segmented_keep_arms。
# 每轮最后回WP2/WP1，下一轮重新检测；finally关闭窗口、相机并释放GDK。
# 注意：注释不是新增安全机制；原程序的安全限制与局限均保持原样。
# 注释版保留真实运动=True、循环10次等原配置；阅读或运行前务必检查。
# ===============================================================
# 【原第3行】原有说明字符串，保留原文；历史描述不一定等于当前参数。
"""
90_g2_pick_wait_until_product_stable_full_yolo.py

独立完整单文件，不依赖52/53/58/62等本地程序。

流程：
HOME -> WAYPOINT_1（一次到点） -> 头部idx13 +20度
-> WAYPOINT_2（一次到点） -> YOLO中心/长轴/深度/TF
-> 高位分段角度对齐 -> 高位XY闭环对准
-> 下降至PreGrasp -> 打开右夹爪 -> 下降至Grasp
-> 闭合右夹爪 -> 抬升10cm。

本版重点修复：
1. 高位XY首次误差25~50mm时，最多执行2次剩余误差闭环修正。
2. 正常通过门限仍为25mm，不简单放宽。
3. 误差超过50mm立即停止，禁止自动追赶。
4. 夹爪扁长指尖使用局部Y轴。
5. Grasp TCP位于产品视觉表面上方25mm，即相对61版提高30mm。
6. 自动准备OpenCV Qt字体目录，并实时显示/保存YOLO画面。

基于已验证85版：完整抓取、右转80度释放、腰部反转80度、右臂回WAYPOINT_2和WAYPOINT_1，共循环3次。
"""

# 【原第26行】导入依赖：import os；供后续函数使用。OS=Operating System,操作系统,它是 Python 用来和：# Ubuntu# Linux# Windows#的交互的工具箱
# 【导入教学】
# Python标准库的操作系统工具；本程序用于目录、文件链接和环境变量。
# import只把工具带进当前文件；是否真正执行功能，要看后续是否调用。
import os

# OpenCV Qt会查找该固定目录。在import cv2前创建字体链接。
# 【原第29行】计算/保存 SYSTEM_FONT_DIR：'/usr/share/fonts/truetype/dejavu'。
SYSTEM_FONT_DIR = "/usr/share/fonts/truetype/dejavu"
# 【原第30行】计算/保存 CV2_QT_FONT_DIR：os.path.expanduser('~/.local/lib/python3.10/site-packages/cv2/qt/fonts')。
CV2_QT_FONT_DIR = os.path.expanduser(
    # 【原第31行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    "~/.local/lib/python3.10/site-packages/cv2/qt/fonts"
# 【原第32行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
)

# 【原第34行】准备OpenCV字体目录和链接；模块导入时调用，不控制机器人。
# ======================================================================================
# 【函数教学卡】prepare_qt_fonts()
# 1. 功能：在导入OpenCV并创建窗口前，准备Qt可访问的字体目录，减少字体缺失导致的窗口警告或显示问题。
# 2. 输入：无显式参数。函数内部读取SYSTEM_FONT_DIR和CV2_QT_FONT_DIR两个全局路径。
# 3. 输出：返回字典result，包含created、linked、error三个字段。
# 4. 谁调用：文件加载阶段直接执行QT_FONT_RESULT = prepare_qt_fonts()。它发生在main()之前。
# 5. 结果流向：返回值写入QT_FONT_RESULT，随后main把它写入运行报告report["qt_font"]。
# 6. G2实际意义：不控制G2，不读取相机，不运行YOLO，只处理Ubuntu文件和环境变量。
# 7. 调试重点：重点看result["error"]、目录权限、软链接数量，以及QT_QPA_FONTDIR环境变量。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def prepare_qt_fonts():
    # 【原第35行】计算/保存 result：{'created': False, 'linked': 0, 'error': None}。
    """【函数教学卡：超详细版】
    
    功能：
        准备OpenCV/Qt字体环境，避免YOLO显示窗口因字体目录缺失产生警告。
    
    Args:
        无显式参数；使用全局SYSTEM_FONT_DIR、CV2_QT_FONT_DIR。
    
    Returns:
        Dict：created表示是否创建目录，linked表示建立字体链接数量，error保存异常文本或None。
    
    调用时机：
        模块加载时，在main()之前调用。
    
    上下游逻辑：
        不控制机器人；只处理Ubuntu目录、软链接和环境变量。
    
    注意事项与调试重点：
        权限不足或系统字体不存在时查看error字段。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    result = {"created": False, "linked": 0, "error": None}
    # 【原第36行】开始异常处理块：try执行正常操作，except处理异常，finally执行清理（若有）。
    try:
        # 【原第37行】执行调用/更新：os.makedirs(CV2_QT_FONT_DIR, exist_ok=True)。
        os.makedirs(CV2_QT_FONT_DIR, exist_ok=True)
        # 【原第38行】条件判断：os.path.isdir(SYSTEM_FONT_DIR)；满足时执行下面缩进代码，否则走else（若有）。
        if os.path.isdir(SYSTEM_FONT_DIR):
            # 【原第39行】循环：依次从 os.listdir(SYSTEM_FONT_DIR) 取值到 name，执行下面缩进块。
            for name in os.listdir(SYSTEM_FONT_DIR):
                # 【原第40行】条件判断：not name.lower().endswith(('.ttf', '.otf'))；满足时执行下面缩进代码，否则走else（若有）。
                if not name.lower().endswith((".ttf", ".otf")):
                    # 【原第41行】跳过本轮剩余代码，进入当前循环的下一轮。
                    continue
                # 【原第42行】计算/保存 src：os.path.join(SYSTEM_FONT_DIR, name)。
                src = os.path.join(SYSTEM_FONT_DIR, name)
                # 【原第43行】计算/保存 dst：os.path.join(CV2_QT_FONT_DIR, name)。
                dst = os.path.join(CV2_QT_FONT_DIR, name)
                # 【原第44行】条件判断：not os.path.lexists(dst)；满足时执行下面缩进代码，否则走else（若有）。
                if not os.path.lexists(dst):
                    # 【原第45行】执行调用/更新：os.symlink(src, dst)。
                    os.symlink(src, dst)
                # 【原第46行】计算/保存 result['linked']：1。
                result["linked"] += 1
        # 【原第47行】计算/保存 os.environ['QT_QPA_FONTDIR']：CV2_QT_FONT_DIR。
        os.environ["QT_QPA_FONTDIR"] = CV2_QT_FONT_DIR
        # 【原第48行】计算/保存 result['created']：True。
        result["created"] = True
    # 【原第49行】异常分支：捕获对应异常，执行下面缩进代码；保持原处理方式。
    except Exception as exc:
        # 【原第50行】计算/保存 result['error']：str(exc)。
        result["error"] = str(exc)
    # 【原第51行】返回结果给调用者：result。
    return result

# 【原第53行】计算/保存 QT_FONT_RESULT：prepare_qt_fonts()。
QT_FONT_RESULT = prepare_qt_fonts()

# 【原第55行】导入依赖：import copy；供后续函数使用。
# 【导入教学】
# 复制工具；deepcopy用于把本轮列表/字典快照放进报告，避免后续修改影响旧数据。
# import只把工具带进当前文件；是否真正执行功能，要看后续是否调用。
import copy
# 【原第56行】导入依赖：import json；供后续函数使用。
# 【导入教学】
# JSON编码工具；把Python字典转换成人和程序都能读取的报告文本。
# import只把工具带进当前文件；是否真正执行功能，要看后续是否调用。
import json
# 【原第57行】导入依赖：import math；供后续函数使用。
# 【导入教学】
# 数学工具；提供三角函数、弧度/角度换算、平方根和圆周率。
# import只把工具带进当前文件；是否真正执行功能，要看后续是否调用。
import math
# 【原第58行】导入依赖：import time；供后续函数使用。
# 【导入教学】
# 时间工具；用于超时、等待、时间戳和控制循环节奏。
# import只把工具带进当前文件；是否真正执行功能，要看后续是否调用。
import time
# 【原第59行】导入依赖：import statistics；供后续函数使用。
# 【导入教学】
# 统计工具；用于中位数和总体标准差，判断产品多批次是否稳定。
# import只把工具带进当前文件；是否真正执行功能，要看后续是否调用。
import statistics
# 【原第60行】导入依赖：from dataclasses import dataclass；供后续函数使用。
# 【导入教学】
# 数据类装饰器；自动为PoseData生成初始化等基础方法。
# import只把工具带进当前文件；是否真正执行功能，要看后续是否调用。
from dataclasses import dataclass
# 【原第61行】导入依赖：from pathlib import Path；供后续函数使用。
# 【导入教学】
# 面向对象路径工具；本程序检查模型文件并保存JSON。
# import只把工具带进当前文件；是否真正执行功能，要看后续是否调用。
from pathlib import Path
# 【原第62行】导入依赖：from typing import Dict, List, Optional, Tuple；供后续函数使用。
# 【导入教学】
# 类型提示：字典，键和值成对保存。
# 类型提示：列表，元素有顺序且可修改。
# 类型提示：值可以是指定类型，也可以是None。
# 类型提示：固定组合的多个值。
# import只把工具带进当前文件；是否真正执行功能，要看后续是否调用。
from typing import Dict, List, Optional, Tuple

# 【原第64行】导入依赖：import cv2；供后续函数使用。
# 【导入教学】
# OpenCV模块；负责图像解码、轮廓、绘图、窗口和JPG保存。
# import只把工具带进当前文件；是否真正执行功能，要看后续是否调用。
import cv2
# 【原第65行】导入依赖：import numpy as np；供后续函数使用。
# 【导入教学】
# NumPy别名；负责数组、向量、矩阵、统计和线性代数。
# import只把工具带进当前文件；是否真正执行功能，要看后续是否调用。
import numpy as np
# 【原第66行】导入依赖：import agibot_gdk；供后续函数使用。
# 【导入教学】
# G2机器人SDK；提供Robot、TF、Camera、关节和末端控制类型。
# import只把工具带进当前文件；是否真正执行功能，要看后续是否调用。
import agibot_gdk
# 【原第67行】导入依赖：from ultralytics import YOLO；供后续函数使用。
# 【导入教学】
# Ultralytics目标检测类；加载best.pt并在颜色图上推理产品框。
# import只把工具带进当前文件；是否真正执行功能，要看后续是否调用。
from ultralytics import YOLO

# -----------------------------------------------------------------------------
# 阶段开关
# -----------------------------------------------------------------------------
# 【原第72行】真实运动总开关；本附件为True，运行会控制机器人。
# 【关键参数教学】
# 参数名：ENABLE_REAL_MOTION
# 作用：总运动开关。True会真实初始化并控制机器人；False只做DRY RUN基础检查后返回。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
ENABLE_REAL_MOTION = True
# 【原第73行】是否执行完整姿态对齐；关闭后在粗对准后返回。
ENABLE_ANGLE_ALIGNMENT = True
# 【原第74行】是否执行XY和预抓取；关闭时只完成规划。
ENABLE_XY_AND_PREGRASP = True
# 【原第75行】是否继续最终下降、夹取和抬升；关闭时停在PreGrasp。
ENABLE_GRIPPER_AND_PICK = True
# 【原第76行】笛卡尔/姿态阶段人工确认开关；False跳过相关输入。
REQUIRE_CONFIRMATION = False  # 83版全自动，不等待PICK/NEXT/TURN/RELEASE

# -----------------------------------------------------------------------------
# 三循环参数
# -----------------------------------------------------------------------------
# 【原第81行】实际循环次数；本附件为10，不是旧说明中的3。
# 【关键参数教学】
# 参数名：CYCLE_COUNT
# 作用：计划循环数。当前值10意味着最多做10轮，不是文件头历史说明的3轮。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
CYCLE_COUNT = 10
# 每轮释放并返回WAYPOINT_1后，等待人工放入下一件产品。
# True: 仅在第1/2轮之间等待输入 READY；False: 按固定秒数自动继续。
# 【原第84行】轮间READY确认开关；False时下一轮进入视觉位等待稳定。
# 【关键参数教学】
# 参数名：WAIT_FOR_PRODUCT_CONFIRMATION
# 作用：是否轮间要求输入READY。False时到WP2后用稳定门控等待产品。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
WAIT_FOR_PRODUCT_CONFIRMATION = False
# 【原第85行】保留参数；本main的自动轮间分支未调用此值进行固定等待。
INTER_CYCLE_WAIT_S = 4.0
# 【原第86行】保留配置名；当前main每轮都无条件回WP2和WP1。
RETURN_TO_WAYPOINT1_AFTER_FINAL_CYCLE = True

# 【原第88行】计算/保存 MODEL_PATH：'runs/detect/runs/product_detector/weights/best.pt'。
MODEL_PATH = "runs/detect/runs/product_detector/weights/best.pt"
# 【原第89行】计算/保存 REPORT_FILE：'g2_pick_fast_smooth_report.json'。
REPORT_FILE = "g2_pick_fast_smooth_report.json"
# 【原第90行】计算/保存 LIVE_IMAGE_FILE：'g2_pick_fast_smooth_live.jpg'。
LIVE_IMAGE_FILE = "g2_pick_fast_smooth_live.jpg"
# 【原第91行】计算/保存 ANGLE_IMAGE_FILE：'g2_pick_fast_smooth_angle.jpg'。
ANGLE_IMAGE_FILE = "g2_pick_fast_smooth_angle.jpg"
# 【原第92行】计算/保存 WINDOW_NAME：'G2 YOLO Live'。
WINDOW_NAME = "G2 YOLO Live"
# 【原第93行】计算/保存 SHOW_YOLO_WINDOW：True。
SHOW_YOLO_WINDOW = True
# 【原第94行】缓存最后一次识别图像；动作阶段刷新通常使用该缓存。
LAST_YOLO_FRAME = None
# 【原第95行】计算/保存 LAST_YOLO_TEXT：[]。
LAST_YOLO_TEXT = []

# 90版方案C：不依赖固定轮间等待，产品连续稳定后才允许动作。
# 【原第98行】保留配置名；当前detect_product直接执行稳定门控，没有按此开关分支。
WAIT_UNTIL_PRODUCT_STABLE = True
# 【原第99行】要求的连续有效批次数，不是单帧数量。
# 【关键参数教学】
# 参数名：STABILITY_BATCHES_REQUIRED
# 作用：需要连续通过的完整检测批次数。每批内部又包含多帧，不等于3帧。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
STABILITY_BATCHES_REQUIRED = 3
# 【原第100行】单批原始产品Z极差门限：0.015米即15毫米。
# 【关键参数教学】
# 参数名：MAX_STABLE_Z_RANGE_M
# 作用：单批产品原始Z最大值减最小值的门限，0.015m=15mm。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
MAX_STABLE_Z_RANGE_M = 0.015
# 【原第101行】多个批次X/Y标准差上限：0.005米即5毫米。
# 【关键参数教学】
# 参数名：MAX_STABLE_XY_STD_M
# 作用：多个批次中心X/Y总体标准差门限，0.005m=5mm。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
MAX_STABLE_XY_STD_M = 0.005
# 【原第102行】批次长轴相对参考方向的最大差上限，单位度。
# 【关键参数教学】
# 参数名：MAX_STABLE_YAW_SPREAD_DEG
# 作用：多个批次长轴相对首批的最大180°等价角差。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
MAX_STABLE_YAW_SPREAD_DEG = 6.0
# 【原第103行】计算/保存 STABLE_MIN_CONFIDENCE：0.1。
STABLE_MIN_CONFIDENCE = 0.10
# 【原第104行】外层等待时限；内层批次调用可使实际总等待超过该值。
# 【关键参数教学】
# 参数名：STABLE_DETECTION_TIMEOUT_S
# 作用：外层稳定等待目标120秒；单批检测本身可能阻塞，所以不是严格墙钟总时限。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
STABLE_DETECTION_TIMEOUT_S = 120.0
# 【原第105行】计算/保存 STABLE_RETRY_SLEEP_S：0.1。
STABLE_RETRY_SLEEP_S = 0.10

# 三循环固定高度基准。
# 【原第108行】实际高度模型使用的固定产品高度，0.040米即40毫米。
# 【关键参数教学】
# 参数名：FIXED_PRODUCT_HEIGHT_M
# 作用：固定物理产品高度40mm；最终高度不再用视觉产品Z减桌面Z切换模式。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
FIXED_PRODUCT_HEIGHT_M = 0.040
# 【原第109行】是否跨循环锁定桌面Z。
LOCK_TABLE_Z_ACROSS_CYCLES = True
# 【原第110行】原始桌面Z与锁定基准差值上限：0.020米即20毫米。
TABLE_Z_SANITY_DELTA_M = 0.020
# 【原第111行】桌面基准；None时首轮稳定检测后锁定，后续复用。
# 【关键参数教学】
# 参数名：CALIBRATED_TABLE_Z_M
# 作用：None表示首轮稳定后自动锁桌面；同一进程后续循环复用。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
CALIBRATED_TABLE_Z_M = None

# 全程YOLO大屏。
# 【原第114行】计算/保存 FULL_UI_WINDOW_WIDTH：1600。
FULL_UI_WINDOW_WIDTH = 1600
# 【原第115行】计算/保存 FULL_UI_WINDOW_HEIGHT：960。
FULL_UI_WINDOW_HEIGHT = 960
# 【原第116行】计算/保存 FULL_UI_PANEL_WIDTH：500。
FULL_UI_PANEL_WIDTH = 500
# 【原第117行】计算/保存 FULL_UI_IMAGE_FILE：'g2_v90_full_yolo_view.jpg'。
FULL_UI_IMAGE_FILE = "g2_v90_full_yolo_view.jpg"
# 【原第118行】界面状态字典，保存循环、阶段、检测与目标数据。
UI_STATE = {
    # 【原第119行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    "cycle": 0, "stage": "STARTUP", "message": "初始化",
    # 【原第120行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    "confidence": None, "position_samples": 0, "angle_samples": 0,
    # 【原第121行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    "stable_batches": 0, "z_range_m": None, "x_std_m": None,
    # 【原第122行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    "y_std_m": None, "yaw_spread_deg": None, "product_xyz": None,
    # 【原第123行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    "table_z_m": None, "pregrasp_xyz": None, "grasp_xyz": None,
    # 【原第124行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    "yaw_deg": None,
# 【原第125行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
}


# -----------------------------------------------------------------------------
# 坐标系和关节
# -----------------------------------------------------------------------------
# 【原第131行】计算/保存 LEFT_FRAME：'arm_l_end_link'。
LEFT_FRAME = "arm_l_end_link"
# 【原第132行】计算/保存 RIGHT_FRAME：'arm_r_end_link'。
RIGHT_FRAME = "arm_r_end_link"
# 【原第133行】计算/保存 TCP_FRAME：'gripper_r_center_link'。
TCP_FRAME = "gripper_r_center_link"
# 【原第134行】计算/保存 HEAD_FRAME：'head_link3'。
HEAD_FRAME = "head_link3"
# 【原第135行】计算/保存 LEFT_JOINTS：[f'idx2{i}_arm_l_joint{i}' for i in range(1, 8)]。
LEFT_JOINTS = [f"idx2{i}_arm_l_joint{i}" for i in range(1, 8)]
# 【原第136行】计算/保存 RIGHT_JOINTS：[f'idx6{i}_arm_r_joint{i}' for i in range(1, 8)]。
RIGHT_JOINTS = [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)]
# 【原第137行】计算/保存 HEAD_JOINTS：['idx11_head_joint1', 'idx12_head_joint2', 'idx13_head_joint3']。
HEAD_JOINTS = ["idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3"]
# 【原第138行】计算/保存 WAIST_JOINTS：['idx01_body_joint1', 'idx02_body_joint2', 'idx03_body_joint3', 'idx04_body_joint4', 'idx05_body_joint5']。
WAIST_JOINTS = [
    # 【原第139行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    "idx01_body_joint1",
    # 【原第140行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    "idx02_body_joint2",
    # 【原第141行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    "idx03_body_joint3",
    # 【原第142行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    "idx04_body_joint4",
    # 【原第143行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    "idx05_body_joint5",
# 【原第144行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
]
# 【原第145行】计算/保存 WAIST_LIMITS_RAD：[(-1.082104, 0.000174), (-0.000174, 2.6529), (-1.919862, 1.57097), (-0.436332, 0.436332), (-3.045599, 3.045599)]。
WAIST_LIMITS_RAD = [
    # 【原第146行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    (-1.082104, 0.000174),  # idx01: -62.0deg ~ +0.01deg
    # 【原第147行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    (-0.000174, 2.652900), # idx02: -0.01deg ~ +152.0deg
    # 【原第148行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    (-1.919862, 1.570970), # idx03: -110.0deg ~ +90.01deg
    # 【原第149行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    (-0.436332, 0.436332), # idx04: -25.0deg ~ +25.0deg
    # 【原第150行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    (-3.045599, 3.045599), # idx05: -174.5deg ~ +174.5deg
# 【原第151行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
]

# 【原第153行】计算/保存 HOME：[-1.57079643, -1.57079608, 1.57079585, -1.57079645, 2.4e-07, 0.0, 3.6e-07]。
HOME = [-1.57079643, -1.57079608, 1.57079585, -1.57079645,
        # 【原第154行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
        0.00000024, 0.00000000, 0.00000036]
# 【原第155行】计算/保存 WAYPOINT_1：[-1.91304851, 0.20393955, 1.83169374, -1.84651811, 0.29203423, -0.28742294, 0.91643847]。
WAYPOINT_1 = [-1.91304851, 0.20393955, 1.83169374, -1.84651811,
              # 【原第156行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
              0.29203423, -0.28742294, 0.91643847]
# 【原第157行】计算/保存 WAYPOINT_2：[-2.64999915, 0.61000018, 2.39123385, -1.39048167, 0.40846278, -0.35681237, 0.91304288]。
# 【关键参数教学】
# 参数名：WAYPOINT_2
# 作用：视觉工作位7关节实测稳定值；顺序对应RIGHT_JOINTS。修改时必须整组基于真机记录。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
WAYPOINT_2 = [-2.64999915, 0.61000018, 2.39123385, -1.39048167,
              # 【原第158行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
              0.40846278, -0.35681237, 0.91304288]
# 【原第159行】示教记录的参考TCP；WP2改关节后参考坐标可能未同步，本函数仅记录TCP差。
RECORDED_TCP = {
    # 【原第160行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    "home": [0.73185, -0.24350, 0.81644],
    # 【原第161行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    "waypoint_1": [0.58639, -0.34289, 0.93608],
    # 【原第162行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    "waypoint_2": [0.65699, -0.16556, 0.98419],
# 【原第163行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
}

# -----------------------------------------------------------------------------
# 运动参数
# -----------------------------------------------------------------------------
# 【原第168行】计算/保存 DIRECT_ARM_SPEED_RAD_S：0.3。
# 【关键参数教学】
# 参数名：DIRECT_ARM_SPEED_RAD_S
# 作用：WP关节运动速度0.30rad/s。不是笛卡尔TCP速度。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
DIRECT_ARM_SPEED_RAD_S = 0.30
# 【原第169行】计算/保存 JOINT_TOLERANCE_RAD：0.06。
JOINT_TOLERANCE_RAD = 0.06
# 【原第170行】保留参数；move_arm_once当前没有用它作为停止条件。
KEYPOINT_TCP_TOLERANCE_M = 0.050
# 【原第171行】计算/保存 START_POSE_TOLERANCE_RAD：0.15。
START_POSE_TOLERANCE_RAD = 0.15
# 【原第172行】计算/保存 HEAD_TARGET_DEG：20.0。
HEAD_TARGET_DEG = 20.0
# 【原第173行】计算/保存 HEAD_SPEED_RAD_S：0.35。
HEAD_SPEED_RAD_S = 0.35
# 【原第174行】计算/保存 HEAD_TOLERANCE_DEG：1.5。
HEAD_TOLERANCE_DEG = 1.5
# 【原第175行】计算/保存 SETTLE_S：0.4。
SETTLE_S = 0.40

# 【原第177行】计算/保存 JOINT_LIMITS：[(-3.071796, 3.071796), (-2.059505, 2.059505), (-3.071796, 3.071796), (-2.495838, 1.012308), (-3.071796, 3.071796), (-1.012308, 1.012308), (-1.535907,…。
JOINT_LIMITS = [
    # 【原第178行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    (-3.071796, 3.071796), (-2.059505, 2.059505),
    # 【原第179行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    (-3.071796, 3.071796), (-2.495838, 1.012308),
    # 【原第180行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    (-3.071796, 3.071796), (-1.012308, 1.012308),
    # 【原第181行】继续上方跨行表达式的参数/数据项；所属模块：参数配置。
    (-1.535907, 1.535907),
# 【原第182行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
]

# -----------------------------------------------------------------------------
# YOLO/深度/角度
# -----------------------------------------------------------------------------
# 【原第187行】YOLO推理输出阈值；0.10是当前配置，不是通用安全阈值。
# 【关键参数教学】
# 参数名：MIN_CONFIDENCE
# 作用：YOLO最低置信度0.10；稳定门控还使用STABLE_MIN_CONFIDENCE。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
MIN_CONFIDENCE = 0.10
# 【原第188行】计算/保存 DETECTION_SAMPLES：8。
DETECTION_SAMPLES = 8
# 【原第189行】计算/保存 MIN_POSITION_SAMPLES：4。
MIN_POSITION_SAMPLES = 4
# 【原第190行】计算/保存 MIN_ANGLE_SAMPLES：3。
MIN_ANGLE_SAMPLES = 3
# 【原第191行】计算/保存 DETECTION_TIMEOUT_S：35.0。
DETECTION_TIMEOUT_S = 35.0
# 【原第192行】计算/保存 CAMERA_TIMEOUT_MS：1000.0。
CAMERA_TIMEOUT_MS = 1000.0
# 【原第193行】中心深度窗口半径4像素，正常窗口约9×9。
DEPTH_RADIUS = 4
# 【原第194行】计算/保存 MIN_DEPTH_RAW：50.0。
MIN_DEPTH_RAW = 50.0
# 【原第195行】计算/保存 MAX_DEPTH_RAW：10000.0。
MAX_DEPTH_RAW = 10000.0
# 【原第196行】计算/保存 MIN_ASPECT_RATIO：1.3。
MIN_ASPECT_RATIO = 1.30
# 【原第197行】计算/保存 MIN_CONTOUR_AREA_RATIO：0.015。
MIN_CONTOUR_AREA_RATIO = 0.015
# 【原第198行】计算/保存 MAX_CONTOUR_AREA_RATIO：0.85。
MAX_CONTOUR_AREA_RATIO = 0.85
# 【原第199行】计算/保存 MAX_CONTOUR_CENTER_DISTANCE_RATIO：0.45。
MAX_CONTOUR_CENTER_DISTANCE_RATIO = 0.45
# 【原第200行】计算/保存 CONTOUR_BORDER_MARGIN_PX：0。
CONTOUR_BORDER_MARGIN_PX = 0
# 【原第201行】检测框每侧扩展比例，给轮廓留出边缘空间。
ROI_PADDING_RATIO = 0.15

# -----------------------------------------------------------------------------
# 抓取参数
# -----------------------------------------------------------------------------
# 【原第206行】PreGrasp相对计算产品顶部的高度，0.120米即120毫米。
# 【关键参数教学】
# 参数名：PREGRASP_ABOVE_PRODUCT_M
# 作用：预抓取TCP相对计算产品顶部上方120mm。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
PREGRASP_ABOVE_PRODUCT_M = 0.120
# 83版产品高度模型
# 【原第208行】旧高度模型保留值；main实际使用FIXED_PRODUCT_HEIGHT_M。
PRODUCT_NOMINAL_HEIGHT_M = 0.040
# 【原第209行】旧模式切换下限；当前main不再用它决定产品高度。
PRODUCT_HEIGHT_VALID_MIN_M = 0.025
# 【原第210行】旧模式切换上限；当前main不再用它决定产品高度。
PRODUCT_HEIGHT_VALID_MAX_M = 0.055


# 【原第213行】最终TCP相对产品顶部的偏移；-0.023米表示顶部以下23毫米，不是指尖高度。
# 【关键参数教学】
# 参数名：GRASP_ABOVE_PRODUCT_TOP_M
# 作用：最终TCP相对产品顶部-23mm，注意TCP并非夹爪指尖。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
GRASP_ABOVE_PRODUCT_TOP_M = -0.023  # TCP相对产品顶部偏移
# 【原第214行】计算/保存 GRASP_ABOVE_PRODUCT_M：GRASP_ABOVE_PRODUCT_TOP_M。
GRASP_ABOVE_PRODUCT_M = GRASP_ABOVE_PRODUCT_TOP_M  # 兼容旧代码
# 【原第215行】名义TCP桌面下限偏移；0.010米是10毫米，旧注释40毫米不符；不等于完整碰撞检测。
# 【关键参数教学】
# 参数名：MIN_TCP_ABOVE_TABLE_M
# 作用：TCP相对桌面名义下限10mm；原行尾“40mm”是旧注释。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
MIN_TCP_ABOVE_TABLE_M = 0.010    # 桌面安全下限改为桌面Z+40mm
# 【原第216行】旧Z极差参数；稳定门控实际比较MAX_STABLE_Z_RANGE_M。
MAX_PRODUCT_Z_RANGE_M = 0.035    # 8帧产品Z极差超过12mm则禁止抓取
# 【原第217行】计算/保存 MIN_TABLE_SAMPLES：8。
MIN_TABLE_SAMPLES = 8
# 【原第218行】计算/保存 TABLE_RING_MARGIN_RATIO：0.35。
TABLE_RING_MARGIN_RATIO = 0.35
# 【原第219行】计算/保存 VERTICAL_TOOL_TILT_TOLERANCE_DEG：8.0。
VERTICAL_TOOL_TILT_TOLERANCE_DEG = 8.0
# 【原第220行】计算/保存 LIFT_AFTER_GRASP_M：0.1。
LIFT_AFTER_GRASP_M = 0.100
# 【原第221行】计算/保存 ENABLE_WAIST_TURN_AND_RELEASE：True。
ENABLE_WAIST_TURN_AND_RELEASE = True
# 【原第222行】本程序右转使用-80度；以现场确认的idx05方向为准。
# 【关键参数教学】
# 参数名：WAIST_TURN_DEG
# 作用：idx05相对转动-80°，现场定义为右转。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
WAIST_TURN_DEG = -80.0         # 现场验证：idx05正方向为左转，因此右转使用负角度
# 【原第223行】腰部每段最大角度40度；80度通常分两段，旧注释段数不符。
# 【关键参数教学】
# 参数名：WAIST_MAX_STEP_DEG
# 作用：腰部一段最多40°，80°通常分2段。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
WAIST_MAX_STEP_DEG = 40.0  # 80deg由8段减至4段       # 80度拆成8个短目标，避免单次规划超时
# 【原第224行】腰部列表索引4，对应第五个关节idx05。
WAIST_YAW_JOINT_INDEX = 4      # idx05_body_joint5，旋转范围约±174.5deg
# 【原第225行】计算/保存 WAIST_SPEED_RAD_S：0.6。
WAIST_SPEED_RAD_S = 0.60
# 【原第226行】计算/保存 WAIST_POSITION_TOLERANCE_DEG：2.0。
WAIST_POSITION_TOLERANCE_DEG = 2.0
# 【原第227行】计算/保存 ARM_HOLD_TOLERANCE_RAD：0.035。
ARM_HOLD_TOLERANCE_RAD = 0.035
# 【原第228行】计算/保存 WAIST_SETTLE_S：0.4。
WAIST_SETTLE_S = 0.40
# 【原第229行】计算/保存 WAIST_SEGMENT_SETTLE_S：0.1。
WAIST_SEGMENT_SETTLE_S = 0.10
# 【原第230行】保留参数；当前腰部循环函数未读取此开关或要求TURN/RELEASE。
REQUIRE_WAIST_CONFIRMATION = False
# 【原第231行】计算/保存 MAX_XY_FROM_WP2_M：0.55。
MAX_XY_FROM_WP2_M = 0.550
# 【原第232行】计算/保存 MAX_PREGRASP_DESCENT_M：0.25。
MAX_PREGRASP_DESCENT_M = 0.250
# 【原第233行】计算/保存 MIN_TCP_Z_M：0.69。
MIN_TCP_Z_M = 0.690
# 【原第234行】夹爪长轴采用局部Y轴；完整垂直姿态构造函数直接按Y轴定义。
GRIPPER_LONG_AXIS_LOCAL = "y"
# 【原第235行】计算/保存 GRIPPER_YAW_OFFSET_DEG：0.0。
GRIPPER_YAW_OFFSET_DEG = 0.0
# 【原第236行】计算/保存 MAX_YAW_CORRECTION_DEG：60.0。
MAX_YAW_CORRECTION_DEG = 60.0
# 【原第237行】End到TCP的局部偏移模型；必须与实际工具标定一致。
# 【关键参数教学】
# 参数名：TCP_OFFSET_END_M
# 作用：End→TCP固定工具偏移[0,0,0.14308]m，用于反算腕部目标。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
TCP_OFFSET_END_M = np.array([0.0, 0.0, 0.14308], dtype=float)

# -----------------------------------------------------------------------------
# 笛卡尔和闭环修正
# -----------------------------------------------------------------------------
# 【原第242行】目标发送频率参数50Hz；实际还受计算、图像保存和接口耗时影响。
RATE_HZ = 50.0
# 【原第243行】每个控制点额外sleep时长，1/50秒。
DT = 1.0 / RATE_HZ
# 【原第244行】计算/保存 LIFE_TIME：0.02。
LIFE_TIME = 0.02
# 【原第245行】计算/保存 CART_STEP_M：0.0015。
# 【关键参数教学】
# 参数名：CART_STEP_M
# 作用：笛卡尔位置插值步长1.5mm；越小越密、越慢。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
CART_STEP_M = 0.0015
# 【原第246行】计算/保存 TCP_TOLERANCE_M：0.025。
# 【关键参数教学】
# 参数名：TCP_TOLERANCE_M
# 作用：闭环允许TCP位置误差25mm。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
TCP_TOLERANCE_M = 0.025
# 【原第247行】计算/保存 ORIENTATION_TOLERANCE_DEG：6.0。
ORIENTATION_TOLERANCE_DEG = 6.0
# 【原第248行】计算/保存 HARD_TCP_ERROR_LIMIT_M：0.05。
# 【关键参数教学】
# 参数名：HARD_TCP_ERROR_LIMIT_M
# 作用：TCP误差超过50mm停止，不继续追赶。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
HARD_TCP_ERROR_LIMIT_M = 0.050
# 【原第249行】计算/保存 MAX_CORRECTION_ATTEMPTS：2。
MAX_CORRECTION_ATTEMPTS = 2
# 【原第250行】计算/保存 CORRECTION_STEP_M：0.001。
CORRECTION_STEP_M = 0.001
# 【原第251行】完整姿态每大段最大变化，名称含Yaw但用于总四元数角差。
# 【关键参数教学】
# 参数名：MAX_YAW_PER_STAGE_DEG
# 作用：名称含Yaw，实际用于完整四元数姿态每大段最大12°。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
MAX_YAW_PER_STAGE_DEG = 12.0  # 约70deg由15段减至6段
# 【原第252行】计算/保存 ROTATION_STEP_DEG：2.0。
# 【关键参数教学】
# 参数名：ROTATION_STEP_DEG
# 作用：每个大姿态段内部再按约2°插值。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
ROTATION_STEP_DEG = 2.0  # 段内插值更少，轨迹仍连续
# 【原第253行】计算/保存 ROTATION_FINAL_HOLD_S：0.08。
ROTATION_FINAL_HOLD_S = 0.08
# 【原第254行】计算/保存 VERTICAL_STAGE_SETTLE_S：0.15。
VERTICAL_STAGE_SETTLE_S = 0.15
# 【原第255行】计算/保存 MAX_ROTATION_STAGE_DRIFT_M：0.035。
MAX_ROTATION_STAGE_DRIFT_M = 0.035
# 【原第256行】计算/保存 VERTICAL_STAGE_ORIENTATION_TOLERANCE_DEG：8.0。
VERTICAL_STAGE_ORIENTATION_TOLERANCE_DEG = 8.0
# 【原第257行】计算/保存 VERTICAL_HARD_TCP_DRIFT_M：0.05。
VERTICAL_HARD_TCP_DRIFT_M = 0.050
# 【原第258行】计算/保存 VERTICAL_HARD_ORIENTATION_ERROR_DEG：12.0。
# 【关键参数教学】
# 参数名：VERTICAL_HARD_ORIENTATION_ERROR_DEG
# 作用：实际姿态与本段目标相差超过12°触发硬限制。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
VERTICAL_HARD_ORIENTATION_ERROR_DEG = 12.0
# 【原第259行】连续软超限次数门限，本附件3；旧异常文字写两段。
VERTICAL_MAX_CONSECUTIVE_SOFT_FAILURES = 3
# 【原第260行】计算/保存 VERTICAL_WORKSPACE_XY_RADIUS_M：0.245。
VERTICAL_WORKSPACE_XY_RADIUS_M = 0.245
# 【原第261行】固定锚点累计漂移软门限；本附件60毫米。
VERTICAL_FIXED_ANCHOR_SOFT_DRIFT_M = 0.060
# 【原第262行】固定锚点累计漂移硬门限；本附件60毫米，与软门限相同。
VERTICAL_FIXED_ANCHOR_HARD_DRIFT_M = 0.060
# 【原第263行】计算/保存 POST_ORIENTATION_MAX_XY_RESIDUAL_M：0.08。
POST_ORIENTATION_MAX_XY_RESIDUAL_M = 0.080
# 【原第264行】计算/保存 PRE_ORIENTATION_MAX_XY_MOVE_M：0.45。
PRE_ORIENTATION_MAX_XY_MOVE_M = 0.450

# -----------------------------------------------------------------------------
# 力矩
# -----------------------------------------------------------------------------
# 【原第269行】计算/保存 BASELINE_SAMPLES：30。
BASELINE_SAMPLES = 30
# 【原第270行】计算/保存 TORQUE_DELTA_LIMIT_NM：30.0。
# 【关键参数教学】
# 参数名：TORQUE_DELTA_LIMIT_NM
# 作用：当前effort相对启动基线变化门限30；单位含义沿用GDK字段及原程序命名。
# 修改原则：先确认单位、调用位置和安全边界；只改本参数不一定改变最终动作，因为程序可能还有max()/容差/锁定值。
TORQUE_DELTA_LIMIT_NM = 30.0


# 【原第273行】装饰器：为下面的数据类型添加dataclass自动生成方法。
@dataclass
# 【原第274行】位姿数据容器：position为XYZ（米），orientation为四元数[x,y,z,w]。
class PoseData:
    # 【原第275行】计算/保存 position：。
    position: List[float]
    # 【原第276行】计算/保存 orientation：。
    orientation: List[float]


# 【原第279行】把当前报告字典写入JSON；被检测、运动、异常处理调用。
# ======================================================================================
# 【函数教学卡】save()
# 1. 功能：把当前运行报告写成JSON文件。
# 2. 输入：report字典。
# 3. 输出：没有return，主要副作用是覆盖REPORT_FILE。
# 4. 谁调用：视觉、关节运动、笛卡尔运动、腰部运动、异常处理等多处调用。
# 5. 结果流向：保存后的JSON供失败复盘、循环统计和参数核对。
# 6. G2实际意义：不直接控制机器人，但频繁磁盘写入可能影响循环实时性。
# 7. 调试重点：检查REPORT_FILE路径、磁盘权限以及报告是否可被json序列化。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def save(report):
    # 【原第280行】执行调用/更新：Path(REPORT_FILE).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')。
    """【函数教学卡：超详细版】
    
    功能：
        把运行报告字典保存为JSON文件。
    
    Args:
        report: Dict，包含状态、检测数据、运动记录和异常信息。
    
    Returns:
        None：无返回值；副作用是覆盖写入REPORT_FILE。
    
    调用时机：
        视觉、运动、循环完成和异常处理阶段都会调用。
    
    上下游逻辑：
        上游产生report，下游由开发者读取JSON进行故障复盘。
    
    注意事项与调试重点：
        report内必须是JSON可序列化数据，NumPy对象通常要先转为list或float。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    Path(REPORT_FILE).write_text(json.dumps(report, ensure_ascii=False, indent=2),
                                 # 【原第281行】继续上方跨行表达式的参数/数据项；所属模块：save。
                                 encoding="utf-8")


# 【原第284行】把四元数归一化，供姿态计算使用；零长度四元数会报错。
# ======================================================================================
# 【函数教学卡】norm_q()
# 1. 功能：把四元数归一化为长度1，避免姿态计算因数值尺度错误而失真。
# 2. 输入：q，可迭代的4个数，顺序[x,y,z,w]。
# 3. 输出：归一化后的Python列表。
# 4. 谁调用：q_mul、slerp、q_error_deg、q_to_r、read_pose等姿态函数。
# 5. 结果流向：结果继续用于矩阵转换、姿态误差和插值。
# 6. G2实际意义：不发动作，只做数学处理。
# 7. 调试重点：若长度接近0会抛“四元数无效”；检查上游TF或目标姿态。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def norm_q(q):
    # 【原第285行】计算/保存 arr：np.asarray(q, dtype=float)。
    """【函数教学卡：超详细版】
    
    功能：
        将四元数归一化为长度1，保证后续姿态计算有效。
    
    Args:
        q: List[float]，四元数顺序为[x,y,z,w]。
    
    Returns:
        List[float]：归一化后的四元数。
    
    调用时机：
        被四元数乘法、矩阵转换、姿态误差和插值函数调用。
    
    上下游逻辑：
        输入来自TF或目标姿态，输出继续进入姿态数学函数。
    
    注意事项与调试重点：
        四元数长度接近0时抛出异常；不要把position误传进来。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    arr = np.asarray(q, dtype=float)
    # 【原第286行】计算/保存 n：float(np.linalg.norm(arr))。
    n = float(np.linalg.norm(arr))
    # 【原第287行】条件判断：n < 1e-12；满足时执行下面缩进代码，否则走else（若有）。
    if n < 1e-12:
        # 【原第288行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
        raise RuntimeError("四元数无效")
    # 【原第289行】返回结果给调用者：(arr / n).tolist()。
    return (arr / n).tolist()


# 【原第292行】计算两个四元数的乘积，用于组合旋转；顺序影响结果。
# ======================================================================================
# 【函数教学卡】q_mul()
# 1. 功能：合成两个旋转。四元数乘法有顺序，q1*q2通常不能交换。
# 2. 输入：q1和q2，均为[x,y,z,w]。
# 3. 输出：合成旋转后且已归一化的四元数。
# 4. 谁调用：旧的rotate_yaw函数。
# 5. 结果流向：生成target_q后交给set_pose。
# 6. G2实际意义：数学计算，本身不移动。
# 7. 调试重点：方向相反时优先检查乘法顺序和坐标系，不要只改正负号。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def q_mul(q1, q2):
    # 【原第293行】计算/保存 (x1, y1, z1, w1)：norm_q(q1)。 计算/保存 (x2, y2, z2, w2)：norm_q(q2)。
    """【函数教学卡：超详细版】
    
    功能：
        计算两个四元数的乘积，用于组合两个旋转。
    
    Args:
        q1、q2: 四元数[x,y,z,w]。
    
    Returns:
        List[float]：组合并归一化后的四元数。
    
    调用时机：
        旧版rotate_yaw()调用。
    
    上下游逻辑：
        组合结果可作为右臂新的目标姿态。
    
    注意事项与调试重点：
        乘法顺序不能随意交换；方向错误时先检查坐标系和顺序。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    x1,y1,z1,w1 = norm_q(q1); x2,y2,z2,w2 = norm_q(q2)
    # 【原第294行】返回结果给调用者：norm_q([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])。
    return norm_q([
        # 【原第295行】继续上方跨行表达式的参数/数据项；所属模块：q_mul。
        w1*x2+x1*w2+y1*z2-z1*y2,
        # 【原第296行】继续上方跨行表达式的参数/数据项；所属模块：q_mul。
        w1*y2-x1*z2+y1*w2+z1*x2,
        # 【原第297行】继续上方跨行表达式的参数/数据项；所属模块：q_mul。
        w1*z2+x1*y2-y1*x2+z1*w2,
        # 【原第298行】继续上方跨行表达式的参数/数据项；所属模块：q_mul。
        w1*w2-x1*x2-y1*y2-z1*z2,
    # 【原第299行】继续上方跨行表达式的参数/数据项；所属模块：q_mul。
    ])


# 【原第302行】构造绕Z轴旋转的四元数，输入角度单位为弧度。
# ======================================================================================
# 【函数教学卡】q_yaw()
# 1. 功能：根据绕Base Z轴的yaw角生成四元数。
# 2. 输入：yaw，单位弧度。
# 3. 输出：[0,0,sin(yaw/2),cos(yaw/2)]。
# 4. 谁调用：rotate_yaw。
# 5. 结果流向：交给q_mul与当前姿态组合。
# 6. G2实际意义：不移动。
# 7. 调试重点：math.radians负责“度→弧度”，日志deg与此处输入单位不要混淆。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def q_yaw(yaw):
    # 【原第303行】返回结果给调用者：[0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]。
    """【函数教学卡：超详细版】
    
    功能：
        根据绕Z轴的Yaw角生成对应四元数。
    
    Args:
        yaw: float，单位为弧度。
    
    Returns:
        List[float]：绕Z轴旋转的四元数。
    
    调用时机：
        旧版rotate_yaw()调用。
    
    上下游逻辑：
        输出通常再与当前姿态做四元数乘法。
    
    注意事项与调试重点：
        日志里的deg是度，本函数输入是弧度，需用math.radians转换。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    return [0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0)]


# 【原第306行】在两个四元数间做球面插值；a从0到1，对应起点到终点。
# ======================================================================================
# 【函数教学卡】slerp()
# 1. 功能：在起始姿态和目标姿态之间生成旋转均匀的中间姿态。
# 2. 输入：q0起点、q1终点、a比例0到1。
# 3. 输出：中间四元数。
# 4. 谁调用：align_orientation_target和rotate_yaw。
# 5. 结果流向：每个中间姿态通过end_from_tcp反算End位置，再由set_pose发送。
# 6. G2实际意义：决定机械臂旋转轨迹是否平滑，但函数本身不发送命令。
# 7. 调试重点：dot<0时翻转q1是为了选择四元数的短路径；q与-q代表同一姿态。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def slerp(q0, q1, a):
    # 【原第307行】计算/保存 q0：np.asarray(norm_q(q0))。 计算/保存 q1：np.asarray(norm_q(q1))。 计算/保存 dot：float(q0 @ q1)。
    """【函数教学卡：超详细版】
    
    功能：
        在起始姿态与目标姿态之间做球面线性插值，使旋转轨迹平滑。
    
    Args:
        q0: 起始四元数；q1: 目标四元数；a: 0~1插值比例。
    
    Returns:
        List[float]：比例a对应的中间姿态。
    
    调用时机：
        固定锚点姿态对齐和旧Yaw旋转调用。
    
    上下游逻辑：
        输出中间姿态，经end_from_tcp计算End位置，再由set_pose发送。
    
    注意事项与调试重点：
        q和-q表示同一姿态；代码会选择短旋转路径。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    q0=np.asarray(norm_q(q0)); q1=np.asarray(norm_q(q1)); dot=float(q0@q1)
    # 【原第308行】条件判断：dot < 0；满足时执行下面缩进代码，否则走else（若有）。 计算/保存 q1：-q1。 计算/保存 dot：-dot。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    if dot < 0: q1=-q1; dot=-dot
    # 【原第309行】计算/保存 dot：max(-1.0, min(1.0, dot))。
    dot=max(-1.0,min(1.0,dot))
    # 【原第310行】条件判断：dot > 0.9995；满足时执行下面缩进代码，否则走else（若有）。 返回结果给调用者：norm_q((1 - a) * q0 + a * q1)。
    if dot > 0.9995: return norm_q((1-a)*q0+a*q1)
    # 【原第311行】计算/保存 th：math.acos(dot)。 计算/保存 st：math.sin(th)。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    th=math.acos(dot); st=math.sin(th)
    # 【原第312行】返回结果给调用者：norm_q(math.sin((1 - a) * th) / st * q0 + math.sin(a * th) / st * q1)。
    return norm_q(math.sin((1-a)*th)/st*q0 + math.sin(a*th)/st*q1)


# 【原第315行】计算两个姿态之间的最短旋转角，输出单位为度。
# ======================================================================================
# 【函数教学卡】q_error_deg()
# 1. 功能：计算两个姿态之间最少还差多少旋转角。
# 2. 输入：q1、q2四元数。
# 3. 输出：角度值，单位deg。
# 4. 谁调用：闭环验证、目标候选选择、姿态分段。
# 5. 结果流向：用于判断到位、选择180度等价方向和触发姿态限制。
# 6. G2实际意义：不移动，是验证与决策指标。
# 7. 调试重点：数值大不一定是Yaw大，它表示三维总姿态差。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def q_error_deg(q1, q2):
    # 【原第316行】计算/保存 dot：abs(sum((a * b for a, b in zip(norm_q(q1), norm_q(q2)))))。
    """【函数教学卡：超详细版】
    
    功能：
        计算两个四元数姿态之间的最小三维旋转角。
    
    Args:
        q1、q2: 四元数[x,y,z,w]。
    
    Returns:
        float：姿态差，单位为度。
    
    调用时机：
        闭环验证、姿态分段和等价长轴候选选择调用。
    
    上下游逻辑：
        结果与姿态软/硬门限比较。
    
    注意事项与调试重点：
        这是三维总姿态差，不等同于单独Yaw角差。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    dot=abs(sum(a*b for a,b in zip(norm_q(q1),norm_q(q2))))
    # 【原第317行】返回结果给调用者：math.degrees(2 * math.acos(max(-1.0, min(1.0, dot))))。
    return math.degrees(2*math.acos(max(-1.0,min(1.0,dot))))


# 【原第320行】把四元数转换成3×3旋转矩阵，供坐标和工具轴计算使用。
# ======================================================================================
# 【函数教学卡】q_to_r()
# 1. 功能：把四元数变成3×3旋转矩阵。矩阵三列分别表示工具局部X/Y/Z轴在Base坐标中的方向。
# 2. 输入：四元数[x,y,z,w]。
# 3. 输出：NumPy 3×3矩阵。
# 4. 谁调用：tf_matrix、gripper_yaw、tool_vertical_error_deg、end_from_tcp。
# 5. 结果流向：用于坐标变换、工具轴方向和TCP偏移计算。
# 6. G2实际意义：纯数学。
# 7. 调试重点：工具垂直判断主要取R[:,2]，即局部Z轴。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def q_to_r(q):
    # 【原第321行】计算/保存 (x, y, z, w)：norm_q(q)。
    """【函数教学卡：超详细版】
    
    功能：
        把四元数转换为3×3旋转矩阵。
    
    Args:
        q: 四元数[x,y,z,w]。
    
    Returns:
        np.ndarray：3×3旋转矩阵。
    
    调用时机：
        坐标变换、工具轴方向、TCP偏移计算调用。
    
    上下游逻辑：
        矩阵三列分别表示工具局部X/Y/Z轴在Base中的方向。
    
    注意事项与调试重点：
        必须先归一化；R[:,2]通常表示工具局部Z轴。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    x,y,z,w=norm_q(q)
    # 【原第322行】返回结果给调用者：np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z。
    return np.array([
        # 【原第323行】继续上方跨行表达式的参数/数据项；所属模块：q_to_r。
        [1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        # 【原第324行】继续上方跨行表达式的参数/数据项；所属模块：q_to_r。
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        # 【原第325行】继续上方跨行表达式的参数/数据项；所属模块：q_to_r。
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)],
    # 【原第326行】继续上方跨行表达式的参数/数据项；所属模块：q_to_r。
    ])


# 【原第329行】把TF的旋转和平移组成4×4齐次变换矩阵。
# ======================================================================================
# 【函数教学卡】tf_matrix()
# 1. 功能：把TF中的旋转和平移组合成4×4齐次变换矩阵。
# 2. 输入：GDK返回的TF对象t。
# 3. 输出：4×4矩阵；左上3×3旋转，右上3×1平移，最后一行[0,0,0,1]。
# 4. 谁调用：detect_product_batch。
# 5. 结果流向：相机三维点乘矩阵后转换到base_link坐标。
# 6. G2实际意义：只转换坐标。
# 7. 调试重点：矩阵乘法顺序非常重要；本程序通过两个TF矩阵相乘构造相机到Base的链路。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def tf_matrix(t):
    # 【原第330行】计算/保存 m：np.eye(4)。
    """【函数教学卡：超详细版】
    
    功能：
        把GDK TF对象转换为4×4齐次变换矩阵。
    
    Args:
        t: GDK返回的TF对象，包含translation和rotation。
    
    Returns:
        np.ndarray：4×4齐次变换矩阵。
    
    调用时机：
        视觉三维坐标转换阶段调用。
    
    上下游逻辑：
        相机坐标点通过矩阵链转换到base_link。
    
    注意事项与调试重点：
        矩阵乘法顺序不可交换；错误会直接导致Base XYZ错误。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    m=np.eye(4)
    # 【原第331行】计算/保存 m[:3, :3]：q_to_r([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])。
    m[:3,:3]=q_to_r([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
    # 【原第332行】计算/保存 m[:3, 3]：[t.translation.x, t.translation.y, t.translation.z]。
    m[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
    # 【原第333行】返回结果给调用者：m。
    return m


# 【原第336行】计算两个XYZ之间的三维欧氏距离，单位为米。 返回结果给调用者：math.sqrt(sum(((float(b[i]) - float(a[i])) ** 2 for i in range(3))))。
# ======================================================================================
# 【函数教学卡】dist()
# 1. 功能：计算两个XYZ点之间的直线距离。
# 2. 输入：a、b，每个至少含3个数。
# 3. 输出：距离，单位跟输入相同，本程序通常为米。
# 4. 谁调用：Waypoint TCP记录、闭环、固定锚点漂移等。
# 5. 结果流向：与容差比较决定通过或报警。
# 6. G2实际意义：不移动。
# 7. 调试重点：0.025m等于25mm，注意米/毫米换算。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def dist(a,b): return math.sqrt(sum((float(b[i])-float(a[i]))**2 for i in range(3)))
# 【原第337行】按比例t在两个XYZ之间线性插值，生成轨迹中的位置。 返回结果给调用者：[(1 - t) * a[i] + t * b[i] for i in range(3)]。
# ======================================================================================
# 【函数教学卡】lerp()
# 1. 功能：在线段起点和终点之间按比例生成位置中间点。
# 2. 输入：a起点、b终点、t比例0到1。
# 3. 输出：3元素位置列表。
# 4. 谁调用：command_cart_path。
# 5. 结果流向：中间位置与固定姿态一起发给set_pose。
# 6. G2实际意义：决定平移轨迹离散点。
# 7. 调试重点：该函数只插值位置，不插值姿态。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def lerp(a,b,t): return [(1-t)*a[i]+t*b[i] for i in range(3)]


# 【原第340行】把无方向长轴角度归入±90度的等价范围，输入输出为弧度。
# ======================================================================================
# 【函数教学卡】wrap_axis()
# 1. 功能：把产品无方向长轴角度归一到[-90°,90°]等价区间。
# 2. 输入：angle弧度。
# 3. 输出：等价长轴角弧度。
# 4. 谁调用：本附件中保留，主要理念被axis_average和_axis_diff_deg使用。
# 5. 结果流向：用于解决长方形长轴没有正反方向的问题。
# 6. G2实际意义：不移动。
# 7. 调试重点：长轴0°与180°等价，这与有方向箭头不同。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def wrap_axis(angle):
    # 【原第341行】重复执行下面代码，直到条件不成立：angle > math.pi / 2。 计算/保存 angle：math.pi。
    """【函数教学卡：超详细版】
    
    功能：
        将无方向长轴角归一化到±90°等价范围。
    
    Args:
        angle: float，单位弧度。
    
    Returns:
        float：归一化后的长轴角。
    
    调用时机：
        长轴相关辅助计算使用。
    
    上下游逻辑：
        解决0°与180°代表同一长轴的问题。
    
    注意事项与调试重点：
        适用于无箭头的长轴，不适用于有明确正方向的轴。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    while angle > math.pi/2: angle -= math.pi
    # 【原第342行】重复执行下面代码，直到条件不成立：angle < -math.pi / 2。 计算/保存 angle：math.pi。
    while angle < -math.pi/2: angle += math.pi
    # 【原第343行】返回结果给调用者：angle。
    return angle


# 【原第346行】用双角法平均长轴方向，避免+89度和-89度被错误平均为0度。
# ======================================================================================
# 【函数教学卡】axis_average()
# 1. 功能：用双角法对180度等价长轴求平均。
# 2. 输入：多个长轴角，单位弧度。
# 3. 输出：平均长轴角，弧度。
# 4. 谁调用：单批角度汇总、多个稳定批次最终角度。
# 5. 结果流向：送入vertical_tool_orientation构造夹爪目标姿态。
# 6. G2实际意义：不移动。
# 7. 调试重点：普通平均+89°和-89°会得到0°，双角法可避免这个错误。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def axis_average(values):
    # 【原第347行】计算/保存 v：np.asarray(values)。
    """【函数教学卡：超详细版】
    
    功能：
        对180°等价的多个产品长轴角求平均。
    
    Args:
        values: List[float]，角度单位弧度。
    
    Returns:
        float：平均长轴角，单位弧度。
    
    调用时机：
        单批和稳定多批次角度汇总调用。
    
    上下游逻辑：
        输出用于构造夹爪目标方向。
    
    注意事项与调试重点：
        不能直接普通平均+89°与-89°，因此采用双角法。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    v=np.asarray(values)
    # 【原第348行】返回结果给调用者：0.5 * math.atan2(float(np.mean(np.sin(2 * v))), float(np.mean(np.cos(2 * v))))。
    return 0.5*math.atan2(float(np.mean(np.sin(2*v))),float(np.mean(np.cos(2*v))))


# 【原第351行】读取指定坐标系相对base_link的TF，返回PoseData。
# ======================================================================================
# 【函数教学卡】read_pose()
# 1. 功能：读取某个机器人坐标系相对base_link的实际位置和姿态。
# 2. 输入：tf_api和frame名称。
# 3. 输出：PoseData(position,orientation)。
# 4. 谁调用：wait_pose。
# 5. 结果流向：用于闭环验证、运动起点、固定TCP锚点。
# 6. G2实际意义：读取状态，不发命令。
# 7. 调试重点：TF有数据不代表机械臂已稳定；通常在运动后结合sleep和误差判断。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def read_pose(tf_api, frame):
    # 【原第352行】计算/保存 t：tf_api.get_tf_from_base_link(frame)。
    """【函数教学卡：超详细版】
    
    功能：
        读取指定坐标系相对base_link的实际位姿。
    
    Args:
        tf_api: GDK TF对象；frame: 坐标系名称。
    
    Returns:
        PoseData：position XYZ和orientation xyzw。
    
    调用时机：
        wait_pose()调用。
    
    上下游逻辑：
        实际TF用于运动起点、固定锚点和到位验证。
    
    注意事项与调试重点：
        TF可读不代表机器人已停止，需要结合等待和误差判断。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    t=tf_api.get_tf_from_base_link(frame)
    # 【原第353行】返回结果给调用者：PoseData([float(t.translation.x), float(t.translation.y), float(t.translation.z)], norm_q([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]))。
    return PoseData([float(t.translation.x),float(t.translation.y),float(t.translation.z)],
                    # 【原第354行】继续上方跨行表达式的参数/数据项；所属模块：read_pose。
                    norm_q([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]))


# 【原第357行】等待TF可用后调用read_pose；超时抛异常，不自动恢复机械臂。
# ======================================================================================
# 【函数教学卡】wait_pose()
# 1. 功能：在限定时间内等待TF可用，再读取位姿。
# 2. 输入：tf_api、frame、timeout秒。
# 3. 输出：PoseData；超时则异常。
# 4. 谁调用：几乎所有运动和验证函数。
# 5. 结果流向：为运动目标和到位误差提供实际状态。
# 6. G2实际意义：读取，不移动。
# 7. 调试重点：TF超时优先检查DDS、坐标系名称、GDK状态，不要盲目增大timeout。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def wait_pose(tf_api, frame, timeout=10.0):
    # 【原第358行】计算/保存 deadline：time.time() + timeout。
    """【函数教学卡：超详细版】
    
    功能：
        等待指定TF可用并读取实际位姿。
    
    Args:
        tf_api: TF接口；frame: 坐标系；timeout: 最长等待秒数。
    
    Returns:
        PoseData：实际位姿；超时抛异常。
    
    调用时机：
        几乎所有运动与验证函数调用。
    
    上下游逻辑：
        向下游提供实际TCP或End位置和姿态。
    
    注意事项与调试重点：
        超时时检查DDS、frame名称和GDK状态，不要只增大timeout。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    deadline=time.time()+timeout
    # 【原第359行】重复执行下面代码，直到条件不成立：time.time() < deadline。
    while time.time()<deadline:
        # 【原第360行】条件判断：tf_api.can_transform('base_link', frame)；满足时执行下面缩进代码，否则走else（若有）。 返回结果给调用者：read_pose(tf_api, frame)。
        if tf_api.can_transform("base_link",frame): return read_pose(tf_api,frame)
        # 【原第361行】暂停当前线程：time.sleep(0.2)；此等待不代表已验证机器人到位。
        time.sleep(0.2)
    # 【原第362行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
    raise RuntimeError(f"TF超时: {frame}")


# 【原第365行】读取关节状态，按关节名称建立字典，供位置读取使用。
# ======================================================================================
# 【函数教学卡】states_map()
# 1. 功能：把GDK关节状态列表变成“关节名→状态”的字典。
# 2. 输入：robot对象。
# 3. 输出：字典。
# 4. 谁调用：get_positions。
# 5. 结果流向：按指定顺序读取motor_position。
# 6. G2实际意义：读取。
# 7. 调试重点：检查返回states中是否包含目标关节名。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def states_map(robot):
    # 【原第366行】返回结果给调用者：{s['name']: s for s in robot.get_joint_states()['states']}。
    """【函数教学卡：超详细版】
    
    功能：
        把GDK关节状态列表转换为按关节名索引的字典。
    
    Args:
        robot: GDK Robot对象。
    
    Returns:
        Dict：键为关节名，值为该关节状态字典。
    
    调用时机：
        get_positions()调用。
    
    上下游逻辑：
        便于按RIGHT_JOINTS等列表读取指定关节。
    
    注意事项与调试重点：
        如果SDK返回结构变化，需要同步调整states字段访问。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    return {s["name"]:s for s in robot.get_joint_states()["states"]}


# 【原第369行】按names指定顺序提取motor_position，返回弧度列表。
# ======================================================================================
# 【函数教学卡】get_positions()
# 1. 功能：按调用者给定的关节名顺序读取实际motor_position。
# 2. 输入：robot和names列表。
# 3. 输出：关节角列表，单位弧度。
# 4. 谁调用：启动姿态、头部、腰部、双臂保持验证等。
# 5. 结果流向：用于误差、限位和动作前后比较。
# 6. G2实际意义：读取。
# 7. 调试重点：motor_position是电机位置；缺少任一名称会抛异常。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def get_positions(robot,names):
    # 【原第370行】计算/保存 states：states_map(robot)。
    """【函数教学卡：超详细版】
    
    功能：
        按指定顺序读取多个关节的motor_position。
    
    Args:
        robot: Robot对象；names: 关节名称列表。
    
    Returns:
        List[float]：关节角，单位弧度。
    
    调用时机：
        启动识别、Waypoint验证、头部和腰部验证调用。
    
    上下游逻辑：
        结果交给max_joint_error、限位检查或报告。
    
    注意事项与调试重点：
        任何关节名缺失都会抛异常。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    states=states_map(robot)
    # 【原第371行】计算/保存 missing：[n for n in names if n not in states]。
    missing=[n for n in names if n not in states]
    # 【原第372行】条件判断：missing；满足时执行下面缩进代码，否则走else（若有）。 抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
    if missing: raise RuntimeError(f"缺少关节: {missing}")
    # 【原第373行】返回结果给调用者：[float(states[n]['motor_position']) for n in names]。
    return [float(states[n]["motor_position"]) for n in names]


# 【原第376行】比较两组关节角，返回最大绝对差，单位为弧度。 返回结果给调用者：max((abs(x - y) for x, y in zip(a, b)))。
# ======================================================================================
# 【函数教学卡】max_joint_error()
# 1. 功能：取两组关节角中最大的单关节绝对误差。
# 2. 输入：a、b等长列表。
# 3. 输出：最大误差，弧度。
# 4. 谁调用：启动识别、关节到位、腰部期间双臂保持。
# 5. 结果流向：与对应容差比较。
# 6. G2实际意义：不移动。
# 7. 调试重点：它不是平均误差，一个关节偏差大就会被发现。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def max_joint_error(a,b): return max(abs(x-y) for x,y in zip(a,b))


# 【原第379行】识别当前右臂最接近HOME/WP1/WP2中的哪一个；差异过大拒绝启动。
# ======================================================================================
# 【函数教学卡】identify_start()
# 1. 功能：判断程序启动时右臂处于HOME、WP1还是WP2附近。
# 2. 输入：当前右臂7个关节角。
# 3. 输出：name、index、error。
# 4. 谁调用：main启动阶段。
# 5. 结果流向：main根据index决定如何进入视觉工作位。
# 6. G2实际意义：不移动，是启动安全门。
# 7. 调试重点：不要把START_POSE_TOLERANCE_RAD放大到覆盖未知姿态。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def identify_start(current):
    # 【原第380行】计算/保存 candidates：[(max_joint_error(current, HOME), 'home', 0), (max_joint_error(current, WAYPOINT_1), 'waypoint_1', 1), (max_joint_error(current, WAYPOINT_2), 'waypoin…。
    """【函数教学卡：超详细版】
    
    功能：
        识别当前右臂最接近HOME、WAYPOINT_1还是WAYPOINT_2。
    
    Args:
        current: 当前右臂7关节角。
    
    Returns:
        Tuple：最近姿态名称、索引、最大关节误差。
    
    调用时机：
        main()启动阶段调用。
    
    上下游逻辑：
        main根据索引决定进入视觉位的路径。
    
    注意事项与调试重点：
        误差过大表示未知姿态，不能通过放大容差强行启动。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    candidates=[(max_joint_error(current,HOME),"home",0),
                # 【原第381行】继续上方跨行表达式的参数/数据项；所属模块：identify_start。
                (max_joint_error(current,WAYPOINT_1),"waypoint_1",1),
                # 【原第382行】继续上方跨行表达式的参数/数据项；所属模块：identify_start。
                (max_joint_error(current,WAYPOINT_2),"waypoint_2",2)]
    # 【原第383行】计算/保存 (error, name, index)：min(candidates)。
    error,name,index=min(candidates)
    # 【原第384行】条件判断：error > START_POSE_TOLERANCE_RAD；满足时执行下面缩进代码，否则走else（若有）。
    if error>START_POSE_TOLERANCE_RAD:
        # 【原第385行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
        raise RuntimeError(f"当前姿态未知，最近{name}，误差={error:.3f}rad")
    # 【原第386行】返回结果给调用者：(name, index, error)。
    return name,index,error


# 【原第389行】读取各关节effort，供基线和变化检查使用。
# ======================================================================================
# 【函数教学卡】get_torques()
# 1. 功能：读取每个关节的effort数值。
# 2. 输入：robot。
# 3. 输出：关节名到effort的字典。
# 4. 谁调用：torque_baseline和verify_torque。
# 5. 结果流向：用于相对基线的力矩变化检查。
# 6. G2实际意义：读取。
# 7. 调试重点：effort含义应以GDK接口定义为准，本程序按相对变化使用。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def get_torques(robot):
    # 【原第390行】返回结果给调用者：{s['name']: float(s['effort']) for s in robot.get_joint_states()['states']}。
    """【函数教学卡：超详细版】
    
    功能：
        读取所有关节当前effort。
    
    Args:
        robot: Robot对象。
    
    Returns:
        Dict[str,float]：关节名到effort值。
    
    调用时机：
        力矩基线和力矩安全检查调用。
    
    上下游逻辑：
        当前effort与启动基线比较。
    
    注意事项与调试重点：
        effort物理含义以GDK文档为准，本程序使用相对变化。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    return {s["name"]:float(s["effort"]) for s in robot.get_joint_states()["states"]}


# 【原第393行】静止时多次读取effort并平均，建立本次运行的参考值。
# ======================================================================================
# 【函数教学卡】torque_baseline()
# 1. 功能：机器人静止时采样多次effort并求平均。
# 2. 输入：robot。
# 3. 输出：基线字典。
# 4. 谁调用：main在进入循环前调用一次。
# 5. 结果流向：后续verify_torque拿当前值减基线。
# 6. G2实际意义：读取，不移动，但要求机器人保持静止。
# 7. 调试重点：建立基线时如果机械臂仍在晃动，会影响后续判断。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def torque_baseline(robot):
    # 【原第394行】打印当前进度或诊断信息到控制台；不发送机器人命令。
    """【函数教学卡：超详细版】
    
    功能：
        机器人静止时多次采样effort并计算平均基线。
    
    Args:
        robot: Robot对象。
    
    Returns:
        Dict：每个关节的平均effort基线。
    
    调用时机：
        main在正式运动循环前调用一次。
    
    上下游逻辑：
        基线传给verify_torque。
    
    注意事项与调试重点：
        采样期间机器人必须静止，否则基线不可靠。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    print("建立力矩基线，请保持机器人静止...")
    # 【原第395行】计算/保存 samples：[]。
    samples=[]
    # 【原第396行】循环：依次从 range(BASELINE_SAMPLES) 取值到 _，执行下面缩进块。 执行调用/更新：samples.append(get_torques(robot))。 暂停当前线程：time.sleep(0.05)；此等待不代表已验证机器人到位。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    for _ in range(BASELINE_SAMPLES): samples.append(get_torques(robot)); time.sleep(0.05)
    # 【原第397行】计算/保存 names：set().union(*(s.keys() for s in samples))。
    names=set().union(*(s.keys() for s in samples))
    # 【原第398行】返回结果给调用者：{n: sum((s.get(n, 0.0) for s in samples)) / len(samples) for n in names}。
    return {n:sum(s.get(n,0.0) for s in samples)/len(samples) for n in names}


# 【原第401行】比较当前effort与基线，变化超阈值时保存报告并抛异常。
# ======================================================================================
# 【函数教学卡】verify_torque()
# 1. 功能：比较当前effort与启动基线，变化过大时阻止继续。
# 2. 输入：robot、base基线、report。
# 3. 输出：正常时无返回；异常时保存并抛错。
# 4. 谁调用：关节运动前、笛卡尔路径每个位置点。
# 5. 结果流向：超限后由main外层except记录并停止正常流程。
# 6. G2实际意义：安全检查，不主动急停或回HOME。
# 7. 调试重点：触发时查看torque_abnormal中的关节名和delta。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def verify_torque(robot,base,report):
    # 【原第402行】计算/保存 bad：[]。
    """【函数教学卡：超详细版】
    
    功能：
        检查当前关节effort相对基线是否变化过大。
    
    Args:
        robot: Robot；base: 基线字典；report: 报告。
    
    Returns:
        None：正常无返回；超限保存报告并抛异常。
    
    调用时机：
        关节运动前和笛卡尔路径点循环调用。
    
    上下游逻辑：
        异常由main外层捕获，正常流程停止。
    
    注意事项与调试重点：
        这不是硬件急停，也不会自动回HOME。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    bad=[]
    # 【原第403行】循环：依次从 get_torques(robot).items() 取值到 (n, v)，执行下面缩进块。
    for n,v in get_torques(robot).items():
        # 【原第404行】条件判断：n in base and abs(v - base[n]) > TORQUE_DELTA_LIMIT_NM；满足时执行下面缩进代码，否则走else（若有）。
        if n in base and abs(v-base[n])>TORQUE_DELTA_LIMIT_NM:
            # 【原第405行】执行调用/更新：bad.append({'joint': n, 'delta': abs(v - base[n])})。
            bad.append({"joint":n,"delta":abs(v-base[n])})
    # 【原第406行】条件判断：bad；满足时执行下面缩进代码，否则走else（若有）。
    if bad:
        # 【原第407行】计算/保存 report['status']：'STOPPED_TORQUE'。 计算/保存 report['torque_abnormal']：bad[:10]。 调用save保存当前报告；便于定位本轮检测或运动问题。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        report["status"]="STOPPED_TORQUE"; report["torque_abnormal"]=bad[:10]; save(report)
        # 【原第408行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
        raise RuntimeError("力矩安全检查触发")


# 【原第411行】调用右臂关节目标控制；检查关节限位和到位误差，TCP误差在此主要记录。
# ======================================================================================
# 【函数教学卡】move_arm_once()
# 1. 功能：让右臂通过GDK关节控制一次运动到HOME/WP1/WP2之一，并验证实际关节误差。
# 2. 输入：robot、tf_api、label、7关节目标、力矩基线、报告。
# 3. 输出：无显式return。
# 4. 谁调用：main启动、每轮前后回Waypoint。
# 5. 结果流向：到达后流程继续头部、视觉或下一轮。
# 6. G2实际意义：会真实控制右臂。
# 7. 调试重点：label必须能在RECORDED_TCP中找到；TCP差只记录，关节差才触发本函数停止。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def move_arm_once(robot,tf_api,label,target,base,report):
    # 【原第412行】执行调用/更新：verify_torque(robot, base, report)。
    """【函数教学卡：超详细版】
    
    功能：
        通过关节空间控制把右臂一次移动到HOME/WP1/WP2目标。
    
    Args:
        robot、tf_api、label、target、baseline、report。
    
    Returns:
        None：无显式返回。
    
    调用时机：
        main在启动进入工作位以及每轮回位时调用。
    
    上下游逻辑：
        到位后进入头部、视觉或下一轮。
    
    注意事项与调试重点：
        会真实控制右臂；主要按关节误差判定到位，TCP误差用于记录。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    verify_torque(robot,base,report)
    # 【原第413行】循环：依次从 enumerate(zip(target, JOINT_LIMITS)) 取值到 (i, (v, lim))，执行下面缩进块。
    for i,(v,lim) in enumerate(zip(target,JOINT_LIMITS)):
        # 【原第414行】条件判断：not lim[0] <= v <= lim[1]；满足时执行下面缩进代码，否则走else（若有）。 抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
        if not lim[0]<=v<=lim[1]: raise RuntimeError(f"{label}关节{i+1}超限")
    # 【原第415行】打印当前进度或诊断信息到控制台；不发送机器人命令。
    print(f"[{label}] 一次到点，速度={DIRECT_ARM_SPEED_RAD_S:.2f}rad/s")
    # 【原第416行】条件判断：robot.move_arm_joint(list(target), [DIRECT_ARM_SPEED_RAD_S] * 7, 1) != 0；满足时执行下面缩进代码，否则走else（若有）。
    if robot.move_arm_joint(list(target),[DIRECT_ARM_SPEED_RAD_S]*7,1)!=0:
        # 【原第417行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
        raise RuntimeError(f"{label}运动失败")
    # 【原第418行】暂停当前线程：time.sleep(SETTLE_S)；此等待不代表已验证机器人到位。
    time.sleep(SETTLE_S)
    # 【原第419行】计算/保存 actual：get_positions(robot, RIGHT_JOINTS)。 计算/保存 je：max_joint_error(actual, target)。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    actual=get_positions(robot,RIGHT_JOINTS); je=max_joint_error(actual,target)
    # 【原第420行】计算/保存 tcp：wait_pose(tf_api, TCP_FRAME)。 计算/保存 te：dist(tcp.position, RECORDED_TCP[label])。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    tcp=wait_pose(tf_api,TCP_FRAME); te=dist(tcp.position,RECORDED_TCP[label])
    # 【原第421行】执行调用/更新：report['joint_stages'].append({'stage': label, 'joint_error': je, 'tcp_error': te, 'actual': actual, 'tcp': tcp.__dict__})。
    report["joint_stages"].append({"stage":label,"joint_error":je,"tcp_error":te,
                                   # 【原第422行】调用save保存当前报告；便于定位本轮检测或运动问题。
                                   # 【本行拆解教学：分号语句】
                                   # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
                                   # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
                                   # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
                                   "actual":actual,"tcp":tcp.__dict__}); save(report)
    # 【原第423行】打印当前进度或诊断信息到控制台；不发送机器人命令。
    print(f"[{label}验证] 关节误差={je:.5f}rad, TCP误差={te:.5f}m")
    # 【原第424行】条件判断：je > JOINT_TOLERANCE_RAD；满足时执行下面缩进代码，否则走else（若有）。
    if je > JOINT_TOLERANCE_RAD:
        # 【原第425行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
        raise RuntimeError(
            # 【原第426行】继续上方跨行表达式的参数/数据项；所属模块：move_arm_once。
            f"{label}未到位"
        # 【原第427行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
        )


# 【原第430行】必要时把头部第三关节移到+20度，再读取实际角度验证。
# ======================================================================================
# 【函数教学卡】ensure_head()
# 1. 功能：保证头部idx13约为+20°，使相机看到工作区。
# 2. 输入：robot、report。
# 3. 输出：无return。
# 4. 谁调用：main进入视觉位前。
# 5. 结果流向：到位后执行WAYPOINT_2或视觉检测。
# 6. G2实际意义：可能控制头部。
# 7. 调试重点：只改变第三个头部关节，前两个保持读取到的当前值。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def ensure_head(robot,report):
    # 【原第431行】计算/保存 before：get_positions(robot, HEAD_JOINTS)。
    """【函数教学卡：超详细版】
    
    功能：
        保证头部第三关节idx13约为+20°，使相机朝向工作区。
    
    Args:
        robot: 机器人对象，用于关节控制与读取；report: 日志字典。
    
    Returns:
        None：无返回值。
    
    调用时机：
        main进入WAYPOINT_2和视觉检测之前调用。
    
    上下游逻辑：
        完成后main继续到视觉工作位并运行YOLO。
    
    注意事项与调试重点：
        只改变第三个头部关节；目标以弧度发送，日志换算成度。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    before=get_positions(robot,HEAD_JOINTS)
    # 【原第432行】计算/保存 target：[before[0], before[1], math.radians(HEAD_TARGET_DEG)]。
    target=[before[0],before[1],math.radians(HEAD_TARGET_DEG)]
    # 【原第433行】条件判断：abs(math.degrees(before[2]) - HEAD_TARGET_DEG) > HEAD_TOLERANCE_DEG；满足时执行下面缩进代码，否则走else（若有）。
    if abs(math.degrees(before[2])-HEAD_TARGET_DEG)>HEAD_TOLERANCE_DEG:
        # 【原第434行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print("[WAYPOINT_1已验证] 头部idx13移动到+20度")
        # 【原第435行】条件判断：robot.move_head_joint(target, [HEAD_SPEED_RAD_S] * 3) != 0；满足时执行下面缩进代码，否则走else（若有）。
        if robot.move_head_joint(target,[HEAD_SPEED_RAD_S]*3)!=0:
            # 【原第436行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
            raise RuntimeError("头部运动失败")
        # 【原第437行】暂停当前线程：time.sleep(SETTLE_S)；此等待不代表已验证机器人到位。
        time.sleep(SETTLE_S)
    # 【原第438行】计算/保存 after：get_positions(robot, HEAD_JOINTS)。 计算/保存 error：abs(math.degrees(after[2]) - 20.0)。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    after=get_positions(robot,HEAD_JOINTS); error=abs(math.degrees(after[2])-20.0)
    # 【原第439行】计算/保存 report['head']：{'before': before, 'after': after, 'error_deg': error}。 调用save保存当前报告；便于定位本轮检测或运动问题。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    report["head"]={"before":before,"after":after,"error_deg":error}; save(report)
    # 【原第440行】打印当前进度或诊断信息到控制台；不发送机器人命令。
    print(f"[头部验证] idx13={math.degrees(after[2]):.3f}deg, 误差={error:.3f}deg")
    # 【原第441行】条件判断：error > HEAD_TOLERANCE_DEG；满足时执行下面缩进代码，否则走else（若有）。 抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
    if error>HEAD_TOLERANCE_DEG: raise RuntimeError("头部+20度未到位")


# 【原第444行】获取指定相机图像；捕获读取异常并返回None，交给视觉循环重试。
# ======================================================================================
# 【函数教学卡】safe_image()
# 1. 功能：读取最新相机图像并把接口异常转换为None，让视觉循环重试。
# 2. 输入：camera和相机kind。
# 3. 输出：图像对象或None。
# 4. 谁调用：detect_product_batch。
# 5. 结果流向：交给decode_color或decode_depth。
# 6. G2实际意义：读取相机。
# 7. 调试重点：连续None时检查相机、超时参数和DDS，不会在这里输出异常细节。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def safe_image(camera,kind):
    # 【原第445行】开始异常处理块：try执行正常操作，except处理异常，finally执行清理（若有）。 返回结果给调用者：camera.get_latest_image(kind, CAMERA_TIMEOUT_MS)。
    """【函数教学卡：超详细版】
    
    功能：
        安全读取最新颜色图或深度图，接口异常时返回None供循环重试。
    
    Args:
        camera: Camera对象；kind: 相机图像类型。
    
    Returns:
        图像对象或None。
    
    调用时机：
        detect_product_batch()调用。
    
    上下游逻辑：
        颜色图进入YOLO，深度图进入三维定位。
    
    注意事项与调试重点：
        连续None时检查相机、DDS和超时配置。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    try: return camera.get_latest_image(kind,CAMERA_TIMEOUT_MS)
    # 【原第446行】返回结果给调用者：None。
    except Exception: return None


# 【原第449行】把图像字节解码成OpenCV的BGR图像。 返回结果给调用者：cv2.imdecode(np.frombuffer(img.data, np.uint8), cv2.IMREAD_COLOR)。
# ======================================================================================
# 【函数教学卡】decode_color()
# 1. 功能：把压缩颜色图字节解码成OpenCV BGR数组。
# 2. 输入：GDK图像对象。
# 3. 输出：NumPy BGR图像。
# 4. 谁调用：detect_product_batch。
# 5. 结果流向：YOLO、绘框和ROI裁剪。
# 6. G2实际意义：不移动。
# 7. 调试重点：OpenCV颜色顺序是BGR，不是RGB。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def decode_color(img): return cv2.imdecode(np.frombuffer(img.data,np.uint8),cv2.IMREAD_COLOR)


# 【原第452行】按bit_depth选择浮点或16位类型，把深度字节恢复成二维数组。
# ======================================================================================
# 【函数教学卡】decode_depth()
# 1. 功能：把深度图字节按位深恢复成二维数组。
# 2. 输入：深度图对象。
# 3. 输出：二维NumPy数组或None。
# 4. 谁调用：detect_product_batch。
# 5. 结果流向：median_depth提取局部深度。
# 6. G2实际意义：不移动。
# 7. 调试重点：bit_depth=32时按float32，否则按uint16；reshape失败会返回None。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def decode_depth(img):
    # 【原第453行】开始异常处理块：try执行正常操作，except处理异常，finally执行清理（若有）。
    """【函数教学卡：超详细版】
    
    功能：
        把深度图原始字节恢复为二维深度数组。
    
    Args:
        image: GDK深度图对象。
    
    Returns:
        np.ndarray或None。
    
    调用时机：
        detect_product_batch()调用。
    
    上下游逻辑：
        交给median_depth提取中心和桌面深度。
    
    注意事项与调试重点：
        依据bit_depth选择float32或uint16；尺寸不匹配会失败。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    try:
        # 【原第454行】计算/保存 dtype：np.float32 if img.bit_depth == 32 else np.uint16。
        dtype=np.float32 if img.bit_depth==32 else np.uint16
        # 【原第455行】返回结果给调用者：np.frombuffer(img.data, dtype=dtype).reshape((img.height, img.width))。
        return np.frombuffer(img.data,dtype=dtype).reshape((img.height,img.width))
    # 【原第456行】返回结果给调用者：None。
    except Exception: return None


# 【原第459行】在指定像素附近取小窗口，过滤无效值后计算深度中位数。
# ======================================================================================
# 【函数教学卡】median_depth()
# 1. 功能：在目标像素周围取窗口，过滤无效深度后用中位数降低噪声。
# 2. 输入：depth二维数组、u/v深度图坐标。
# 3. 输出：原始深度值或None。
# 4. 谁调用：产品中心和桌面环形采样。
# 5. 结果流向：按数值大小转换成米，再反投影三维点。
# 6. G2实际意义：不移动。
# 7. 调试重点：DEPTH_RADIUS=4代表最大9×9区域；阈值是原始深度单位。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def median_depth(depth,u,v):
    # 【原第460行】计算/保存 (h, w)：depth.shape[:2]。
    """【函数教学卡：超详细版】
    
    功能：
        在指定深度像素附近取窗口并计算有效深度中位数。
    
    Args:
        depth: 深度数组；u、v: 像素坐标。
    
    Returns:
        float原始深度值或None。
    
    调用时机：
        产品中心和桌面环形采样调用。
    
    上下游逻辑：
        深度值转换为米后用于反投影和Base坐标。
    
    注意事项与调试重点：
        DEPTH_RADIUS=4时窗口最大9×9。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    h,w=depth.shape[:2]
    # 【原第461行】计算/保存 p：depth[max(0, v - DEPTH_RADIUS):min(h, v + DEPTH_RADIUS + 1), max(0, u - DEPTH_RADIUS):min(w, u + DEPTH_RADIUS + 1)].astype(float)。
    p=depth[max(0,v-DEPTH_RADIUS):min(h,v+DEPTH_RADIUS+1),
            # 【原第462行】继续上方跨行表达式的参数/数据项；所属模块：median_depth。
            max(0,u-DEPTH_RADIUS):min(w,u+DEPTH_RADIUS+1)].astype(float)
    # 【原第463行】计算/保存 p：p[np.isfinite(p)]。 计算/保存 p：p[(p > MIN_DEPTH_RAW) & (p < MAX_DEPTH_RAW)]。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    p=p[np.isfinite(p)]; p=p[(p>MIN_DEPTH_RAW)&(p<MAX_DEPTH_RAW)]
    # 【原第464行】返回结果给调用者：None if p.size == 0 else float(np.median(p))。
    return None if p.size==0 else float(np.median(p))



# 【原第468行】格式化面板数值；没有数据时显示短横线。
# ======================================================================================
# 【函数教学卡】_ui_fmt()
# 1. 功能：把右侧面板数值格式化成固定小数位。
# 2. 输入：value、digits、suffix。
# 3. 输出：字符串。
# 4. 谁调用：refresh_full_yolo_window。
# 5. 结果流向：交给_ui_put绘制。
# 6. G2实际意义：只影响显示。
# 7. 调试重点：None显示“-”；格式化异常则直接str(value)。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def _ui_fmt(value, digits=3, suffix=""):
    # 【原第469行】条件判断：value is None；满足时执行下面缩进代码，否则走else（若有）。
    """【函数教学卡：超详细版】
    
    功能：
        格式化YOLO右侧面板数值。
    
    Args:
        value、digits、suffix。
    
    Returns:
        str：可显示文本。
    
    调用时机：
        refresh_full_yolo_window()调用。
    
    上下游逻辑：
        输出给_ui_put绘制。
    
    注意事项与调试重点：
        None显示“-”，只影响界面。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    if value is None:
        # 【原第470行】返回结果给调用者：'-'。
        return "-"
    # 【原第471行】开始异常处理块：try执行正常操作，except处理异常，finally执行清理（若有）。
    try:
        # 【原第472行】返回结果给调用者：f'{float(value):.{digits}f}{suffix}'。
        return f"{float(value):.{digits}f}{suffix}"
    # 【原第473行】异常分支：捕获对应异常，执行下面缩进代码；保持原处理方式。
    except Exception:
        # 【原第474行】返回结果给调用者：str(value)。
        return str(value)


# 【原第477行】用OpenCV在画面指定位置绘制文字，不参与运动控制。
# ======================================================================================
# 【函数教学卡】_ui_put()
# 1. 功能：在OpenCV图像上写一行英文或数字。
# 2. 输入：画布、文字、坐标、颜色、字号、线宽。
# 3. 输出：无返回，直接修改image。
# 4. 谁调用：大屏刷新函数。
# 5. 结果流向：最终canvas显示和保存。
# 6. G2实际意义：只显示。
# 7. 调试重点：cv2.putText默认不适合复杂中文，本程序面板主要使用英文标签。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def _ui_put(image, text, x, y, color=(230,230,230), scale=0.50, thickness=1):
    # 【原第478行】执行调用/更新：cv2.putText(image, str(text), (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)。
    """【函数教学卡：超详细版】
    
    功能：
        在OpenCV画布上绘制一行文字。
    
    Args:
        image、text、x、y、color、scale、thickness。
    
    Returns:
        None：直接修改传入图像。
    
    调用时机：
        大屏刷新函数多次调用。
    
    上下游逻辑：
        文字最终显示并保存到JPG。
    
    注意事项与调试重点：
        cv2.putText对复杂中文支持有限。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    cv2.putText(image, str(text), (x,y), cv2.FONT_HERSHEY_SIMPLEX,
                # 【原第479行】继续上方跨行表达式的参数/数据项；所属模块：_ui_put。
                scale, color, thickness, cv2.LINE_AA)


# 【原第482行】组合缓存图像和参数面板，保存并显示；运动期间通常是锁定图像而非新相机帧。
# ======================================================================================
# 【函数教学卡】refresh_full_yolo_window()
# 1. 功能：持续生成1600×960大屏：左侧最后一次YOLO图，右侧实时阶段和数值。
# 2. 输入：可选stage和message。
# 3. 输出：无return；保存JPG并刷新窗口。
# 4. 谁调用：启动、视觉、笛卡尔、姿态、腰部、轮间等多个阶段。
# 5. 结果流向：更新UI_STATE显示，不改变运动目标。
# 6. G2实际意义：不控制机器人；但磁盘保存和imshow会增加少量循环开销。
# 7. 调试重点：运动期间显示的是LAST_YOLO_FRAME缓存，不代表相机实时新画面。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def refresh_full_yolo_window(stage=None, message=None):
    # 【原第483行】声明使用模块级变量：LAST_YOLO_FRAME，赋值会影响后续循环。
    """【函数教学卡：超详细版】
    
    功能：
        生成并刷新全程YOLO大屏：左边检测画面，右边运行参数。
    
    Args:
        stage、message为可选阶段与说明。
    
    Returns:
        None：显示窗口并保存JPG。
    
    调用时机：
        启动、检测、运动、腰部和循环阶段调用。
    
    上下游逻辑：
        只更新UI_STATE和画面，不改变机器人目标。
    
    注意事项与调试重点：
        运动期间多为最后检测帧缓存，并非持续获取新相机帧。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    global LAST_YOLO_FRAME
    # 【原第484行】条件判断：stage is not None；满足时执行下面缩进代码，否则走else（若有）。
    if stage is not None:
        # 【原第485行】计算/保存 UI_STATE['stage']：str(stage)。
        UI_STATE["stage"] = str(stage)
    # 【原第486行】条件判断：message is not None；满足时执行下面缩进代码，否则走else（若有）。
    if message is not None:
        # 【原第487行】计算/保存 UI_STATE['message']：str(message)。
        UI_STATE["message"] = str(message)
    # 【原第488行】计算/保存 frame：LAST_YOLO_FRAME。
    frame = LAST_YOLO_FRAME
    # 【原第489行】条件判断：frame is None；满足时执行下面缩进代码，否则走else（若有）。
    if frame is None:
        # 【原第490行】计算/保存 frame：np.zeros((720, 960, 3), dtype=np.uint8)。
        frame = np.zeros((720,960,3), dtype=np.uint8)
        # 【原第491行】执行调用/更新：_ui_put(frame, 'Waiting for YOLO image...', 40, 70, (0, 220, 255), 0.85, 2)。
        _ui_put(frame, "Waiting for YOLO image...", 40, 70, (0,220,255), 0.85, 2)
    # 【原第492行】否则分支：前面的条件不成立时执行这里。
    else:
        # 【原第493行】计算/保存 frame：frame.copy()。
        frame = frame.copy()
    # 【原第494行】计算/保存 image_width：FULL_UI_WINDOW_WIDTH - FULL_UI_PANEL_WIDTH。
    image_width = FULL_UI_WINDOW_WIDTH - FULL_UI_PANEL_WIDTH
    # 【原第495行】计算/保存 (ih, iw)：frame.shape[:2]。
    ih, iw = frame.shape[:2]
    # 【原第496行】计算/保存 scale：min(image_width / max(iw, 1), FULL_UI_WINDOW_HEIGHT / max(ih, 1))。
    scale = min(image_width/max(iw,1), FULL_UI_WINDOW_HEIGHT/max(ih,1))
    # 【原第497行】计算/保存 resized：cv2.resize(frame, (max(1, int(iw * scale)), max(1, int(ih * scale))))。
    resized = cv2.resize(frame, (max(1,int(iw*scale)), max(1,int(ih*scale))))
    # 【原第498行】计算/保存 canvas：np.zeros((FULL_UI_WINDOW_HEIGHT, FULL_UI_WINDOW_WIDTH, 3), dtype=np.uint8)。
    canvas = np.zeros((FULL_UI_WINDOW_HEIGHT,FULL_UI_WINDOW_WIDTH,3), dtype=np.uint8)
    # 【原第499行】计算/保存 ox：(image_width - resized.shape[1]) // 2。 计算/保存 oy：(FULL_UI_WINDOW_HEIGHT - resized.shape[0]) // 2。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    ox=(image_width-resized.shape[1])//2; oy=(FULL_UI_WINDOW_HEIGHT-resized.shape[0])//2
    # 【原第500行】计算/保存 canvas[oy:oy + resized.shape[0], ox:ox + resized.shape[1]]：resized。
    canvas[oy:oy+resized.shape[0],ox:ox+resized.shape[1]]=resized
    # 【原第501行】计算/保存 px：image_width。 计算/保存 canvas[:, px:]：(22, 22, 22)。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    px=image_width; canvas[:,px:]=(22,22,22)
    # 【原第502行】执行调用/更新：cv2.line(canvas, (px, 0), (px, FULL_UI_WINDOW_HEIGHT), (90, 90, 90), 2)。
    cv2.line(canvas,(px,0),(px,FULL_UI_WINDOW_HEIGHT),(90,90,90),2)
    # 【原第503行】计算/保存 x：px + 20。 计算/保存 y：32。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    x=px+20; y=32
    # 【原第504行】执行调用/更新：_ui_put(canvas, 'G2 V90 YOLO STABILITY GATE', x, y, (0, 220, 255), 0.66, 2)。 计算/保存 y：30。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    _ui_put(canvas,"G2 V90 YOLO STABILITY GATE",x,y,(0,220,255),0.66,2); y+=30
    # 【原第505行】执行调用/更新：_ui_put(canvas, f"CYCLE {UI_STATE['cycle']}/{CYCLE_COUNT}", x, y, (100, 255, 100), 0.58, 2)。 计算/保存 y：26。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    _ui_put(canvas,f"CYCLE {UI_STATE['cycle']}/{CYCLE_COUNT}",x,y,(100,255,100),0.58,2); y+=26
    # 【原第506行】执行调用/更新：_ui_put(canvas, f"STAGE {UI_STATE['stage']}", x, y, (90, 220, 255), 0.48, 1)。 计算/保存 y：22。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    _ui_put(canvas,f"STAGE {UI_STATE['stage']}",x,y,(90,220,255),0.48,1); y+=22
    # 【原第507行】执行调用/更新：_ui_put(canvas, UI_STATE['message'][:58], x, y, (190, 220, 255), 0.42, 1)。 计算/保存 y：24。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    _ui_put(canvas,UI_STATE['message'][:58],x,y,(190,220,255),0.42,1); y+=24
    # 【原第508行】执行调用/更新：cv2.line(canvas, (x, y), (FULL_UI_WINDOW_WIDTH - 15, y), (70, 70, 70), 1)。 计算/保存 y：24。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    cv2.line(canvas,(x,y),(FULL_UI_WINDOW_WIDTH-15,y),(70,70,70),1); y+=24
    # 【原第509行】计算/保存 lines：[f"CONF              {_ui_fmt(UI_STATE['confidence'], 3)}", f"POSITION SAMPLES  {UI_STATE['position_samples']}/{DETECTION_SAMPLES}", f"ANGLE SAMPLES  …。
    lines=[
        # 【原第510行】继续上方跨行表达式的参数/数据项；所属模块：refresh_full_yolo_window。
        f"CONF              {_ui_fmt(UI_STATE['confidence'],3)}",
        # 【原第511行】继续上方跨行表达式的参数/数据项；所属模块：refresh_full_yolo_window。
        f"POSITION SAMPLES  {UI_STATE['position_samples']}/{DETECTION_SAMPLES}",
        # 【原第512行】继续上方跨行表达式的参数/数据项；所属模块：refresh_full_yolo_window。
        f"ANGLE SAMPLES     {UI_STATE['angle_samples']}/{MIN_ANGLE_SAMPLES}",
        # 【原第513行】继续上方跨行表达式的参数/数据项；所属模块：refresh_full_yolo_window。
        f"STABLE BATCHES    {UI_STATE['stable_batches']}/{STABILITY_BATCHES_REQUIRED}",
        # 【原第514行】继续上方跨行表达式的参数/数据项；所属模块：refresh_full_yolo_window。
        f"Z RANGE           {_ui_fmt(None if UI_STATE['z_range_m'] is None else UI_STATE['z_range_m']*1000,1,' mm')}",
        # 【原第515行】继续上方跨行表达式的参数/数据项；所属模块：refresh_full_yolo_window。
        f"X STD             {_ui_fmt(None if UI_STATE['x_std_m'] is None else UI_STATE['x_std_m']*1000,1,' mm')}",
        # 【原第516行】继续上方跨行表达式的参数/数据项；所属模块：refresh_full_yolo_window。
        f"Y STD             {_ui_fmt(None if UI_STATE['y_std_m'] is None else UI_STATE['y_std_m']*1000,1,' mm')}",
        # 【原第517行】继续上方跨行表达式的参数/数据项；所属模块：refresh_full_yolo_window。
        f"YAW SPREAD        {_ui_fmt(UI_STATE['yaw_spread_deg'],1,' deg')}",
        # 【原第518行】继续上方跨行表达式的参数/数据项；所属模块：refresh_full_yolo_window。
        f"YAW               {_ui_fmt(UI_STATE['yaw_deg'],2,' deg')}",
        # 【原第519行】继续上方跨行表达式的参数/数据项；所属模块：refresh_full_yolo_window。
        f"LOCKED TABLE Z    {_ui_fmt(UI_STATE['table_z_m'],5,' m')}",
        # 【原第520行】继续上方跨行表达式的参数/数据项；所属模块：refresh_full_yolo_window。
        f"PRODUCT HEIGHT    {FIXED_PRODUCT_HEIGHT_M*1000:.1f} mm",
    # 【原第521行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
    ]
    # 【原第522行】循环：依次从 lines 取值到 line，执行下面缩进块。
    for line in lines:
        # 【原第523行】执行调用/更新：_ui_put(canvas, line, x, y)。 计算/保存 y：21。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        _ui_put(canvas,line,x,y); y+=21
    # 【原第524行】循环：依次从 (('PRODUCT BASE XYZ', 'product_xyz'), ('PREGRASP XYZ', 'pregrasp_xyz'), ('GRASP XYZ', 'grasp_xyz')) 取值到 (title, key)，执行下面缩进块。
    for title,key in (("PRODUCT BASE XYZ","product_xyz"),("PREGRASP XYZ","pregrasp_xyz"),("GRASP XYZ","grasp_xyz")):
        # 【原第525行】计算/保存 y：8。 执行调用/更新：_ui_put(canvas, title, x, y, (255, 190, 80), 0.48, 1)。 计算/保存 y：20。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        y+=8; _ui_put(canvas,title,x,y,(255,190,80),0.48,1); y+=20
        # 【原第526行】计算/保存 xyz：UI_STATE.get(key) or [None, None, None]。
        xyz=UI_STATE.get(key) or [None,None,None]
        # 【原第527行】循环：依次从 zip('XYZ', xyz) 取值到 (n, v)，执行下面缩进块。
        for n,v in zip("XYZ",xyz):
            # 【原第528行】执行调用/更新：_ui_put(canvas, f"  {n}  {_ui_fmt(v, 5, ' m')}", x, y)。 计算/保存 y：20。
            # 【本行拆解教学：分号语句】
            # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
            # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
            # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
            _ui_put(canvas,f"  {n}  {_ui_fmt(v,5,' m')}",x,y); y+=20
    # 【原第529行】执行调用/更新：_ui_put(canvas, 'Q/ESC: close display only', x, FULL_UI_WINDOW_HEIGHT - 18, (130, 130, 130), 0.38, 1)。
    _ui_put(canvas,"Q/ESC: close display only",x,FULL_UI_WINDOW_HEIGHT-18,(130,130,130),0.38,1)
    # 【原第530行】开始异常处理块：try执行正常操作，except处理异常，finally执行清理（若有）。
    try:
        # 【原第531行】执行调用/更新：cv2.imwrite(FULL_UI_IMAGE_FILE, canvas)。
        cv2.imwrite(FULL_UI_IMAGE_FILE,canvas)
        # 【原第532行】条件判断：SHOW_YOLO_WINDOW；满足时执行下面缩进代码，否则走else（若有）。
        if SHOW_YOLO_WINDOW:
            # 【原第533行】执行调用/更新：cv2.imshow(WINDOW_NAME, canvas)。 执行调用/更新：cv2.waitKey(1)。
            # 【本行拆解教学：分号语句】
            # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
            # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
            # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
            cv2.imshow(WINDOW_NAME,canvas); cv2.waitKey(1)
    # 【原第534行】异常分支：捕获对应异常，执行下面缩进代码；保持原处理方式。
    except cv2.error as exc:
        # 【原第535行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print(f"[V90窗口] {exc}")


# 【原第538行】缓存当前检测画面，从文字中解析置信度和样本数，再刷新大屏。
# ======================================================================================
# 【函数教学卡】show_frame()
# 1. 功能：接收当前YOLO标注图，缓存图像并从简短文本中抽取置信度/样本数。
# 2. 输入：frame和lines。
# 3. 输出：无return。
# 4. 谁调用：detect_product_batch。
# 5. 结果流向：调用refresh_full_yolo_window。
# 6. G2实际意义：不移动。
# 7. 调试重点：lines格式必须形如conf=、position=、angle=，否则解析不到面板。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def show_frame(frame,lines):
    # 【原第539行】声明使用模块级变量：LAST_YOLO_FRAME, LAST_YOLO_TEXT，赋值会影响后续循环。
    """【函数教学卡：超详细版】
    
    功能：
        缓存当前YOLO标注图，并解析置信度、位置样本数和角度样本数。
    
    Args:
        frame: 标注图；lines: 简短状态字符串列表。
    
    Returns:
        None。
    
    调用时机：
        detect_product_batch()在视觉循环中调用。
    
    上下游逻辑：
        随后刷新完整大屏。
    
    注意事项与调试重点：
        lines格式需要保留conf=、position=、angle=。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    global LAST_YOLO_FRAME, LAST_YOLO_TEXT
    # 【原第540行】缓存最后一次识别图像；动作阶段刷新通常使用该缓存。 计算/保存 LAST_YOLO_TEXT：list(lines or [])。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    LAST_YOLO_FRAME=frame.copy(); LAST_YOLO_TEXT=list(lines or [])
    # 【原第541行】循环：依次从 LAST_YOLO_TEXT 取值到 item，执行下面缩进块。
    for item in LAST_YOLO_TEXT:
        # 【原第542行】计算/保存 text：str(item).strip().lower()。
        text=str(item).strip().lower()
        # 【原第543行】开始异常处理块：try执行正常操作，except处理异常，finally执行清理（若有）。
        try:
            # 【原第544行】条件判断：text.startswith('conf=')；满足时执行下面缩进代码，否则走else（若有）。 计算/保存 UI_STATE['confidence']：float(text.split('=', 1)[1])。
            if text.startswith("conf="): UI_STATE["confidence"]=float(text.split("=",1)[1])
            # 【原第545行】条件判断：text.startswith('position=')；满足时执行下面缩进代码，否则走else（若有）。 计算/保存 UI_STATE['position_samples']：int(text.split('=', 1)[1].split('/', 1)[0])。
            elif text.startswith("position="): UI_STATE["position_samples"]=int(text.split("=",1)[1].split("/",1)[0])
            # 【原第546行】条件判断：text.startswith('angle=')；满足时执行下面缩进代码，否则走else（若有）。 计算/保存 UI_STATE['angle_samples']：int(text.split('=', 1)[1].split('/', 1)[0])。
            elif text.startswith("angle="): UI_STATE["angle_samples"]=int(text.split("=",1)[1].split("/",1)[0])
        # 【原第547行】异常分支：捕获对应异常，执行下面缩进代码；保持原处理方式。
        except Exception:
            # 【原第548行】此分支不执行额外操作；原程序选择忽略该异常或情况。
            pass
    # 【原第549行】执行调用/更新：refresh_full_yolo_window('YOLO DETECTION', '持续识别并等待产品稳定')。
    refresh_full_yolo_window("YOLO DETECTION","持续识别并等待产品稳定")
# 【原第550行】单批视觉：YOLO框→深度→Base XYZ→桌面外围采样→轮廓长轴；返回统计结果。
# ======================================================================================
# 【函数教学卡】detect_product_batch()
# 1. 功能：完成一批产品视觉测量：YOLO框、深度三维点、桌面Z、轮廓长轴和统计。
# 2. 输入：YOLO模型、相机、TF接口、报告。
# 3. 输出：data字典，含base_xyz、confidence、yaw、table_z等。
# 4. 谁调用：detect_product稳定门控。
# 5. 结果流向：外层把多个批次合并，稳定后main规划抓取。
# 6. G2实际意义：停在WAYPOINT_2读取相机，不移动机械臂。
# 7. 调试重点：依次看no_image/no_detection/no_depth/no_angle/no_table和angle_rejects。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def detect_product_batch(model,camera,tf_api,report):
    # 【原第551行】条件判断：SHOW_YOLO_WINDOW；满足时执行下面缩进代码，否则走else（若有）。
    """【函数教学卡：超详细版】
    
    功能：
        执行一次完整视觉采样批次：检测、深度、Base XYZ、桌面Z和长轴。
    
    Args:
        model、camera、tf_api、report。
    
    Returns:
        Dict：批次检测结果与统计。
    
    调用时机：
        detect_product()反复调用。
    
    上下游逻辑：
        多个批次由稳定门控合并后交给main。
    
    注意事项与调试重点：
        机器人停在WAYPOINT_2；重点查看拒绝统计和角度轮廓。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    if SHOW_YOLO_WINDOW:
        # 【原第552行】开始异常处理块：try执行正常操作，except处理异常，finally执行清理（若有）。
        try:
            # 【原第553行】执行调用/更新：cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)。
            cv2.namedWindow(WINDOW_NAME,cv2.WINDOW_NORMAL)
            # 【原第554行】执行调用/更新：cv2.resizeWindow(WINDOW_NAME, 960, 540)。 执行调用/更新：cv2.moveWindow(WINDOW_NAME, 40, 40)。
            # 【本行拆解教学：分号语句】
            # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
            # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
            # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
            cv2.resizeWindow(WINDOW_NAME,960,540); cv2.moveWindow(WINDOW_NAME,40,40)
        # 【原第555行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        except cv2.error as exc: print(f"[YOLO窗口创建失败] {exc}")
    # 【原第556行】计算/保存 intr：camera.get_camera_intrinsic(agibot_gdk.CameraType.kHeadDepth)。
    intr=camera.get_camera_intrinsic(agibot_gdk.CameraType.kHeadDepth)
    # 【原第557行】计算/保存 (fx, fy, cx, cy)：map(float, list(intr.intrinsic)[:4])。
    fx,fy,cx,cy=map(float,list(intr.intrinsic)[:4])
    # 【原第558行】相机点到Base的变换矩阵；变量名容易误解，实际用于b2c@cam。
    b2c=tf_matrix(tf_api.get_tf_from_base_link(HEAD_FRAME)) @ tf_matrix(
        # 【原第559行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
        tf_api.get_tf_from_sensor(agibot_gdk.SensorExtrinsicType.kHeadRGBDToHeadLink3))
    # 【原第560行】计算/保存 pos：[]。 计算/保存 angles：[]。 计算/保存 table_z_samples：[]。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    pos=[]; angles=[]; table_z_samples=[]
    # 【原第561行】计算/保存 rejects：{'no_image': 0, 'no_detection': 0, 'no_depth': 0, 'no_angle': 0, 'no_table': 0}。
    rejects={"no_image":0,"no_detection":0,"no_depth":0,"no_angle":0,"no_table":0}
    # 【原第562行】计算/保存 angle_rejects：{'area_too_small': 0, 'area_too_large': 0, 'touches_border': 0, 'aspect_too_small': 0, 'center_too_far': 0, 'no_valid_contour': 0, 'invalid_axis': 0}。
    angle_rejects={
        # 【原第563行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
        "area_too_small":0,
        # 【原第564行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
        "area_too_large":0,
        # 【原第565行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
        "touches_border":0,
        # 【原第566行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
        "aspect_too_small":0,
        # 【原第567行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
        "center_too_far":0,
        # 【原第568行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
        "no_valid_contour":0,
        # 【原第569行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
        "invalid_axis":0,
    # 【原第570行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
    }
    # 【原第571行】计算/保存 last：None。 计算/保存 deadline：time.time() + DETECTION_TIMEOUT_S。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    last=None; deadline=time.time()+DETECTION_TIMEOUT_S
    # 【原第572行】重复执行下面代码，直到条件不成立：time.time() < deadline and (len(pos) < DETECTION_SAMPLES or len(angles) < MIN_ANGLE_SAMPLES)。
    while time.time()<deadline and (
        # 【原第573行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
        len(pos)<DETECTION_SAMPLES or len(angles)<MIN_ANGLE_SAMPLES
    # 【原第574行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
    ):
        # 【原第575行】计算/保存 co：safe_image(camera, agibot_gdk.CameraType.kHeadColor)。
        co=safe_image(camera,agibot_gdk.CameraType.kHeadColor)
        # 【原第576行】计算/保存 do：safe_image(camera, agibot_gdk.CameraType.kHeadDepth)。
        do=safe_image(camera,agibot_gdk.CameraType.kHeadDepth)
        # 【原第577行】条件判断：co is None or do is None；满足时执行下面缩进代码，否则走else（若有）。 计算/保存 rejects['no_image']：1。 跳过本轮剩余代码，进入当前循环的下一轮。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 【continue教学】continue会跳过当前循环本轮剩余代码，立即回到for/while顶部开始下一轮。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        if co is None or do is None: rejects["no_image"]+=1; continue
        # 【原第578行】计算/保存 color：decode_color(co)。 计算/保存 depth：decode_depth(do)。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        color=decode_color(co); depth=decode_depth(do)
        # 【原第579行】条件判断：color is None or depth is None；满足时执行下面缩进代码，否则走else（若有）。 计算/保存 rejects['no_image']：1。 跳过本轮剩余代码，进入当前循环的下一轮。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 【continue教学】continue会跳过当前循环本轮剩余代码，立即回到for/while顶部开始下一轮。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        if color is None or depth is None: rejects["no_image"]+=1; continue
        # 【原第580行】计算/保存 result：model.predict(color, conf=MIN_CONFIDENCE, verbose=False)[0]。
        result=model.predict(color,conf=MIN_CONFIDENCE,verbose=False)[0]
        # 【原第581行】条件判断：result.boxes is None or not len(result.boxes)；满足时执行下面缩进代码，否则走else（若有）。
        if result.boxes is None or not len(result.boxes):
            # 【原第582行】计算/保存 rejects['no_detection']：1。 执行调用/更新：show_frame(color, ['YOLO: NO DETECTION'])。 暂停当前线程：time.sleep(0.03)；此等待不代表已验证机器人到位。 跳过本轮剩余代码，进入当前循环的下一轮。
            # 【本行拆解教学：分号语句】
            # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
            # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
            # 第1步 rejects["no_detection"] += 1：把“没有检测到产品”的累计次数增加1，便于报告统计。
            # 第2步 show_frame(...)：继续显示当前颜色画面，并在界面提示“YOLO: NO DETECTION”。
            # 第3步 time.sleep(0.03)：让当前Python线程暂停0.03秒，也就是30毫秒。
            #      作用是避免未检测到目标时while循环以最高速度空转，降低CPU占用，并给相机/界面一点刷新时间。
            #      这30毫秒期间本行不会发送机械臂、腰部、头部或夹爪运动命令。
            #      sleep只代表等待时间，不代表YOLO已经识别成功，也不代表机器人已经到位。
            # 第4步 continue：立即跳过本次while循环后面的全部检测处理，返回while顶部开始下一帧检测。
            # 因此整行的人话含义是：“记录一次未识别→显示未识别→短暂停30ms→重新检测下一帧”。
            rejects["no_detection"]+=1; show_frame(color,["YOLO: NO DETECTION"]); time.sleep(0.03); continue
        # 【原第583行】计算/保存 box：max(result.boxes, key=lambda b: float(b.conf[0]))。 计算/保存 conf：float(box.conf[0])。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        box=max(result.boxes,key=lambda b:float(b.conf[0])); conf=float(box.conf[0])
        # 【原第584行】计算/保存 (raw_x1, raw_y1, raw_x2, raw_y2)：map(int, box.xyxy[0].tolist())。 计算/保存 (h, w)：color.shape[:2]。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        raw_x1,raw_y1,raw_x2,raw_y2=map(int,box.xyxy[0].tolist()); h,w=color.shape[:2]
        # 【原第585行】计算/保存 box_width：max(1, raw_x2 - raw_x1)。
        box_width=max(1,raw_x2-raw_x1)
        # 【原第586行】计算/保存 box_height：max(1, raw_y2 - raw_y1)。
        box_height=max(1,raw_y2-raw_y1)
        # 【原第587行】计算/保存 padding_x：int(round(box_width * ROI_PADDING_RATIO))。
        padding_x=int(round(box_width*ROI_PADDING_RATIO))
        # 【原第588行】计算/保存 padding_y：int(round(box_height * ROI_PADDING_RATIO))。
        padding_y=int(round(box_height*ROI_PADDING_RATIO))
        # 【原第589行】计算/保存 x1：max(0, raw_x1 - padding_x)。
        x1=max(0,raw_x1-padding_x)
        # 【原第590行】计算/保存 y1：max(0, raw_y1 - padding_y)。
        y1=max(0,raw_y1-padding_y)
        # 【原第591行】计算/保存 x2：min(w - 1, raw_x2 + padding_x)。
        x2=min(w-1,raw_x2+padding_x)
        # 【原第592行】计算/保存 y2：min(h - 1, raw_y2 + padding_y)。
        y2=min(h-1,raw_y2+padding_y)
        # 【原第593行】计算/保存 px：(x1 + x2) // 2。 计算/保存 py：(y1 + y2) // 2。 计算/保存 (dh, dw)：depth.shape[:2]。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        px=(x1+x2)//2; py=(y1+y2)//2; dh,dw=depth.shape[:2]
        # 【原第594行】计算/保存 u：max(0, min(dw - 1, int(round(px * dw / w))))。 计算/保存 v：max(0, min(dh - 1, int(round(py * dh / h))))。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        u=max(0,min(dw-1,int(round(px*dw/w)))); v=max(0,min(dh-1,int(round(py*dh/h))))
        # 【原第595行】计算/保存 raw：median_depth(depth, u, v)。
        raw=median_depth(depth,u,v)
        # 【原第596行】计算/保存 annotated：color.copy()。 执行调用/更新：cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        annotated=color.copy(); cv2.rectangle(annotated,(x1,y1),(x2,y2),(0,255,0),2)
        # 【原第597行】执行调用/更新：cv2.circle(annotated, (px, py), 5, (0, 255, 255), -1)。
        cv2.circle(annotated,(px,py),5,(0,255,255),-1)
        # 【原第598行】条件判断：raw is None；满足时执行下面缩进代码，否则走else（若有）。
        if raw is None:
            # 【原第599行】计算/保存 rejects['no_depth']：1。 执行调用/更新：show_frame(annotated, ['YOLO DETECTED', 'DEPTH INVALID'])。 跳过本轮剩余代码，进入当前循环的下一轮。
            # 【本行拆解教学：分号语句】
            # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
            # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
            # 【continue教学】continue会跳过当前循环本轮剩余代码，立即回到for/while顶部开始下一轮。
            # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
            rejects["no_depth"]+=1; show_frame(annotated,["YOLO DETECTED","DEPTH INVALID"]); continue
        # 【原第600行】计算/保存 z：raw / 1000.0 if raw > 20 else raw。
        z=raw/1000.0 if raw>20 else raw
        # 【原第601行】计算/保存 cam：np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])。 计算/保存 base：(b2c @ cam)[:3]。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        cam=np.array([(u-cx)*z/fx,(v-cy)*z/fy,z,1.0]); base=(b2c@cam)[:3]
        # 【原第602行】条件判断：len(pos) < DETECTION_SAMPLES；满足时执行下面缩进代码，否则走else（若有）。
        if len(pos)<DETECTION_SAMPLES:
            # 【原第603行】执行调用/更新：pos.append(base.tolist())。
            pos.append(base.tolist())

        # 在检测框外围采样桌面深度，转换为Base Z，用于防撞硬保护。
        # 【原第606行】计算/保存 bw：max(1, x2 - x1)。 计算/保存 bh：max(1, y2 - y1)。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        bw=max(1,x2-x1); bh=max(1,y2-y1)
        # 【原第607行】计算/保存 mx：max(8, int(round(bw * TABLE_RING_MARGIN_RATIO)))。
        mx=max(8,int(round(bw*TABLE_RING_MARGIN_RATIO)))
        # 【原第608行】计算/保存 my：max(8, int(round(bh * TABLE_RING_MARGIN_RATIO)))。
        my=max(8,int(round(bh*TABLE_RING_MARGIN_RATIO)))
        # 【原第609行】计算/保存 ring_color_points：[(x1 - mx, y1 - my), (x2 + mx, y1 - my), (x1 - mx, y2 + my), (x2 + mx, y2 + my), ((x1 + x2) // 2, y1 - my), ((x1 + x2) // 2, y2 + my), (x1 - mx, (y1 +…。
        ring_color_points=[
            # 【原第610行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
            (x1-mx,y1-my),(x2+mx,y1-my),(x1-mx,y2+my),(x2+mx,y2+my),
            # 【原第611行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
            ((x1+x2)//2,y1-my),((x1+x2)//2,y2+my),
            # 【原第612行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
            (x1-mx,(y1+y2)//2),(x2+mx,(y1+y2)//2),
        # 【原第613行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
        ]
        # 【原第614行】计算/保存 frame_table：[]。
        frame_table=[]
        # 【原第615行】循环：依次从 ring_color_points 取值到 (tx, ty)，执行下面缩进块。
        for tx,ty in ring_color_points:
            # 【原第616行】计算/保存 tx：max(0, min(w - 1, int(tx)))。 计算/保存 ty：max(0, min(h - 1, int(ty)))。
            # 【本行拆解教学：分号语句】
            # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
            # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
            # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
            tx=max(0,min(w-1,int(tx))); ty=max(0,min(h-1,int(ty)))
            # 【原第617行】计算/保存 tu：max(0, min(dw - 1, int(round(tx * dw / w))))。
            tu=max(0,min(dw-1,int(round(tx*dw/w))))
            # 【原第618行】计算/保存 tv：max(0, min(dh - 1, int(round(ty * dh / h))))。
            tv=max(0,min(dh-1,int(round(ty*dh/h))))
            # 【原第619行】计算/保存 traw：median_depth(depth, tu, tv)。
            traw=median_depth(depth,tu,tv)
            # 【原第620行】条件判断：traw is None；满足时执行下面缩进代码，否则走else（若有）。 跳过本轮剩余代码，进入当前循环的下一轮。
            if traw is None: continue
            # 【原第621行】计算/保存 tz：traw / 1000.0 if traw > 20 else traw。
            tz=traw/1000.0 if traw>20 else traw
            # 【原第622行】计算/保存 tcam：np.array([(tu - cx) * tz / fx, (tv - cy) * tz / fy, tz, 1.0])。
            tcam=np.array([(tu-cx)*tz/fx,(tv-cy)*tz/fy,tz,1.0])
            # 【原第623行】计算/保存 tbase：(b2c @ tcam)[:3]。
            tbase=(b2c@tcam)[:3]
            # 【原第624行】执行调用/更新：frame_table.append(float(tbase[2]))。
            frame_table.append(float(tbase[2]))
        # 【原第625行】条件判断：frame_table；满足时执行下面缩进代码，否则走else（若有）。
        if frame_table:
            # 【原第626行】执行调用/更新：table_z_samples.extend(frame_table)。
            table_z_samples.extend(frame_table)
        # 【原第627行】否则分支：前面的条件不成立时执行这里。
        else:
            # 【原第628行】计算/保存 rejects['no_table']：1。
            rejects["no_table"]+=1

        # 【原第630行】计算/保存 roi：color[y1:y2 + 1, x1:x2 + 1]。
        roi=color[y1:y2+1,x1:x2+1]
        # 【原第631行】计算/保存 gray：cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)。 计算/保存 blur：cv2.GaussianBlur(gray, (5, 5), 0)。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        gray=cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY); blur=cv2.GaussianBlur(gray,(5,5),0)
        # 【原第632行】计算/保存 masks：[]。
        masks=[]
        # 【原第633行】计算/保存 edge：cv2.Canny(blur, 30, 110)。
        edge=cv2.Canny(blur,30,110)
        # 【原第634行】计算/保存 edge：cv2.morphologyEx(edge, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)。
        edge=cv2.morphologyEx(edge,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8),iterations=2)
        # 【原第635行】执行调用/更新：masks.append(edge)。
        masks.append(edge)
        # 【原第636行】计算/保存 (_, otsu)：cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)。
        _,otsu=cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        # 【原第637行】计算/保存 masks：[otsu, cv2.bitwise_not(otsu)]。
        masks += [otsu,cv2.bitwise_not(otsu)]
        # 【原第638行】计算/保存 best：None。 计算/保存 roi_area：max(1, roi.shape[0] * roi.shape[1])。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        best=None; roi_area=max(1,roi.shape[0]*roi.shape[1])
        # 【原第639行】循环：依次从 masks 取值到 mask，执行下面缩进块。
        for mask in masks:
            # 【原第640行】计算/保存 (contours, _)：cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)。
            contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            # 【原第641行】循环：依次从 contours 取值到 c，执行下面缩进块。
            for c in contours:
                # 【原第642行】计算/保存 area：float(cv2.contourArea(c))。
                area=float(cv2.contourArea(c))
                # 【原第643行】计算/保存 area_ratio：area / roi_area。
                area_ratio=area/roi_area
                # 【原第644行】条件判断：area_ratio < MIN_CONTOUR_AREA_RATIO；满足时执行下面缩进代码，否则走else（若有）。
                if area_ratio<MIN_CONTOUR_AREA_RATIO:
                    # 【原第645行】计算/保存 angle_rejects['area_too_small']：1。
                    angle_rejects["area_too_small"]+=1
                    # 【原第646行】跳过本轮剩余代码，进入当前循环的下一轮。
                    continue
                # 【原第647行】条件判断：area_ratio > MAX_CONTOUR_AREA_RATIO；满足时执行下面缩进代码，否则走else（若有）。
                if area_ratio>MAX_CONTOUR_AREA_RATIO:
                    # 【原第648行】计算/保存 angle_rejects['area_too_large']：1。
                    angle_rejects["area_too_large"]+=1
                    # 【原第649行】跳过本轮剩余代码，进入当前循环的下一轮。
                    continue

                # 【原第651行】计算/保存 (contour_x, contour_y, contour_w, contour_h)：cv2.boundingRect(c)。
                contour_x,contour_y,contour_w,contour_h=cv2.boundingRect(c)
                # 【原第652行】计算/保存 touches_border：contour_x <= CONTOUR_BORDER_MARGIN_PX or contour_y <= CONTOUR_BORDER_MARGIN_PX or contour_x + contour_w >= roi.shape[1] - CONTOUR_BORDER_MARGIN_PX or …。
                touches_border=(
                    # 【原第653行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
                    contour_x<=CONTOUR_BORDER_MARGIN_PX
                    # 【原第654行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
                    or contour_y<=CONTOUR_BORDER_MARGIN_PX
                    # 【原第655行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
                    or contour_x+contour_w>=roi.shape[1]-CONTOUR_BORDER_MARGIN_PX
                    # 【原第656行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
                    or contour_y+contour_h>=roi.shape[0]-CONTOUR_BORDER_MARGIN_PX
                # 【原第657行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
                )
                # 【原第658行】条件判断：touches_border；满足时执行下面缩进代码，否则走else（若有）。
                if touches_border:
                    # 【原第659行】计算/保存 angle_rejects['touches_border']：1。
                    angle_rejects["touches_border"]+=1
                    # 【原第660行】跳过本轮剩余代码，进入当前循环的下一轮。
                    continue

                # 【原第662行】计算/保存 rect：cv2.minAreaRect(c)。
                rect=cv2.minAreaRect(c)
                # 【原第663行】计算/保存 ((rect_center_x, rect_center_y), (rw, rh), _)：rect。
                (rect_center_x,rect_center_y),(rw,rh),_=rect
                # 【原第664行】条件判断：min(rw, rh) < 3；满足时执行下面缩进代码，否则走else（若有）。
                if min(rw,rh)<3:
                    # 【原第665行】计算/保存 angle_rejects['aspect_too_small']：1。
                    angle_rejects["aspect_too_small"]+=1
                    # 【原第666行】跳过本轮剩余代码，进入当前循环的下一轮。
                    continue
                # 【原第667行】计算/保存 aspect_ratio：max(rw, rh) / max(1.0, min(rw, rh))。
                aspect_ratio=max(rw,rh)/max(1.0,min(rw,rh))
                # 【原第668行】条件判断：aspect_ratio < MIN_ASPECT_RATIO；满足时执行下面缩进代码，否则走else（若有）。
                if aspect_ratio<MIN_ASPECT_RATIO:
                    # 【原第669行】计算/保存 angle_rejects['aspect_too_small']：1。
                    angle_rejects["aspect_too_small"]+=1
                    # 【原第670行】跳过本轮剩余代码，进入当前循环的下一轮。
                    continue

                # 【原第672行】计算/保存 roi_center_x：roi.shape[1] / 2.0。
                roi_center_x=roi.shape[1]/2.0
                # 【原第673行】计算/保存 roi_center_y：roi.shape[0] / 2.0。
                roi_center_y=roi.shape[0]/2.0
                # 【原第674行】计算/保存 center_distance：math.hypot(rect_center_x - roi_center_x, rect_center_y - roi_center_y)。
                center_distance=math.hypot(
                    # 【原第675行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
                    rect_center_x-roi_center_x,
                    # 【原第676行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
                    rect_center_y-roi_center_y,
                # 【原第677行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
                )
                # 【原第678行】计算/保存 center_distance_ratio：center_distance / max(roi.shape[1], roi.shape[0])。
                center_distance_ratio=center_distance/max(roi.shape[1],roi.shape[0])
                # 【原第679行】条件判断：center_distance_ratio > MAX_CONTOUR_CENTER_DISTANCE_RATIO；满足时执行下面缩进代码，否则走else（若有）。
                if center_distance_ratio>MAX_CONTOUR_CENTER_DISTANCE_RATIO:
                    # 【原第680行】计算/保存 angle_rejects['center_too_far']：1。
                    angle_rejects["center_too_far"]+=1
                    # 【原第681行】跳过本轮剩余代码，进入当前循环的下一轮。
                    continue

                # 【原第683行】计算/保存 pts：cv2.boxPoints(rect)。 计算/保存 longest：None。
                # 【本行拆解教学：分号语句】
                # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
                # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
                # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
                pts=cv2.boxPoints(rect); longest=None
                # 【原第684行】循环：依次从 range(4) 取值到 i，执行下面缩进块。
                for i in range(4):
                    # 【原第685行】计算/保存 a：pts[i]。 计算/保存 b：pts[(i + 1) % 4]。
                    # 【本行拆解教学：分号语句】
                    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
                    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
                    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
                    a=pts[i]; b=pts[(i+1)%4]
                    # 【原第686行】计算/保存 length：float(np.linalg.norm(b - a))。
                    length=float(np.linalg.norm(b-a))
                    # 【原第687行】条件判断：longest is None or length > longest[0]；满足时执行下面缩进代码，否则走else（若有）。 计算/保存 longest：(length, a, b)。
                    if longest is None or length>longest[0]: longest=(length,a,b)

                # 【原第689行】计算/保存 center_score：max(0.0, 1.0 - center_distance_ratio)。
                center_score=max(0.0,1.0-center_distance_ratio)
                # 【原第690行】计算/保存 border_clearance：min(contour_x, contour_y, roi.shape[1] - (contour_x + contour_w), roi.shape[0] - (contour_y + contour_h))。
                border_clearance=min(
                    # 【原第691行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
                    contour_x,
                    # 【原第692行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
                    contour_y,
                    # 【原第693行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
                    roi.shape[1]-(contour_x+contour_w),
                    # 【原第694行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
                    roi.shape[0]-(contour_y+contour_h),
                # 【原第695行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
                )
                # 【原第696行】计算/保存 score：area_ratio * min(aspect_ratio, 6.0) * center_score * max(1.0, float(border_clearance))。
                score=(
                    # 【原第697行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
                    area_ratio
                    # 【原第698行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
                    * min(aspect_ratio,6.0)
                    # 【原第699行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
                    * center_score
                    # 【原第700行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
                    * max(1.0,float(border_clearance))
                # 【原第701行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
                )
                # 【原第702行】条件判断：best is None or score > best[0]；满足时执行下面缩进代码，否则走else（若有）。 计算/保存 best：(score, longest[1], longest[2])。
                if best is None or score>best[0]: best=(score,longest[1],longest[2])
        # 【原第703行】条件判断：best is not None；满足时执行下面缩进代码，否则走else（若有）。
        if best is not None:
            # 【原第704行】计算/保存 a：best[1] + np.array([x1, y1])。 计算/保存 b：best[2] + np.array([x1, y1])。
            # 【本行拆解教学：分号语句】
            # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
            # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
            # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
            a=best[1]+np.array([x1,y1]); b=best[2]+np.array([x1,y1])
            # 【原第705行】计算/保存 pbase：[]。
            pbase=[]
            # 【原第706行】循环：依次从 (a, b) 取值到 p，执行下面缩进块。
            for p in (a,b):
                # 【原第707行】计算/保存 uu：max(0, min(dw - 1, int(round(p[0] * dw / w))))。 计算/保存 vv：max(0, min(dh - 1, int(round(p[1] * dh / h))))。
                # 【本行拆解教学：分号语句】
                # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
                # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
                # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
                uu=max(0,min(dw-1,int(round(p[0]*dw/w)))); vv=max(0,min(dh-1,int(round(p[1]*dh/h))))
                # 【原第708行】计算/保存 cc：np.array([(uu - cx) * z / fx, (vv - cy) * z / fy, z, 1.0])。 执行调用/更新：pbase.append((b2c @ cc)[:3])。
                # 【本行拆解教学：分号语句】
                # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
                # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
                # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
                cc=np.array([(uu-cx)*z/fx,(vv-cy)*z/fy,z,1.0]); pbase.append((b2c@cc)[:3])
            # 【原第709行】计算/保存 axis：pbase[1] - pbase[0]。
            axis=pbase[1]-pbase[0]
            # 【原第710行】条件判断：np.linalg.norm(axis[:2]) > 1e-05；满足时执行下面缩进代码，否则走else（若有）。
            if np.linalg.norm(axis[:2])>1e-5:
                # 【原第711行】计算/保存 yaw：math.atan2(float(axis[1]), float(axis[0]))。 执行调用/更新：angles.append(yaw)。
                # 【本行拆解教学：分号语句】
                # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
                # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
                # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
                yaw=math.atan2(float(axis[1]),float(axis[0])); angles.append(yaw)
                # 【原第712行】执行调用/更新：cv2.line(annotated, tuple(np.round(a).astype(int)), tuple(np.round(b).astype(int)), (0, 0, 255), 3)。
                cv2.line(annotated,tuple(np.round(a).astype(int)),tuple(np.round(b).astype(int)),(0,0,255),3)
            # 【原第713行】否则分支：前面的条件不成立时执行这里。
            else:
                # 【原第714行】计算/保存 rejects['no_angle']：1。
                rejects["no_angle"]+=1
                # 【原第715行】计算/保存 angle_rejects['invalid_axis']：1。
                angle_rejects["invalid_axis"]+=1
        # 【原第716行】否则分支：前面的条件不成立时执行这里。
        else:
            # 【原第717行】计算/保存 rejects['no_angle']：1。
            rejects["no_angle"]+=1
            # 【原第718行】计算/保存 angle_rejects['no_valid_contour']：1。
            angle_rejects["no_valid_contour"]+=1
        # 【原第719行】计算/保存 last：annotated。
        last=annotated
        # 【原第720行】执行调用/更新：show_frame(annotated, [f'conf={conf:.3f}', f'position={len(pos)}/8', f'angle={len(angles)}'])。
        show_frame(annotated,[f"conf={conf:.3f}",f"position={len(pos)}/8",f"angle={len(angles)}"])
        # 【原第721行】暂停当前线程：time.sleep(0.1)；此等待不代表已验证机器人到位。
        time.sleep(0.1)
    # 【原第722行】条件判断：last is not None；满足时执行下面缩进代码，否则走else（若有）。 执行调用/更新：cv2.imwrite(ANGLE_IMAGE_FILE, last)。
    if last is not None: cv2.imwrite(ANGLE_IMAGE_FILE,last)
    # 【原第723行】计算/保存 report['vision']：{'position_samples': len(pos), 'angle_samples': len(angles), 'rejects': rejects, 'angle_rejects': angle_rejects, 'roi_padding_ratio': ROI_PADDING_RATI…。
    report["vision"]={
        # 【原第724行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
        "position_samples":len(pos),
        # 【原第725行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
        "angle_samples":len(angles),
        # 【原第726行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
        "rejects":rejects,
        # 【原第727行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
        "angle_rejects":angle_rejects,
        # 【原第728行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
        "roi_padding_ratio":ROI_PADDING_RATIO,
    # 【原第729行】调用save保存当前报告；便于定位本轮检测或运动问题。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    };save(report)
    # 【原第730行】条件判断：len(pos) < MIN_POSITION_SAMPLES；满足时执行下面缩进代码，否则走else（若有）。 抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
    if len(pos)<MIN_POSITION_SAMPLES: raise RuntimeError(f"产品位置样本不足: {len(pos)}, {rejects}")
    # 【原第731行】计算/保存 pos_array：np.array(pos, dtype=float)。
    pos_array=np.array(pos,dtype=float)
    # 【原第732行】计算/保存 product_xyz：np.median(pos_array, axis=0).tolist()。
    product_xyz=np.median(pos_array,axis=0).tolist()
    # 【原第733行】计算/保存 product_z_range：float(np.max(pos_array[:, 2]) - np.min(pos_array[:, 2]))。
    product_z_range=float(np.max(pos_array[:,2])-np.min(pos_array[:,2]))
    # 【原第734行】计算/保存 table_z：None。
    table_z=None
    # 【原第735行】条件判断：len(table_z_samples) >= MIN_TABLE_SAMPLES；满足时执行下面缩进代码，否则走else（若有）。
    if len(table_z_samples)>=MIN_TABLE_SAMPLES:
        # 使用较高分位附近的中位思想，降低桌边/地面的干扰。
        # 【原第737行】计算/保存 table_values：np.asarray(table_z_samples, dtype=float)。
        table_values=np.asarray(table_z_samples,dtype=float)
        # 【原第738行】计算/保存 (q25, q75)：np.percentile(table_values, [25, 75])。
        q25,q75=np.percentile(table_values,[25,75])
        # 【原第739行】计算/保存 central：table_values[(table_values >= q25) & (table_values <= q75)]。
        central=table_values[(table_values>=q25)&(table_values<=q75)]
        # 【原第740行】计算/保存 table_z：float(np.median(central if central.size else table_values))。
        table_z=float(np.median(central if central.size else table_values))
    # 【原第741行】计算/保存 data：{'base_xyz': product_xyz, 'confidence': float(conf), 'angle_valid': False, 'position_samples': len(pos), 'angle_samples': len(angles), 'product_z_rang…。
    data={"base_xyz":product_xyz,"confidence":float(conf),"angle_valid":False,
          # 【原第742行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
          "position_samples":len(pos),"angle_samples":len(angles),
          # 【原第743行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
          "product_z_range_m":product_z_range,
          # 【原第744行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
          "table_z_m":table_z,"table_samples":len(table_z_samples),
          # 【原第745行】继续上方跨行表达式的参数/数据项；所属模块：detect_product_batch。
          "rejects":rejects,"angle_rejects":angle_rejects}
    # 【原第746行】条件判断：len(angles) >= MIN_ANGLE_SAMPLES；满足时执行下面缩进代码，否则走else（若有）。
    if len(angles)>=MIN_ANGLE_SAMPLES:
        # 【原第747行】计算/保存 yaw：axis_average(angles)。 执行调用/更新：data.update({'angle_valid': True, 'yaw_rad': yaw, 'yaw_deg': math.degrees(yaw)})。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        yaw=axis_average(angles);data.update({"angle_valid":True,"yaw_rad":yaw,"yaw_deg":math.degrees(yaw)})
    # 【原第748行】返回结果给调用者：data。
    return data



# 【原第752行】计算两条长轴的180度等价角差，输出度数。
# ======================================================================================
# 【函数教学卡】_axis_diff_deg()
# 1. 功能：计算两条180°等价长轴的最小角差。
# 2. 输入：a、b弧度。
# 3. 输出：[-90°,90°]范围角差。
# 4. 谁调用：detect_product。
# 5. 结果流向：用于批次间yaw稳定性。
# 6. G2实际意义：不移动。
# 7. 调试重点：这不是完整四元数姿态误差，只是二维产品长轴差。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def _axis_diff_deg(a,b):
    # 【原第753行】计算/保存 d：math.degrees(a - b)。
    """【函数教学卡：超详细版】
    
    功能：
        计算两个产品长轴的180°等价最小角差。
    
    Args:
        a、b: 弧度。
    
    Returns:
        float：角差，单位度。
    
    调用时机：
        detect_product()调用。
    
    上下游逻辑：
        用于多批次Yaw稳定性判定。
    
    注意事项与调试重点：
        只表示二维长轴差，不是完整三维姿态差。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    d=math.degrees(a-b)
    # 【原第754行】重复执行下面代码，直到条件不成立：d > 90.0。 计算/保存 d：180.0。
    while d>90.0: d-=180.0
    # 【原第755行】重复执行下面代码，直到条件不成立：d < -90.0。 计算/保存 d：180.0。
    while d<-90.0: d+=180.0
    # 【原第756行】返回结果给调用者：d。
    return d


# 【原第759行】稳定门控：反复调用单批检测；多个批次的位置和角度稳定后才返回目标。
# ======================================================================================
# 【函数教学卡】detect_product()
# 1. 功能：方案C稳定门控。多次做完整批次检测，直到连续3个批次同时满足Z、XY和Yaw稳定。
# 2. 输入：model、camera、tf_api、report。
# 3. 输出：稳定后的final检测字典。
# 4. 谁调用：main每轮在WAYPOINT_2调用。
# 5. 结果流向：main锁定桌面高度并计算PreGrasp/Grasp。
# 6. G2实际意义：检测期间机械臂不动作；满足稳定后才返回给运动流程。
# 7. 调试重点：单批失败会清空累积；查看stable_batches、z_range、x/y std、yaw spread。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def detect_product(model,camera,tf_api,report):
    # 【原第760行】原有说明字符串，保留原文；历史描述不一定等于当前参数。
    """【函数教学卡：超详细版】
    
    功能：
        持续运行视觉批次，直到连续多个批次满足产品稳定条件。
    
    Args:
        model、camera、tf_api、report。
    
    Returns:
        Dict：稳定后的最终产品检测结果。
    
    调用时机：
        main每轮在WAYPOINT_2调用。
    
    上下游逻辑：
        结果用于桌面锁定、高度计算、姿态候选和运动规划。
    
    注意事项与调试重点：
        检测失败会清空连续批次；产品未稳定时机械臂不动作。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    """方案C：持续YOLO检测，直到多个完整批次均稳定后才返回。"""
    # 【原第761行】计算/保存 deadline：time.monotonic() + STABLE_DETECTION_TIMEOUT_S。
    deadline=time.monotonic()+STABLE_DETECTION_TIMEOUT_S
    # 【原第762行】计算/保存 stable_batches：[]。
    stable_batches=[]
    # 【原第763行】计算/保存 all_attempts：[]。
    all_attempts=[]
    # 【原第764行】重复执行下面代码，直到条件不成立：time.monotonic() < deadline。
    while time.monotonic()<deadline:
        # 【原第765行】开始异常处理块：try执行正常操作，except处理异常，finally执行清理（若有）。
        try:
            # 【原第766行】计算/保存 det：detect_product_batch(model, camera, tf_api, report)。
            det=detect_product_batch(model,camera,tf_api,report)
        # 【原第767行】异常分支：捕获对应异常，执行下面缩进代码；保持原处理方式。
        except Exception as exc:
            # 【原第768行】执行调用/更新：stable_batches.clear()。
            stable_batches.clear()
            # 【原第769行】计算/保存 UI_STATE['stable_batches']：0。
            UI_STATE["stable_batches"]=0
            # 【原第770行】执行调用/更新：refresh_full_yolo_window('WAIT PRODUCT', f'检测未满足条件: {exc}')。
            refresh_full_yolo_window("WAIT PRODUCT",f"检测未满足条件: {exc}")
            # 【原第771行】暂停当前线程：time.sleep(STABLE_RETRY_SLEEP_S)；此等待不代表已验证机器人到位。
            time.sleep(STABLE_RETRY_SLEEP_S)
            # 【原第772行】跳过本轮剩余代码，进入当前循环的下一轮。
            continue
        # 【原第773行】计算/保存 conf：float(det.get('confidence', 0.0))。
        conf=float(det.get("confidence",0.0))
        # 【原第774行】计算/保存 angle_valid：bool(det.get('angle_valid', False))。
        angle_valid=bool(det.get("angle_valid",False))
        # 【原第775行】计算/保存 xyz：[float(v) for v in det['base_xyz']]。
        xyz=[float(v) for v in det["base_xyz"]]
        # 【原第776行】计算/保存 zrange：float(det.get('product_z_range_m', 999.0))。
        zrange=float(det.get("product_z_range_m",999.0))
        # 【原第777行】计算/保存 yaw：float(det.get('yaw_rad', 0.0))。
        yaw=float(det.get("yaw_rad",0.0))
        # 【原第778行】计算/保存 table：det.get('table_z_m')。
        table=det.get("table_z_m")
        # 【原第779行】单批是否满足置信度、角度、桌面数据和Z极差要求。
        accepted=(conf>=STABLE_MIN_CONFIDENCE and angle_valid and table is not None and zrange<=MAX_STABLE_Z_RANGE_M)
        # 【原第780行】条件判断：not accepted；满足时执行下面缩进代码，否则走else（若有）。
        if not accepted:
            # 【原第781行】执行调用/更新：stable_batches.clear()。
            stable_batches.clear()
        # 【原第782行】否则分支：前面的条件不成立时执行这里。
        else:
            # 【原第783行】执行调用/更新：stable_batches.append({'det': det, 'xyz': xyz, 'yaw': yaw, 'table': float(table)})。
            stable_batches.append({"det":det,"xyz":xyz,"yaw":yaw,"table":float(table)})
            # 【原第784行】条件判断：len(stable_batches) > STABILITY_BATCHES_REQUIRED；满足时执行下面缩进代码，否则走else（若有）。
            if len(stable_batches)>STABILITY_BATCHES_REQUIRED:
                # 【原第785行】计算/保存 stable_batches：stable_batches[-STABILITY_BATCHES_REQUIRED:]。
                stable_batches=stable_batches[-STABILITY_BATCHES_REQUIRED:]
        # 【原第786行】计算/保存 xs：[s['xyz'][0] for s in stable_batches]。 计算/保存 ys：[s['xyz'][1] for s in stable_batches]。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        xs=[s["xyz"][0] for s in stable_batches]; ys=[s["xyz"][1] for s in stable_batches]
        # 【原第787行】计算/保存 xstd：statistics.pstdev(xs) if len(xs) > 1 else None。
        xstd=statistics.pstdev(xs) if len(xs)>1 else None
        # 【原第788行】计算/保存 ystd：statistics.pstdev(ys) if len(ys) > 1 else None。
        ystd=statistics.pstdev(ys) if len(ys)>1 else None
        # 【原第789行】计算/保存 yawspread：0.0。
        yawspread=0.0
        # 【原第790行】条件判断：len(stable_batches) > 1；满足时执行下面缩进代码，否则走else（若有）。
        if len(stable_batches)>1:
            # 【原第791行】计算/保存 ref：stable_batches[0]['yaw']。
            ref=stable_batches[0]["yaw"]
            # 【原第792行】计算/保存 yawspread：max((abs(_axis_diff_deg(s['yaw'], ref)) for s in stable_batches))。
            yawspread=max(abs(_axis_diff_deg(s["yaw"],ref)) for s in stable_batches)
        # 【原第793行】多个有效批次是否同时满足XY标准差与长轴差要求。
        spatial_ok=(len(stable_batches)>=STABILITY_BATCHES_REQUIRED and xstd is not None and ystd is not None and xstd<=MAX_STABLE_XY_STD_M and ystd<=MAX_STABLE_XY_STD_M and yawspread<=MAX_STABLE_YAW_SPREAD_DEG)
        # 【原第794行】执行调用/更新：UI_STATE.update({'confidence': conf, 'position_samples': det.get('position_samples', 0), 'angle_samples': det.get('angle_samples', 0), 'stable_batches': len(stable_batches), 'z_ran。
        UI_STATE.update({
            # 【原第795行】继续上方跨行表达式的参数/数据项；所属模块：detect_product。
            "confidence":conf,"position_samples":det.get("position_samples",0),
            # 【原第796行】继续上方跨行表达式的参数/数据项；所属模块：detect_product。
            "angle_samples":det.get("angle_samples",0),"stable_batches":len(stable_batches),
            # 【原第797行】继续上方跨行表达式的参数/数据项；所属模块：detect_product。
            "z_range_m":zrange,"x_std_m":xstd,"y_std_m":ystd,
            # 【原第798行】继续上方跨行表达式的参数/数据项；所属模块：detect_product。
            "yaw_spread_deg":yawspread,"product_xyz":xyz,"yaw_deg":math.degrees(yaw),
        # 【原第799行】继续上方跨行表达式的参数/数据项；所属模块：detect_product。
        })
        # 【原第800行】执行调用/更新：all_attempts.append({'confidence': conf, 'angle_valid': angle_valid, 'xyz': xyz, 'z_range_m': zrange, 'stable_batches': len(stable_batches)})。
        all_attempts.append({"confidence":conf,"angle_valid":angle_valid,"xyz":xyz,"z_range_m":zrange,"stable_batches":len(stable_batches)})
        # 【原第801行】计算/保存 report['stability_gate_v90']：{'attempts': all_attempts[-30:], 'required_batches': STABILITY_BATCHES_REQUIRED}。
        report["stability_gate_v90"]={"attempts":all_attempts[-30:],"required_batches":STABILITY_BATCHES_REQUIRED}
        # 【原第802行】调用save保存当前报告；便于定位本轮检测或运动问题。
        save(report)
        # 【原第803行】执行调用/更新：refresh_full_yolo_window('WAIT PRODUCT STABLE', f'稳定批次 {len(stable_batches)}/{STABILITY_BATCHES_REQUIRED}')。
        refresh_full_yolo_window("WAIT PRODUCT STABLE",f"稳定批次 {len(stable_batches)}/{STABILITY_BATCHES_REQUIRED}")
        # 【原第804行】条件判断：spatial_ok；满足时执行下面缩进代码，否则走else（若有）。
        if spatial_ok:
            # 【原第805行】计算/保存 final：copy.deepcopy(stable_batches[-1]['det'])。
            final=copy.deepcopy(stable_batches[-1]["det"])
            # 【原第806行】计算/保存 final['base_xyz']：[statistics.median(xs), statistics.median(ys), statistics.median([s['xyz'][2] for s in stable_batches])]。
            final["base_xyz"]=[statistics.median(xs),statistics.median(ys),statistics.median([s["xyz"][2] for s in stable_batches])]
            # 【原第807行】计算/保存 final['yaw_rad']：axis_average([s['yaw'] for s in stable_batches])。
            final["yaw_rad"]=axis_average([s["yaw"] for s in stable_batches])
            # 【原第808行】计算/保存 final['yaw_deg']：math.degrees(final['yaw_rad'])。
            final["yaw_deg"]=math.degrees(final["yaw_rad"])
            # 【原第809行】计算/保存 final['table_z_m']：statistics.median([s['table'] for s in stable_batches])。
            final["table_z_m"]=statistics.median([s["table"] for s in stable_batches])
            # 【原第810行】计算/保存 final['stability_batches']：len(stable_batches)。
            final["stability_batches"]=len(stable_batches)
            # 【原第811行】计算/保存 final['stability_x_std_m']：xstd。
            final["stability_x_std_m"]=xstd
            # 【原第812行】计算/保存 final['stability_y_std_m']：ystd。
            final["stability_y_std_m"]=ystd
            # 【原第813行】计算/保存 final['stability_yaw_spread_deg']：yawspread。
            final["stability_yaw_spread_deg"]=yawspread
            # 【原第814行】执行调用/更新：refresh_full_yolo_window('PRODUCT STABLE LOCKED', '产品稳定，开始抓取')。
            refresh_full_yolo_window("PRODUCT STABLE LOCKED","产品稳定，开始抓取")
            # 【原第815行】返回结果给调用者：final。
            return final
        # 【原第816行】暂停当前线程：time.sleep(STABLE_RETRY_SLEEP_S)；此等待不代表已验证机器人到位。
        time.sleep(STABLE_RETRY_SLEEP_S)
    # 【原第817行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
    raise RuntimeError(f"等待产品稳定超时: {STABLE_DETECTION_TIMEOUT_S:.1f}s")


# 【原第820行】组装双臂末端控制请求；左臂使用保持位姿，右臂使用新目标。
# ======================================================================================
# 【函数教学卡】set_pose()
# 1. 功能：把左臂保持位姿和右臂目标位姿装入GDK双臂末端请求并发送。
# 2. 输入：robot、left PoseData、right PoseData。
# 3. 输出：正常无return；接口非0抛异常。
# 4. 谁调用：笛卡尔路径、固定锚点姿态、旧Yaw函数。
# 5. 结果流向：机器人执行短生命周期末端目标。
# 6. G2实际意义：会控制双臂组。虽然左臂目标是保持值，仍属于双臂请求。
# 7. 调试重点：位置单位米、四元数顺序xyzw、life_time过期会影响连续控制。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def set_pose(robot,left,right):
    # 【原第821行】计算/保存 req：agibot_gdk.EndEffectorPose()。 计算/保存 req.life_time：LIFE_TIME。
    """【函数教学卡：超详细版】
    
    功能：
        组装双臂末端控制请求并发送。
    
    Args:
        robot、left PoseData、right PoseData。
    
    Returns:
        None；接口失败抛异常。
    
    调用时机：
        笛卡尔平移、固定锚点姿态和旧Yaw函数调用。
    
    上下游逻辑：
        短生命周期目标发送到G2控制器。
    
    注意事项与调试重点：
        会真实控制双臂组；位置单位米，四元数顺序xyzw。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    req=agibot_gdk.EndEffectorPose();req.life_time=LIFE_TIME
    # 【原第822行】计算/保存 req.group：agibot_gdk.EndEffectorControlGroup.kBothArms。
    req.group=agibot_gdk.EndEffectorControlGroup.kBothArms
    # 【原第823行】计算/保存 (lp, lq)：(req.left_end_effector_pose.position, req.left_end_effector_pose.orientation)。
    lp,lq=req.left_end_effector_pose.position,req.left_end_effector_pose.orientation
    # 【原第824行】计算/保存 (rp, rq)：(req.right_end_effector_pose.position, req.right_end_effector_pose.orientation)。
    rp,rq=req.right_end_effector_pose.position,req.right_end_effector_pose.orientation
    # 【原第825行】计算/保存 (lp.x, lp.y, lp.z)：left.position。 计算/保存 (lq.x, lq.y, lq.z, lq.w)：left.orientation。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    lp.x,lp.y,lp.z=left.position;lq.x,lq.y,lq.z,lq.w=left.orientation
    # 【原第826行】计算/保存 (rp.x, rp.y, rp.z)：right.position。 计算/保存 (rq.x, rq.y, rq.z, rq.w)：right.orientation。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    rp.x,rp.y,rp.z=right.position;rq.x,rq.y,rq.z,rq.w=right.orientation
    # 【原第827行】条件判断：robot.end_effector_pose_control(req) != 0；满足时执行下面缩进代码，否则走else（若有）。 抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
    if robot.end_effector_pose_control(req)!=0: raise RuntimeError("末端控制失败")


# 【原第830行】根据TCP目标和姿态反算腕部End目标：TCP减去旋转后的工具偏移。 返回结果给调用者：(np.asarray(tcp) - q_to_r(q).dot(TCP_OFFSET_END_M)).tolist()。
# ======================================================================================
# 【函数教学卡】end_from_tcp()
# 1. 功能：根据期望TCP和姿态，反算GDK需要发送的arm_r_end_link位置。
# 2. 输入：tcp XYZ和四元数q。
# 3. 输出：End位置列表。
# 4. 谁调用：固定锚点姿态和旧Yaw旋转。
# 5. 结果流向：与姿态一起构成PoseData给set_pose。
# 6. G2实际意义：数学转换。
# 7. 调试重点：TCP_OFFSET_END_M错误会导致“看似姿态正确但TCP漂移”。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def end_from_tcp(tcp,q): return (np.asarray(tcp)-q_to_r(q).dot(TCP_OFFSET_END_M)).tolist()


# 【原第833行】生成密集位置插值，逐点检查力矩、发送末端命令并刷新界面。
# ======================================================================================
# 【函数教学卡】command_cart_path()
# 1. 功能：把直线平移拆成很多小位置点并连续发送。
# 2. 输入：起点End、终点End、固定姿态、步长等。
# 3. 输出：无return。
# 4. 谁调用：move_tcp_closed_loop。
# 5. 结果流向：发送后由闭环函数读取实际TCP并纠偏。
# 6. G2实际意义：会连续控制末端。
# 7. 调试重点：CART_STEP_M越小越密集更慢；每个点调用力矩检查和窗口刷新。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def command_cart_path(robot,left,start_end,target_end,q,base,report,label,step=CART_STEP_M):
    # 【原第834行】计算/保存 length：dist(start_end, target_end)。 计算/保存 n：max(2, int(math.ceil(length / step)))。
    """【函数教学卡：超详细版】
    
    功能：
        把右臂直线平移拆成密集小步并连续发送。
    
    Args:
        起点End、终点End、固定姿态、步长、baseline等。
    
    Returns:
        None。
    
    调用时机：
        move_tcp_closed_loop()调用。
    
    上下游逻辑：
        发送后由闭环函数读取实际TCP并验证。
    
    注意事项与调试重点：
        会真实移动；步长越小越慢且命令越密。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    length=dist(start_end,target_end);n=max(2,int(math.ceil(length/step)))
    # 【原第835行】循环：依次从 range(1, n + 1) 取值到 i，执行下面缩进块。
    for i in range(1,n+1):
        # 【原第836行】执行调用/更新：verify_torque(robot, base, report)。 计算/保存 p：lerp(start_end, target_end, i / n)。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        verify_torque(robot,base,report);p=lerp(start_end,target_end,i/n)
        # 【原第837行】发送末端目标位姿；该调用会控制机器人，后续仍需实际TF验证。 执行调用/更新：refresh_full_yolo_window(label, '运动执行中')。 暂停当前线程：time.sleep(DT)；此等待不代表已验证机器人到位。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        set_pose(robot,left,PoseData(p,q.copy()));refresh_full_yolo_window(label,"运动执行中");time.sleep(DT)


# 【原第840行】执行TCP增量移动，读取实际位置；在硬限制内最多补偿两次残差。
# ======================================================================================
# 【函数教学卡】move_tcp_closed_loop()
# 1. 功能：执行一段TCP平移，并根据实际TF最多进行两次剩余误差修正。
# 2. 输入：delta为相对移动XYZ、q为保持姿态。
# 3. 输出：最终实际TCP和End。
# 4. 谁调用：粗对准、精修、PreGrasp下降、Grasp下降、Lift。
# 5. 结果流向：结果用于后续运动或残差检查。
# 6. G2实际意义：会控制右臂末端。
# 7. 调试重点：首先区分目标误差te和姿态误差qe；超过50mm立即停止追赶。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def move_tcp_closed_loop(robot,tf_api,left,delta,q,base,report,label):
    # 【原第841行】条件判断：REQUIRE_CONFIRMATION；满足时执行下面缩进代码，否则走else（若有）。
    """【函数教学卡：超详细版】
    
    功能：
        执行TCP增量平移，并根据实际误差最多自动修正两次。
    
    Args:
        robot、tf_api、left、delta、q、baseline、report、label等。
    
    Returns:
        Tuple[PoseData,PoseData]：最终实际TCP和End。
    
    调用时机：
        粗XY、精XY、PreGrasp、Grasp、Lift调用。
    
    上下游逻辑：
        结果交给下一运动阶段或状态记录。
    
    注意事项与调试重点：
        TCP误差超过50mm停止，不继续追赶。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    if REQUIRE_CONFIRMATION:
        # 【原第842行】条件判断：input(f'准备执行{label}，delta={[round(v, 5) for v in delta]}，输入 NEXT：').strip() != 'NEXT'；满足时执行下面缩进代码，否则走else（若有）。
        if input(f"准备执行{label}，delta={[round(v,5) for v in delta]}，输入 NEXT：").strip()!="NEXT":
            # 【原第843行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
            raise RuntimeError(f"用户取消{label}")
    # 【原第844行】计算/保存 start_tcp：wait_pose(tf_api, TCP_FRAME)。 计算/保存 start_end：wait_pose(tf_api, RIGHT_FRAME)。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    start_tcp=wait_pose(tf_api,TCP_FRAME);start_end=wait_pose(tf_api,RIGHT_FRAME)
    # 【原第845行】计算/保存 target_tcp：[start_tcp.position[i] + delta[i] for i in range(3)]。
    target_tcp=[start_tcp.position[i]+delta[i] for i in range(3)]
    # 【原第846行】条件判断：target_tcp[2] < MIN_TCP_Z_M；满足时执行下面缩进代码，否则走else（若有）。 抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
    if target_tcp[2]<MIN_TCP_Z_M: raise RuntimeError(f"{label}目标过低")
    # 【原第847行】计算/保存 target_end：[start_end.position[i] + delta[i] for i in range(3)]。
    target_end=[start_end.position[i]+delta[i] for i in range(3)]
    # 【原第848行】执行调用/更新：command_cart_path(robot, left, start_end.position, target_end, q, base, report, label)。
    command_cart_path(robot,left,start_end.position,target_end,q,base,report,label)
    # 【原第849行】计算/保存 attempts：[]。
    attempts=[]
    # 【原第850行】循环：依次从 range(MAX_CORRECTION_ATTEMPTS + 1) 取值到 attempt，执行下面缩进块。
    for attempt in range(MAX_CORRECTION_ATTEMPTS+1):
        # 【原第851行】暂停当前线程：time.sleep(SETTLE_S)；此等待不代表已验证机器人到位。 计算/保存 actual_tcp：wait_pose(tf_api, TCP_FRAME)。 计算/保存 actual_end：wait_pose(tf_api, RIGHT_FRAME)。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        time.sleep(SETTLE_S);actual_tcp=wait_pose(tf_api,TCP_FRAME);actual_end=wait_pose(tf_api,RIGHT_FRAME)
        # 【原第852行】计算/保存 te：dist(actual_tcp.position, target_tcp)。 计算/保存 qe：q_error_deg(actual_end.orientation, q)。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        te=dist(actual_tcp.position,target_tcp);qe=q_error_deg(actual_end.orientation,q)
        # 【原第853行】执行调用/更新：attempts.append({'attempt': attempt, 'tcp_error': te, 'orientation_error': qe, 'actual_tcp': actual_tcp.__dict__})。
        attempts.append({"attempt":attempt,"tcp_error":te,"orientation_error":qe,
                         # 【原第854行】继续上方跨行表达式的参数/数据项；所属模块：move_tcp_closed_loop。
                         "actual_tcp":actual_tcp.__dict__})
        # 【原第855行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print(f"[{label}闭环{attempt}] TCP误差={te:.4f}m, 姿态误差={qe:.2f}deg")
        # 【原第856行】条件判断：te <= TCP_TOLERANCE_M and qe <= ORIENTATION_TOLERANCE_DEG；满足时执行下面缩进代码，否则走else（若有）。 退出当前最近的一层循环，继续执行循环后面的代码。
        if te<=TCP_TOLERANCE_M and qe<=ORIENTATION_TOLERANCE_DEG: break
        # 【原第857行】条件判断：te > HARD_TCP_ERROR_LIMIT_M；满足时执行下面缩进代码，否则走else（若有）。
        if te>HARD_TCP_ERROR_LIMIT_M:
            # 【原第858行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
            raise RuntimeError(f"{label} TCP误差{te:.4f}m超过50mm硬限制")
        # 【原第859行】条件判断：attempt >= MAX_CORRECTION_ATTEMPTS；满足时执行下面缩进代码，否则走else（若有）。
        if attempt>=MAX_CORRECTION_ATTEMPTS:
            # 【原第860行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
            raise RuntimeError(f"{label}闭环修正后仍未到位")
        # 【原第861行】计算/保存 residual：[target_tcp[i] - actual_tcp.position[i] for i in range(3)]。
        residual=[target_tcp[i]-actual_tcp.position[i] for i in range(3)]
        # 【原第862行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print(f"[{label}] 自动修正{attempt+1}: {[round(v,5) for v in residual]}")
        # 【原第863行】计算/保存 corr_target：[actual_end.position[i] + residual[i] for i in range(3)]。
        corr_target=[actual_end.position[i]+residual[i] for i in range(3)]
        # 【原第864行】执行调用/更新：command_cart_path(robot, left, actual_end.position, corr_target, q, base, report, label, CORRECTION_STEP_M)。
        command_cart_path(robot,left,actual_end.position,corr_target,q,base,report,label,CORRECTION_STEP_M)
    # 【原第865行】执行调用/更新：report['cartesian_stages'].append({'stage': label, 'target_tcp': target_tcp, 'attempts': attempts})。 调用save保存当前报告；便于定位本轮检测或运动问题。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    report["cartesian_stages"].append({"stage":label,"target_tcp":target_tcp,"attempts":attempts});save(report)
    # 【原第866行】返回结果给调用者：(wait_pose(tf_api, TCP_FRAME), wait_pose(tf_api, RIGHT_FRAME))。
    return wait_pose(tf_api,TCP_FRAME),wait_pose(tf_api,RIGHT_FRAME)


# 【原第869行】取夹爪局部X或Y轴在Base XY平面的投影方向。
# ======================================================================================
# 【函数教学卡】gripper_yaw()
# 1. 功能：从夹爪姿态矩阵中取局部长轴在Base XY平面的方向角。
# 2. 输入：四元数。
# 3. 输出：yaw弧度。
# 4. 谁调用：当前主流程未直接依赖，属于辅助函数。
# 5. 结果流向：可用于比较产品长轴与夹爪长轴。
# 6. G2实际意义：不移动。
# 7. 调试重点：由GRIPPER_LONG_AXIS_LOCAL决定取矩阵X列还是Y列。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def gripper_yaw(q):
    # 【原第870行】计算/保存 axis：q_to_r(q)[:, 1 if GRIPPER_LONG_AXIS_LOCAL == 'y' else 0]。
    """【函数教学卡：超详细版】
    
    功能：
        计算夹爪局部长轴在Base XY平面的Yaw。
    
    Args:
        q: 四元数。
    
    Returns:
        float：Yaw弧度。
    
    调用时机：
        当前主流程未直接调用，属于保留辅助函数。
    
    上下游逻辑：
        可用于产品长轴与夹爪方向比较。
    
    注意事项与调试重点：
        GRIPPER_LONG_AXIS_LOCAL决定使用局部X还是Y轴。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    axis=q_to_r(q)[:,1 if GRIPPER_LONG_AXIS_LOCAL=="y" else 0]
    # 【原第871行】返回结果给调用者：math.atan2(float(axis[1]), float(axis[0]))。
    return math.atan2(float(axis[1]),float(axis[0]))


# 【原第874行】按矩阵迹和对角元素选择分支，将旋转矩阵转换为四元数。
# ======================================================================================
# 【函数教学卡】rotation_to_q()
# 1. 功能：把3×3旋转矩阵转换成四元数。多个分支用于不同数值情形下保持稳定。
# 2. 输入：有效旋转矩阵r。
# 3. 输出：归一化四元数[x,y,z,w]。
# 4. 谁调用：vertical_tool_orientation。
# 5. 结果流向：目标姿态送给等价方向选择和姿态对齐。
# 6. G2实际意义：纯数学。
# 7. 调试重点：不要随意删分支；矩阵必须正交且描述合法旋转。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def rotation_to_q(r):
    # 【原第875行】计算/保存 trace：float(np.trace(r))。
    """【函数教学卡：超详细版】
    
    功能：
        把3×3旋转矩阵转换成四元数。
    
    Args:
        r: 合法旋转矩阵。
    
    Returns:
        List[float]：[x,y,z,w]。
    
    调用时机：
        vertical_tool_orientation()调用。
    
    上下游逻辑：
        输出成为夹爪目标姿态。
    
    注意事项与调试重点：
        多个数学分支用于数值稳定，不要随意删除。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    trace=float(np.trace(r))
    # 【原第876行】条件判断：trace > 0.0；满足时执行下面缩进代码，否则走else（若有）。
    if trace>0.0:
        # 【原第877行】计算/保存 ss：math.sqrt(trace + 1.0) * 2.0。 计算/保存 w：0.25 * ss。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        ss=math.sqrt(trace+1.0)*2.0; w=0.25*ss
        # 【原第878行】计算/保存 x：(r[2, 1] - r[1, 2]) / ss。 计算/保存 y：(r[0, 2] - r[2, 0]) / ss。 计算/保存 z：(r[1, 0] - r[0, 1]) / ss。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        x=(r[2,1]-r[1,2])/ss; y=(r[0,2]-r[2,0])/ss; z=(r[1,0]-r[0,1])/ss
    # 【原第879行】条件判断：r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]；满足时执行下面缩进代码，否则走else（若有）。
    elif r[0,0]>r[1,1] and r[0,0]>r[2,2]:
        # 【原第880行】计算/保存 ss：math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0。
        ss=math.sqrt(1.0+r[0,0]-r[1,1]-r[2,2])*2.0
        # 【原第881行】计算/保存 w：(r[2, 1] - r[1, 2]) / ss。 计算/保存 x：0.25 * ss。 计算/保存 y：(r[0, 1] + r[1, 0]) / ss。 计算/保存 z：(r[0, 2] + r[2, 0]) / ss。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        w=(r[2,1]-r[1,2])/ss; x=0.25*ss; y=(r[0,1]+r[1,0])/ss; z=(r[0,2]+r[2,0])/ss
    # 【原第882行】条件判断：r[1, 1] > r[2, 2]；满足时执行下面缩进代码，否则走else（若有）。
    elif r[1,1]>r[2,2]:
        # 【原第883行】计算/保存 ss：math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0。
        ss=math.sqrt(1.0+r[1,1]-r[0,0]-r[2,2])*2.0
        # 【原第884行】计算/保存 w：(r[0, 2] - r[2, 0]) / ss。 计算/保存 x：(r[0, 1] + r[1, 0]) / ss。 计算/保存 y：0.25 * ss。 计算/保存 z：(r[1, 2] + r[2, 1]) / ss。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        w=(r[0,2]-r[2,0])/ss; x=(r[0,1]+r[1,0])/ss; y=0.25*ss; z=(r[1,2]+r[2,1])/ss
    # 【原第885行】否则分支：前面的条件不成立时执行这里。
    else:
        # 【原第886行】计算/保存 ss：math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0。
        ss=math.sqrt(1.0+r[2,2]-r[0,0]-r[1,1])*2.0
        # 【原第887行】计算/保存 w：(r[1, 0] - r[0, 1]) / ss。 计算/保存 x：(r[0, 2] + r[2, 0]) / ss。 计算/保存 y：(r[1, 2] + r[2, 1]) / ss。 计算/保存 z：0.25 * ss。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        w=(r[1,0]-r[0,1])/ss; x=(r[0,2]+r[2,0])/ss; y=(r[1,2]+r[2,1])/ss; z=0.25*ss
    # 【原第888行】返回结果给调用者：norm_q([x, y, z, w])。
    return norm_q([x,y,z,w])


# 【原第891行】构造局部Z向下、局部Y沿产品长轴的目标姿态；依赖当前工具轴约定。
# ======================================================================================
# 【函数教学卡】vertical_tool_orientation()
# 1. 功能：根据产品长轴构造“夹爪局部Z向下、局部Y沿产品”的完整三维目标姿态。
# 2. 输入：product_yaw弧度。
# 3. 输出：目标四元数。
# 4. 谁调用：main生成yaw/yaw±180°候选。
# 5. 结果流向：选择姿态变化最小候选后交给align_orientation_target。
# 6. G2实际意义：不直接移动。
# 7. 调试重点：工具轴定义必须与真实TCP标定一致；否则垂直方向会错。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def vertical_tool_orientation(product_yaw):
    # local +Z从腕部指向TCP，因此令local +Z垂直向下。
    # 【原第893行】计算/保存 local_y：np.array([math.cos(product_yaw), math.sin(product_yaw), 0.0], dtype=float)。
    """【函数教学卡：超详细版】
    
    功能：
        构造夹爪局部Z向下、局部Y沿产品长轴的完整目标姿态。
    
    Args:
        product_yaw: 产品长轴角，弧度。
    
    Returns:
        List[float]：目标四元数。
    
    调用时机：
        main生成yaw和yaw±180°候选。
    
    上下游逻辑：
        选最短候选后交给align_orientation_target。
    
    注意事项与调试重点：
        工具局部轴定义必须与真实夹爪标定一致。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    local_y=np.array([math.cos(product_yaw),math.sin(product_yaw),0.0],dtype=float)
    # 【原第894行】计算/保存 local_z：np.array([0.0, 0.0, -1.0], dtype=float)。
    local_z=np.array([0.0,0.0,-1.0],dtype=float)
    # 【原第895行】计算/保存 local_x：np.cross(local_y, local_z)。 计算/保存 local_x：np.linalg.norm(local_x)。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    local_x=np.cross(local_y,local_z); local_x/=np.linalg.norm(local_x)
    # 【原第896行】计算/保存 local_y：np.cross(local_z, local_x)。 计算/保存 local_y：np.linalg.norm(local_y)。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    local_y=np.cross(local_z,local_x); local_y/=np.linalg.norm(local_y)
    # 【原第897行】返回结果给调用者：rotation_to_q(np.column_stack((local_x, local_y, local_z)))。
    return rotation_to_q(np.column_stack((local_x,local_y,local_z)))


# 【原第900行】计算工具局部Z与Base向下方向的夹角，不是直接测量桌面法线。
# ======================================================================================
# 【函数教学卡】tool_vertical_error_deg()
# 1. 功能：计算夹爪局部Z轴与Base竖直向下[0,0,-1]之间的夹角。
# 2. 输入：四元数。
# 3. 输出：垂直误差deg，0°表示完全向下。
# 4. 谁调用：align_orientation_target。
# 5. 结果流向：每段日志、最佳阶段和最终容差。
# 6. G2实际意义：不移动。
# 7. 调试重点：它不包含产品长轴Yaw误差，只衡量工具Z是否向下。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def tool_vertical_error_deg(q):
    # 【原第901行】计算/保存 tool_z：q_to_r(q)[:, 2]。
    """【函数教学卡：超详细版】
    
    功能：
        计算工具局部Z轴和Base向下方向的夹角。
    
    Args:
        q: 四元数。
    
    Returns:
        float：垂直误差，单位度。
    
    调用时机：
        align_orientation_target()调用。
    
    上下游逻辑：
        用于逐段日志和最终垂直容差。
    
    注意事项与调试重点：
        0°表示工具Z完全向下，不包含产品Yaw误差。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    tool_z=q_to_r(q)[:,2]
    # 【原第902行】计算/保存 dot：float(np.dot(tool_z, np.array([0.0, 0.0, -1.0])))。
    dot=float(np.dot(tool_z,np.array([0.0,0.0,-1.0])))
    # 【原第903行】返回结果给调用者：math.degrees(math.acos(max(-1.0, min(1.0, dot))))。
    return math.degrees(math.acos(max(-1.0,min(1.0,dot))))


# 【原第906行】围绕同一个TCP锚点分段调整姿态，验证累计漂移、跟踪误差和最终垂直度。
# ======================================================================================
# 【函数教学卡】align_orientation_target()
# 1. 功能：在固定同一个TCP锚点的前提下，把右臂姿态分段旋转到目标并逐段验证。
# 2. 输入：机器人、TF、左臂保持位姿、目标四元数、力矩基线、报告。
# 3. 输出：最终实际TCP和End。
# 4. 谁调用：main在XY粗对准后。
# 5. 结果流向：main做姿态后XY精修，再下降PreGrasp。
# 6. G2实际意义：会连续控制末端姿态。
# 7. 调试重点：关键日志是累计TCP漂移、姿态误差、垂直误差；硬限制与连续软超限都会停止。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def align_orientation_target(robot,tf_api,left,target_q,base,report):
    # 【原第907行】原有说明字符串，保留原文；历史描述不一定等于当前参数。
    """【函数教学卡：超详细版】
    
    功能：
        围绕固定TCP锚点分段旋转夹爪到目标姿态。
    
    Args:
        robot、tf_api、left、target_q、baseline、report。
    
    Returns:
        Tuple：最终实际TCP和End。
    
    调用时机：
        main完成XY粗对准后调用。
    
    上下游逻辑：
        完成后main做XY精修并下降PreGrasp。
    
    注意事项与调试重点：
        会真实移动；重点监控累计TCP漂移、姿态误差、垂直误差。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    """绕同一个固定TCP锚点分段调整姿态，禁止累计漂移。"""
    # 【原第908行】计算/保存 initial_end：wait_pose(tf_api, RIGHT_FRAME)。
    initial_end=wait_pose(tf_api,RIGHT_FRAME)
    # 【原第909行】整个姿态调整开始时固定的TCP锚点，不随每段实际漂移更新。
    anchor_tcp=wait_pose(tf_api,TCP_FRAME)
    # 【原第910行】计算/保存 initial_q：initial_end.orientation.copy()。
    initial_q=initial_end.orientation.copy()
    # 【原第911行】计算/保存 total：q_error_deg(initial_q, target_q)。
    total=q_error_deg(initial_q,target_q)
    # 【原第912行】计算/保存 stages：max(1, int(math.ceil(total / MAX_YAW_PER_STAGE_DEG)))。
    stages=max(1,int(math.ceil(total/MAX_YAW_PER_STAGE_DEG)))

    # 【原第914行】条件判断：REQUIRE_CONFIRMATION；满足时执行下面缩进代码，否则走else（若有）。
    if REQUIRE_CONFIRMATION:
        # 【原第915行】计算/保存 command：input(f'准备固定TCP锚点垂直姿态对齐{total:.1f}deg/{stages}段，输入 NEXT：').strip()。
        command=input(
            # 【原第916行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            f"准备固定TCP锚点垂直姿态对齐{total:.1f}deg/{stages}段，输入 NEXT："
        # 【原第917行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
        ).strip()
        # 【原第918行】条件判断：command != 'NEXT'；满足时执行下面缩进代码，否则走else（若有）。
        if command!="NEXT":
            # 【原第919行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
            raise RuntimeError("用户取消固定锚点垂直姿态对齐")

    # 【原第921行】计算/保存 records：[]。
    records=[]
    # 【原第922行】计算/保存 consecutive_soft_failures：0。
    consecutive_soft_failures=0
    # 【原第923行】计算/保存 best_vertical_error：float('inf')。
    best_vertical_error=float("inf")
    # 【原第924行】计算/保存 best_stage：0。
    best_stage=0

    # 【原第926行】循环：依次从 range(1, stages + 1) 取值到 si，执行下面缩进块。
    for si in range(1,stages+1):
        # 【原第927行】计算/保存 stage_target：slerp(initial_q, target_q, si / stages)。
        stage_target=slerp(initial_q,target_q,si/stages)
        # 【原第928行】计算/保存 actual_end_before：wait_pose(tf_api, RIGHT_FRAME)。
        actual_end_before=wait_pose(tf_api,RIGHT_FRAME)
        # 【原第929行】计算/保存 orientation_change：q_error_deg(actual_end_before.orientation, stage_target)。
        orientation_change=q_error_deg(actual_end_before.orientation,stage_target)
        # 【原第930行】计算/保存 n：max(2, int(math.ceil(orientation_change / ROTATION_STEP_DEG)))。
        n=max(2,int(math.ceil(orientation_change/ROTATION_STEP_DEG)))

        # 【原第932行】循环：依次从 range(1, n + 1) 取值到 i，执行下面缩进块。
        for i in range(1,n+1):
            # 【原第933行】计算/保存 qq：slerp(actual_end_before.orientation, stage_target, i / n)。
            qq=slerp(actual_end_before.orientation,stage_target,i/n)
            # 关键：每个插值点都使用最初的固定anchor_tcp，而不是本段实际TCP。
            # 【原第935行】计算/保存 pp：end_from_tcp(anchor_tcp.position, qq)。
            pp=end_from_tcp(anchor_tcp.position,qq)
            # 【原第936行】发送末端目标位姿；该调用会控制机器人，后续仍需实际TF验证。
            set_pose(robot,left,PoseData(pp,qq))
            # 【原第937行】执行调用/更新：refresh_full_yolo_window('VERTICAL ORIENTATION', f'固定锚点姿态 {si}/{stages}')。
            refresh_full_yolo_window("VERTICAL ORIENTATION",f"固定锚点姿态 {si}/{stages}")
            # 【原第938行】暂停当前线程：time.sleep(DT)；此等待不代表已验证机器人到位。
            time.sleep(DT)

        # 【原第940行】计算/保存 hold：max(1, int(ROTATION_FINAL_HOLD_S * RATE_HZ))。
        hold=max(1,int(ROTATION_FINAL_HOLD_S*RATE_HZ))
        # 【原第941行】计算/保存 final_end_position：end_from_tcp(anchor_tcp.position, stage_target)。
        final_end_position=end_from_tcp(anchor_tcp.position,stage_target)
        # 【原第942行】循环：依次从 range(hold) 取值到 _，执行下面缩进块。
        for _ in range(hold):
            # 【原第943行】发送末端目标位姿；该调用会控制机器人，后续仍需实际TF验证。
            set_pose(robot,left,PoseData(final_end_position,stage_target))
            # 【原第944行】暂停当前线程：time.sleep(DT)；此等待不代表已验证机器人到位。
            time.sleep(DT)

        # 【原第946行】暂停当前线程：time.sleep(VERTICAL_STAGE_SETTLE_S)；此等待不代表已验证机器人到位。
        time.sleep(VERTICAL_STAGE_SETTLE_S)
        # 【原第947行】计算/保存 actual_tcp：wait_pose(tf_api, TCP_FRAME)。
        actual_tcp=wait_pose(tf_api,TCP_FRAME)
        # 【原第948行】计算/保存 actual_end：wait_pose(tf_api, RIGHT_FRAME)。
        actual_end=wait_pose(tf_api,RIGHT_FRAME)
        # 【原第949行】计算/保存 cumulative_drift：dist(actual_tcp.position, anchor_tcp.position)。
        cumulative_drift=dist(actual_tcp.position,anchor_tcp.position)
        # 【原第950行】计算/保存 orientation_error：q_error_deg(actual_end.orientation, stage_target)。
        orientation_error=q_error_deg(actual_end.orientation,stage_target)
        # 【原第951行】计算/保存 vertical_error：tool_vertical_error_deg(actual_end.orientation)。
        vertical_error=tool_vertical_error_deg(actual_end.orientation)

        # 【原第953行】条件判断：vertical_error < best_vertical_error；满足时执行下面缩进代码，否则走else（若有）。
        if vertical_error<best_vertical_error:
            # 【原第954行】计算/保存 best_vertical_error：vertical_error。
            best_vertical_error=vertical_error
            # 【原第955行】计算/保存 best_stage：si。
            best_stage=si

        # 【原第957行】计算/保存 soft_failed：cumulative_drift > VERTICAL_FIXED_ANCHOR_SOFT_DRIFT_M or orientation_error > VERTICAL_STAGE_ORIENTATION_TOLERANCE_DEG。
        soft_failed=(
            # 【原第958行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            cumulative_drift>VERTICAL_FIXED_ANCHOR_SOFT_DRIFT_M
            # 【原第959行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            or orientation_error>VERTICAL_STAGE_ORIENTATION_TOLERANCE_DEG
        # 【原第960行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
        )
        # 【原第961行】计算/保存 hard_failed：cumulative_drift > VERTICAL_FIXED_ANCHOR_HARD_DRIFT_M or orientation_error > VERTICAL_HARD_ORIENTATION_ERROR_DEG。
        hard_failed=(
            # 【原第962行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            cumulative_drift>VERTICAL_FIXED_ANCHOR_HARD_DRIFT_M
            # 【原第963行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            or orientation_error>VERTICAL_HARD_ORIENTATION_ERROR_DEG
        # 【原第964行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
        )
        # 【原第965行】执行调用/更新：records.append({'stage': si, 'stage_count': stages, 'anchor_tcp': anchor_tcp.__dict__, 'actual_tcp': actual_tcp.__dict__, 'cumulative_anchor_drift_m': cumulative_drift, 'orientatio。
        records.append({
            # 【原第966行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            "stage":si,
            # 【原第967行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            "stage_count":stages,
            # 【原第968行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            "anchor_tcp":anchor_tcp.__dict__,
            # 【原第969行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            "actual_tcp":actual_tcp.__dict__,
            # 【原第970行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            "cumulative_anchor_drift_m":cumulative_drift,
            # 【原第971行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            "orientation_error_deg":orientation_error,
            # 【原第972行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            "vertical_error_deg":vertical_error,
            # 【原第973行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            "soft_failed":soft_failed,
            # 【原第974行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            "hard_failed":hard_failed,
        # 【原第975行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
        })
        # 【原第976行】计算/保存 report['vertical_orientation_stages']：records。
        report["vertical_orientation_stages"]=records
        # 【原第977行】调用save保存当前报告；便于定位本轮检测或运动问题。
        save(report)

        # 【原第979行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print(
            # 【原第980行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            f"[固定锚点垂直姿态 {si}/{stages}] "
            # 【原第981行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            f"累计TCP漂移={cumulative_drift:.4f}m, "
            # 【原第982行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            f"姿态误差={orientation_error:.2f}deg, "
            # 【原第983行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            f"垂直误差={vertical_error:.2f}deg"
        # 【原第984行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
        )

        # 【原第986行】条件判断：hard_failed；满足时执行下面缩进代码，否则走else（若有）。
        if hard_failed:
            # 【原第987行】计算/保存 report['vertical_failure']：{'type': 'fixed_anchor_hard_limit', 'stage': si, 'best_stage': best_stage, 'best_vertical_error_deg': best_vertical_error}。
            report["vertical_failure"]={
                # 【原第988行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                "type":"fixed_anchor_hard_limit",
                # 【原第989行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                "stage":si,
                # 【原第990行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                "best_stage":best_stage,
                # 【原第991行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                "best_vertical_error_deg":best_vertical_error,
            # 【原第992行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            }
            # 【原第993行】调用save保存当前报告；便于定位本轮检测或运动问题。
            save(report)
            # 【原第994行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
            raise RuntimeError(
                # 【原第995行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                f"固定锚点垂直姿态第{si}段达到硬限制: "
                # 【原第996行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                f"累计TCP漂移={cumulative_drift:.4f}m, "
                # 【原第997行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                f"姿态误差={orientation_error:.2f}deg"
            # 【原第998行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )

        # 【原第1000行】条件判断：soft_failed；满足时执行下面缩进代码，否则走else（若有）。
        if soft_failed:
            # 【原第1001行】计算/保存 consecutive_soft_failures：1。
            consecutive_soft_failures+=1
            # 【原第1002行】打印当前进度或诊断信息到控制台；不发送机器人命令。
            print(
                # 【原第1003行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                f"[固定锚点姿态警告] 轻微超限 "
                # 【原第1004行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                f"{consecutive_soft_failures}/"
                # 【原第1005行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                f"{VERTICAL_MAX_CONSECUTIVE_SOFT_FAILURES}"
            # 【原第1006行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )
        # 【原第1007行】否则分支：前面的条件不成立时执行这里。
        else:
            # 【原第1008行】计算/保存 consecutive_soft_failures：0。
            consecutive_soft_failures=0

        # 【原第1010行】条件判断：consecutive_soft_failures >= VERTICAL_MAX_CONSECUTIVE_SOFT_FAILURES；满足时执行下面缩进代码，否则走else（若有）。
        if consecutive_soft_failures>=VERTICAL_MAX_CONSECUTIVE_SOFT_FAILURES:
            # 【原第1011行】计算/保存 report['vertical_failure']：{'type': 'fixed_anchor_consecutive_soft_limit', 'stage': si, 'best_stage': best_stage, 'best_vertical_error_deg': best_vertical_error}。
            report["vertical_failure"]={
                # 【原第1012行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                "type":"fixed_anchor_consecutive_soft_limit",
                # 【原第1013行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                "stage":si,
                # 【原第1014行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                "best_stage":best_stage,
                # 【原第1015行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                "best_vertical_error_deg":best_vertical_error,
            # 【原第1016行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            }
            # 【原第1017行】调用save保存当前报告；便于定位本轮检测或运动问题。
            save(report)
            # 【原第1018行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
            raise RuntimeError(
                # 【原第1019行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                "固定锚点垂直姿态连续两段轻微超限，"
                # 【原第1020行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
                "当前构型接近可达边界，禁止继续"
            # 【原第1021行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )

    # 【原第1023行】计算/保存 final_tcp：wait_pose(tf_api, TCP_FRAME)。
    final_tcp=wait_pose(tf_api,TCP_FRAME)
    # 【原第1024行】计算/保存 final_end：wait_pose(tf_api, RIGHT_FRAME)。
    final_end=wait_pose(tf_api,RIGHT_FRAME)
    # 【原第1025行】计算/保存 final_anchor_drift：dist(final_tcp.position, anchor_tcp.position)。
    final_anchor_drift=dist(final_tcp.position,anchor_tcp.position)
    # 【原第1026行】计算/保存 final_vertical_error：tool_vertical_error_deg(final_end.orientation)。
    final_vertical_error=tool_vertical_error_deg(final_end.orientation)
    # 【原第1027行】计算/保存 report['vertical_orientation_summary']：{'anchor_tcp': anchor_tcp.__dict__, 'final_tcp': final_tcp.__dict__, 'final_anchor_drift_m': final_anchor_drift, 'final_vertical_error_deg': final_ver…。
    report["vertical_orientation_summary"]={
        # 【原第1028行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
        "anchor_tcp":anchor_tcp.__dict__,
        # 【原第1029行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
        "final_tcp":final_tcp.__dict__,
        # 【原第1030行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
        "final_anchor_drift_m":final_anchor_drift,
        # 【原第1031行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
        "final_vertical_error_deg":final_vertical_error,
        # 【原第1032行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
        "best_stage":best_stage,
        # 【原第1033行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
        "best_vertical_error_deg":best_vertical_error,
    # 【原第1034行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
    }
    # 【原第1035行】调用save保存当前报告；便于定位本轮检测或运动问题。
    save(report)

    # 【原第1037行】条件判断：final_anchor_drift > VERTICAL_FIXED_ANCHOR_HARD_DRIFT_M；满足时执行下面缩进代码，否则走else（若有）。
    if final_anchor_drift>VERTICAL_FIXED_ANCHOR_HARD_DRIFT_M:
        # 【原第1038行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
        raise RuntimeError(
            # 【原第1039行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            f"垂直姿态最终累计TCP漂移{final_anchor_drift:.4f}m超过50mm"
        # 【原第1040行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
        )
    # 【原第1041行】条件判断：final_vertical_error > VERTICAL_TOOL_TILT_TOLERANCE_DEG；满足时执行下面缩进代码，否则走else（若有）。
    if final_vertical_error>VERTICAL_TOOL_TILT_TOLERANCE_DEG:
        # 【原第1042行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
        raise RuntimeError(
            # 【原第1043行】继续上方跨行表达式的参数/数据项；所属模块：align_orientation_target。
            f"夹爪末端未达到垂直容差: {final_vertical_error:.2f}deg"
        # 【原第1044行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
        )
    # 【原第1045行】返回结果给调用者：(final_tcp, final_end)。
    return final_tcp,final_end


# 【原第1048行】保留的旧Yaw旋转函数；当前main主要调用完整垂直姿态函数。
# ======================================================================================
# 【函数教学卡】rotate_yaw()
# 1. 功能：旧版仅绕Yaw调整的保留函数。
# 2. 输入：delta弧度等。
# 3. 输出：实际TCP和End。
# 4. 谁调用：当前main没有调用。
# 5. 结果流向：无当前主流程下游。
# 6. G2实际意义：若被调用会移动。
# 7. 调试重点：不要把“函数存在”误认为“当前流程执行”；应从main调用链判断。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def rotate_yaw(robot,tf_api,left,delta,base,report):
    # 【原第1049行】计算/保存 stages：max(1, int(math.ceil(abs(math.degrees(delta)) / MAX_YAW_PER_STAGE_DEG)))。
    """【函数教学卡：超详细版】
    
    功能：
        旧版仅调整Yaw的保留函数。
    
    Args:
        delta弧度及运动对象。
    
    Returns:
        Tuple：实际TCP与End。
    
    调用时机：
        当前main未调用。
    
    上下游逻辑：
        当前主流程无下游。
    
    注意事项与调试重点：
        函数存在不代表运行；应从main调用链判断。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    stages=max(1,int(math.ceil(abs(math.degrees(delta))/MAX_YAW_PER_STAGE_DEG)))
    # 【原第1050行】条件判断：REQUIRE_CONFIRMATION and input(f'准备角度对齐{math.degrees(delta):.1f}deg/{stages}段，输入 NEXT：').strip() != 'NEXT'；满足时执行下面缩进代码，否则走else（若有）。
    if REQUIRE_CONFIRMATION and input(f"准备角度对齐{math.degrees(delta):.1f}deg/{stages}段，输入 NEXT：").strip()!="NEXT":
        # 【原第1051行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
        raise RuntimeError("用户取消角度对齐")
    # 【原第1052行】计算/保存 initial_end：wait_pose(tf_api, RIGHT_FRAME)。 计算/保存 initial_q：initial_end.orientation.copy()。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    initial_end=wait_pose(tf_api,RIGHT_FRAME);initial_q=initial_end.orientation.copy()
    # 【原第1053行】计算/保存 records：[]。
    records=[]
    # 【原第1054行】循环：依次从 range(1, stages + 1) 取值到 si，执行下面缩进块。
    for si in range(1,stages+1):
        # 【原第1055行】计算/保存 target_q：q_mul(q_yaw(delta * si / stages), initial_q)。
        target_q=q_mul(q_yaw(delta*si/stages),initial_q)
        # 【原第1056行】计算/保存 start_end：wait_pose(tf_api, RIGHT_FRAME)。 计算/保存 start_tcp：wait_pose(tf_api, TCP_FRAME)。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        start_end=wait_pose(tf_api,RIGHT_FRAME);start_tcp=wait_pose(tf_api,TCP_FRAME)
        # 【原第1057行】计算/保存 n：max(2, int(math.ceil(q_error_deg(start_end.orientation, target_q) / ROTATION_STEP_DEG)))。
        n=max(2,int(math.ceil(q_error_deg(start_end.orientation,target_q)/ROTATION_STEP_DEG)))
        # 【原第1058行】循环：依次从 range(1, n + 1) 取值到 i，执行下面缩进块。
        for i in range(1,n+1):
            # 【原第1059行】计算/保存 q：slerp(start_end.orientation, target_q, i / n)。 计算/保存 p：end_from_tcp(start_tcp.position, q)。
            # 【本行拆解教学：分号语句】
            # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
            # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
            # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
            q=slerp(start_end.orientation,target_q,i/n);p=end_from_tcp(start_tcp.position,q)
            # 【原第1060行】发送末端目标位姿；该调用会控制机器人，后续仍需实际TF验证。 暂停当前线程：time.sleep(DT)；此等待不代表已验证机器人到位。
            # 【本行拆解教学：分号语句】
            # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
            # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
            # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
            set_pose(robot,left,PoseData(p,q));time.sleep(DT)
        # 【原第1061行】计算/保存 hold：max(1, int(ROTATION_FINAL_HOLD_S * RATE_HZ))。 计算/保存 p：end_from_tcp(start_tcp.position, target_q)。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        hold=max(1,int(ROTATION_FINAL_HOLD_S*RATE_HZ));p=end_from_tcp(start_tcp.position,target_q)
        # 【原第1062行】循环：依次从 range(hold) 取值到 _，执行下面缩进块。 发送末端目标位姿；该调用会控制机器人，后续仍需实际TF验证。 暂停当前线程：time.sleep(DT)；此等待不代表已验证机器人到位。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        for _ in range(hold): set_pose(robot,left,PoseData(p,target_q));time.sleep(DT)
        # 【原第1063行】暂停当前线程：time.sleep(SETTLE_S)；此等待不代表已验证机器人到位。 计算/保存 at：wait_pose(tf_api, TCP_FRAME)。 计算/保存 ae：wait_pose(tf_api, RIGHT_FRAME)。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        time.sleep(SETTLE_S);at=wait_pose(tf_api,TCP_FRAME);ae=wait_pose(tf_api,RIGHT_FRAME)
        # 【原第1064行】计算/保存 drift：dist(at.position, start_tcp.position)。 计算/保存 qe：q_error_deg(ae.orientation, target_q)。
        # 【本行拆解教学：分号语句】
        # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
        # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
        # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
        drift=dist(at.position,start_tcp.position);qe=q_error_deg(ae.orientation,target_q)
        # 【原第1065行】执行调用/更新：records.append({'stage': si, 'drift': drift, 'orientation_error': qe})。
        records.append({"stage":si,"drift":drift,"orientation_error":qe})
        # 【原第1066行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print(f"[角度对齐 {si}/{stages}] TCP漂移={drift:.4f}m, 姿态误差={qe:.2f}deg")
        # 【原第1067行】条件判断：drift > MAX_ROTATION_STAGE_DRIFT_M or qe > ORIENTATION_TOLERANCE_DEG；满足时执行下面缩进代码，否则走else（若有）。
        if drift>MAX_ROTATION_STAGE_DRIFT_M or qe>ORIENTATION_TOLERANCE_DEG:
            # 【原第1068行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
            raise RuntimeError(f"角度对齐第{si}段超限")
    # 【原第1069行】计算/保存 report['angle_stages']：records。 调用save保存当前报告；便于定位本轮检测或运动问题。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    report["angle_stages"]=records;save(report)
    # 【原第1070行】返回结果给调用者：(wait_pose(tf_api, TCP_FRAME), wait_pose(tf_api, RIGHT_FRAME))。
    return wait_pose(tf_api,TCP_FRAME),wait_pose(tf_api,RIGHT_FRAME)


# 【原第1073行】腰部按短目标到绝对角度；不发双臂命令，但逐段检查双臂关节变化。
# ======================================================================================
# 【函数教学卡】move_waist_segmented_keep_arms()
# 1. 功能：把腰部idx05从当前绝对角度分段移动到目标，同时验证左右臂关节基本不变。
# 2. 输入：目标腰部5关节、左右臂参考值、label等。
# 3. 输出：最终腰部关节和各段记录。
# 4. 谁调用：turn_release_and_return_waist正转和反转各调用一次。
# 5. 结果流向：到放置位后松爪，返回后main回WP2/WP1。
# 6. G2实际意义：会控制腰部。
# 7. 调试重点：每段检查idx05角度误差以及左右臂最大变化；单次80°曾超时，所以保留分段。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def move_waist_segmented_keep_arms(
    # 【原第1074行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
    robot, report, target_waist, label,
    # 【原第1075行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
    left_reference, right_reference,
# 【原第1076行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
):
    # 【原第1077行】原有说明字符串，保留原文；历史描述不一定等于当前参数。
    """【函数教学卡：超详细版】
    
    功能：
        分段控制腰部idx05到目标角，并验证双臂关节保持。
    
    Args:
        robot、target_waist、左右臂参考、report、label。
    
    Returns:
        Tuple：实际腰部和分段记录。
    
    调用时机：
        放置正转与返回反转各调用一次。
    
    上下游逻辑：
        到放置位后松爪，返回后main回WP2/WP1。
    
    注意事项与调试重点：
        会真实控制腰部；80°通常拆为两段。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    """分段移动腰部到绝对目标；整个过程不发送双臂命令，并验证双臂保持。"""
    # 【原第1078行】计算/保存 waist_before：get_positions(robot, WAIST_JOINTS)。
    waist_before=get_positions(robot,WAIST_JOINTS)
    # 【原第1079行】循环：依次从 enumerate(zip(target_waist, WAIST_LIMITS_RAD)) 取值到 (index, (value, limits))，执行下面缩进块。
    for index,(value,limits) in enumerate(zip(target_waist,WAIST_LIMITS_RAD)):
        # 【原第1080行】条件判断：not limits[0] <= value <= limits[1]；满足时执行下面缩进代码，否则走else（若有）。
        if not limits[0] <= value <= limits[1]:
            # 【原第1081行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
            raise RuntimeError(
                # 【原第1082行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
                f"{label}目标越界: {WAIST_JOINTS[index]}="
                # 【原第1083行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
                f"{math.degrees(value):.2f}deg, 允许范围="
                # 【原第1084行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
                f"[{math.degrees(limits[0]):.2f}, {math.degrees(limits[1]):.2f}]deg"
            # 【原第1085行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )

    # 【原第1087行】计算/保存 turn_total_rad：target_waist[WAIST_YAW_JOINT_INDEX] - waist_before[WAIST_YAW_JOINT_INDEX]。
    turn_total_rad=(
        # 【原第1088行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
        target_waist[WAIST_YAW_JOINT_INDEX]
        # 【原第1089行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
        - waist_before[WAIST_YAW_JOINT_INDEX]
    # 【原第1090行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
    )
    # 【原第1091行】计算/保存 segment_count：max(1, int(math.ceil(abs(math.degrees(turn_total_rad)) / WAIST_MAX_STEP_DEG)))。
    segment_count=max(1,int(math.ceil(
        # 【原第1092行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
        abs(math.degrees(turn_total_rad))/WAIST_MAX_STEP_DEG
    # 【原第1093行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
    )))
    # 【原第1094行】计算/保存 records：[]。
    records=[]
    # 【原第1095行】打印当前进度或诊断信息到控制台；不发送机器人命令。
    print("-"*78)
    # 【原第1096行】打印当前进度或诊断信息到控制台；不发送机器人命令。
    print(
        # 【原第1097行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
        f"[{label}] idx05: "
        # 【原第1098行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
        f"{math.degrees(waist_before[WAIST_YAW_JOINT_INDEX]):.2f}deg -> "
        # 【原第1099行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
        f"{math.degrees(target_waist[WAIST_YAW_JOINT_INDEX]):.2f}deg, "
        # 【原第1100行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
        f"共{segment_count}段"
    # 【原第1101行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
    )

    # 【原第1103行】循环：依次从 range(1, segment_count + 1) 取值到 segment_index，执行下面缩进块。
    for segment_index in range(1,segment_count+1):
        # 【原第1104行】计算/保存 alpha：segment_index / segment_count。
        alpha=segment_index/segment_count
        # 【原第1105行】计算/保存 segment_target：waist_before.copy()。
        segment_target=waist_before.copy()
        # 【原第1106行】计算/保存 segment_target[WAIST_YAW_JOINT_INDEX]：waist_before[WAIST_YAW_JOINT_INDEX] + turn_total_rad * alpha。
        segment_target[WAIST_YAW_JOINT_INDEX]=(
            # 【原第1107行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            waist_before[WAIST_YAW_JOINT_INDEX]
            # 【原第1108行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            + turn_total_rad*alpha
        # 【原第1109行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
        )
        # 【原第1110行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print(
            # 【原第1111行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            f"[{label} {segment_index}/{segment_count}] "
            # 【原第1112行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            f"idx05目标={math.degrees(segment_target[WAIST_YAW_JOINT_INDEX]):.2f}deg"
        # 【原第1113行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
        )
        # 【原第1114行】执行调用/更新：refresh_full_yolo_window(label, f'腰部段 {segment_index}/{segment_count}')。
        refresh_full_yolo_window(label,f"腰部段 {segment_index}/{segment_count}")
        # 【原第1115行】开始异常处理块：try执行正常操作，except处理异常，finally执行清理（若有）。
        try:
            # 【原第1116行】计算/保存 result：robot.move_waist_joint(segment_target, [WAIST_SPEED_RAD_S] * 5)。
            result=robot.move_waist_joint(
                # 【原第1117行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
                segment_target,
                # 【原第1118行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
                [WAIST_SPEED_RAD_S]*5,
            # 【原第1119行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )
        # 【原第1120行】异常分支：捕获对应异常，执行下面缩进代码；保持原处理方式。
        except Exception as exc:
            # 【原第1121行】计算/保存 actual：get_positions(robot, WAIST_JOINTS)。
            actual=get_positions(robot,WAIST_JOINTS)
            # 【原第1122行】执行调用/更新：report.setdefault('waist_failures', []).append({'label': label, 'segment_index': segment_index, 'segment_count': segment_count, 'target_rad': segment_target, 'actual_rad': actual, 。
            report.setdefault("waist_failures",[]).append({
                # 【原第1123行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
                "label":label,
                # 【原第1124行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
                "segment_index":segment_index,
                # 【原第1125行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
                "segment_count":segment_count,
                # 【原第1126行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
                "target_rad":segment_target,
                # 【原第1127行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
                "actual_rad":actual,
                # 【原第1128行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
                "exception":str(exc),
            # 【原第1129行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            })
            # 【原第1130行】调用save保存当前报告；便于定位本轮检测或运动问题。
            save(report)
            # 【原第1131行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
            raise RuntimeError(
                # 【原第1132行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
                f"{label}第{segment_index}/{segment_count}段失败: {exc}"
            # 【原第1133行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            ) from exc
        # 【原第1134行】条件判断：result != 0；满足时执行下面缩进代码，否则走else（若有）。
        if result != 0:
            # 【原第1135行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
            raise RuntimeError(
                # 【原第1136行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
                f"{label}第{segment_index}/{segment_count}段返回失败: {result}"
            # 【原第1137行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )

        # 【原第1139行】暂停当前线程：time.sleep(WAIST_SEGMENT_SETTLE_S)；此等待不代表已验证机器人到位。
        time.sleep(WAIST_SEGMENT_SETTLE_S)
        # 【原第1140行】计算/保存 waist_actual：get_positions(robot, WAIST_JOINTS)。
        waist_actual=get_positions(robot,WAIST_JOINTS)
        # 【原第1141行】计算/保存 left_actual：get_positions(robot, LEFT_JOINTS)。
        left_actual=get_positions(robot,LEFT_JOINTS)
        # 【原第1142行】计算/保存 right_actual：get_positions(robot, RIGHT_JOINTS)。
        right_actual=get_positions(robot,RIGHT_JOINTS)
        # 【原第1143行】计算/保存 segment_error_deg：abs(math.degrees(waist_actual[WAIST_YAW_JOINT_INDEX] - segment_target[WAIST_YAW_JOINT_INDEX]))。
        segment_error_deg=abs(math.degrees(
            # 【原第1144行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            waist_actual[WAIST_YAW_JOINT_INDEX]
            # 【原第1145行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            - segment_target[WAIST_YAW_JOINT_INDEX]
        # 【原第1146行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
        ))
        # 【原第1147行】计算/保存 left_error：max_joint_error(left_actual, left_reference)。
        left_error=max_joint_error(left_actual,left_reference)
        # 【原第1148行】计算/保存 right_error：max_joint_error(right_actual, right_reference)。
        right_error=max_joint_error(right_actual,right_reference)
        # 【原第1149行】计算/保存 record：{'label': label, 'segment_index': segment_index, 'segment_count': segment_count, 'target_rad': segment_target, 'actual_rad': waist_actual, 'segment_er…。
        record={
            # 【原第1150行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            "label":label,
            # 【原第1151行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            "segment_index":segment_index,
            # 【原第1152行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            "segment_count":segment_count,
            # 【原第1153行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            "target_rad":segment_target,
            # 【原第1154行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            "actual_rad":waist_actual,
            # 【原第1155行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            "segment_error_deg":segment_error_deg,
            # 【原第1156行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            "left_arm_max_change_rad":left_error,
            # 【原第1157行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            "right_arm_max_change_rad":right_error,
        # 【原第1158行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
        }
        # 【原第1159行】执行调用/更新：records.append(record)。
        records.append(record)
        # 【原第1160行】执行调用/更新：report.setdefault('waist_motion_segments', []).append(record)。
        report.setdefault("waist_motion_segments",[]).append(record)
        # 【原第1161行】调用save保存当前报告；便于定位本轮检测或运动问题。
        save(report)
        # 【原第1162行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print(
            # 【原第1163行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            f"[{label} {segment_index}/{segment_count}验证] "
            # 【原第1164行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            f"误差={segment_error_deg:.2f}deg, "
            # 【原第1165行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            f"左臂变化={left_error:.5f}rad, 右臂变化={right_error:.5f}rad"
        # 【原第1166行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
        )
        # 【原第1167行】条件判断：segment_error_deg > WAIST_POSITION_TOLERANCE_DEG；满足时执行下面缩进代码，否则走else（若有）。
        if segment_error_deg > WAIST_POSITION_TOLERANCE_DEG:
            # 【原第1168行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
            raise RuntimeError(f"{label}第{segment_index}段未到位")
        # 【原第1169行】条件判断：left_error > ARM_HOLD_TOLERANCE_RAD or right_error > ARM_HOLD_TOLERANCE_RAD；满足时执行下面缩进代码，否则走else（若有）。
        if (
            # 【原第1170行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            left_error > ARM_HOLD_TOLERANCE_RAD
            # 【原第1171行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
            or right_error > ARM_HOLD_TOLERANCE_RAD
        # 【原第1172行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
        ):
            # 【原第1173行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
            raise RuntimeError(f"{label}第{segment_index}段双臂变化超限")

    # 【原第1175行】暂停当前线程：time.sleep(WAIST_SETTLE_S)；此等待不代表已验证机器人到位。
    time.sleep(WAIST_SETTLE_S)
    # 【原第1176行】计算/保存 waist_after：get_positions(robot, WAIST_JOINTS)。
    waist_after=get_positions(robot,WAIST_JOINTS)
    # 【原第1177行】计算/保存 final_error_deg：abs(math.degrees(waist_after[WAIST_YAW_JOINT_INDEX] - target_waist[WAIST_YAW_JOINT_INDEX]))。
    final_error_deg=abs(math.degrees(
        # 【原第1178行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
        waist_after[WAIST_YAW_JOINT_INDEX]
        # 【原第1179行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
        - target_waist[WAIST_YAW_JOINT_INDEX]
    # 【原第1180行】继续上方跨行表达式的参数/数据项；所属模块：move_waist_segmented_keep_arms。
    ))
    # 【原第1181行】条件判断：final_error_deg > WAIST_POSITION_TOLERANCE_DEG；满足时执行下面缩进代码，否则走else（若有）。
    if final_error_deg > WAIST_POSITION_TOLERANCE_DEG:
        # 【原第1182行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
        raise RuntimeError(f"{label}最终未到位: {final_error_deg:.2f}deg")
    # 【原第1183行】返回结果给调用者：(waist_after, records)。
    return waist_after, records


# 【原第1186行】记录原腰部角度→右转→释放→反转回原角度；随后main负责右臂回Waypoint。
# ======================================================================================
# 【函数教学卡】turn_release_and_return_waist()
# 1. 功能：完成抓取后的放置动作：记录原腰部→右转80°→松爪→反转回原腰部。
# 2. 输入：robot、report。
# 3. 输出：原腰部关节列表。
# 4. 谁调用：main完成Lift后。
# 5. 结果流向：main再让右臂回WP2和WP1。
# 6. G2实际意义：控制腰部和夹爪，不发送新的手臂目标。
# 7. 调试重点：“手臂不动”由关节变化验证，不代表空间TCP绝对不动，因为腰部旋转会带动整条运动链。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def turn_release_and_return_waist(robot,report):
    # 【原第1187行】原有说明字符串，保留原文；历史描述不一定等于当前参数。
    """【函数教学卡：超详细版】
    
    功能：
        抓取后腰部右转80°、松开产品，再反转回原始腰部角。
    
    Args:
        robot、report。
    
    Returns:
        List[float]：原始腰部关节角。
    
    调用时机：
        main完成抬升后调用。
    
    上下游逻辑：
        完成后main让右臂回WAYPOINT_2和WAYPOINT_1。
    
    注意事项与调试重点：
        控制腰部和夹爪；手臂关节保持不等于空间TCP不动。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    """右转80度释放产品；双臂不动；再反转80度回到本轮转身前腰部姿态。"""
    # 【原第1188行】计算/保存 waist_origin：get_positions(robot, WAIST_JOINTS)。
    waist_origin=get_positions(robot,WAIST_JOINTS)
    # 【原第1189行】计算/保存 left_reference：get_positions(robot, LEFT_JOINTS)。
    left_reference=get_positions(robot,LEFT_JOINTS)
    # 【原第1190行】计算/保存 right_reference：get_positions(robot, RIGHT_JOINTS)。
    right_reference=get_positions(robot,RIGHT_JOINTS)

    # 【原第1192行】计算/保存 waist_drop：waist_origin.copy()。
    waist_drop=waist_origin.copy()
    # 【原第1193行】计算/保存 waist_drop[WAIST_YAW_JOINT_INDEX]：math.radians(WAIST_TURN_DEG)。
    waist_drop[WAIST_YAW_JOINT_INDEX] += math.radians(WAIST_TURN_DEG)
    # 【原第1194行】执行调用/更新：move_waist_segmented_keep_arms(robot, report, waist_drop, '腰部右转放置', left_reference, right_reference)。
    move_waist_segmented_keep_arms(
        # 【原第1195行】继续上方跨行表达式的参数/数据项；所属模块：turn_release_and_return_waist。
        robot,report,waist_drop,"腰部右转放置",
        # 【原第1196行】继续上方跨行表达式的参数/数据项；所属模块：turn_release_and_return_waist。
        left_reference,right_reference,
    # 【原第1197行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
    )

    # 【原第1199行】打印当前进度或诊断信息到控制台；不发送机器人命令。
    print("松开右夹爪，释放产品...")
    # 【原第1200行】调用夹爪控制函数；参数沿用原程序，不更改开合逻辑。
    set_gripper(robot,-0.785)
    # 【原第1201行】计算/保存 release_time：time.time()。
    release_time=time.time()
    # 【原第1202行】执行调用/更新：report.setdefault('release_events', []).append({'time_epoch_s': release_time, 'waist_rad': get_positions(robot, WAIST_JOINTS), 'right_arm_rad': get_positions(robot, RIGHT_JOINTS)})。
    report.setdefault("release_events",[]).append({
        # 【原第1203行】继续上方跨行表达式的参数/数据项；所属模块：turn_release_and_return_waist。
        "time_epoch_s":release_time,
        # 【原第1204行】继续上方跨行表达式的参数/数据项；所属模块：turn_release_and_return_waist。
        "waist_rad":get_positions(robot,WAIST_JOINTS),
        # 【原第1205行】继续上方跨行表达式的参数/数据项；所属模块：turn_release_and_return_waist。
        "right_arm_rad":get_positions(robot,RIGHT_JOINTS),
    # 【原第1206行】继续上方跨行表达式的参数/数据项；所属模块：turn_release_and_return_waist。
    })
    # 【原第1207行】调用save保存当前报告；便于定位本轮检测或运动问题。
    save(report)

    # 释放后双臂仍保持；腰部按原路径反向回到本轮起始姿态。
    # 【原第1210行】执行调用/更新：move_waist_segmented_keep_arms(robot, report, waist_origin, '腰部反转回抓取方向', left_reference, right_reference)。
    move_waist_segmented_keep_arms(
        # 【原第1211行】继续上方跨行表达式的参数/数据项；所属模块：turn_release_and_return_waist。
        robot,report,waist_origin,"腰部反转回抓取方向",
        # 【原第1212行】继续上方跨行表达式的参数/数据项；所属模块：turn_release_and_return_waist。
        left_reference,right_reference,
    # 【原第1213行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
    )
    # 【原第1214行】打印当前进度或诊断信息到控制台；不发送机器人命令。
    print("腰部已反转回本轮起始角度，双臂保持验证通过。")
    # 【原第1215行】返回结果给调用者：waist_origin。
    return waist_origin


# 【原第1218行】组装右工具关节请求；当前流程用-0.785打开、0.0闭合，再等待0.7秒。
# ======================================================================================
# 【函数教学卡】set_gripper()
# 1. 功能：发送右夹爪位置命令。
# 2. 输入：value，当前程序-0.785用于打开、0.0用于闭合。
# 3. 输出：无return；失败抛异常。
# 4. 谁调用：抓取前打开、抓取点闭合、放置处打开。
# 5. 结果流向：动作后固定等待0.7秒。
# 6. G2实际意义：真实控制夹爪。
# 7. 调试重点：代码没有独立读取夹持成功信号，因此“闭合完成”不等于一定抓住产品。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def set_gripper(robot,value):
    # 【原第1219行】计算/保存 req：agibot_gdk.JointStates()。 计算/保存 req.group：'right_tool'。 计算/保存 req.target_type：'omnipicker'。
    """【函数教学卡：超详细版】
    
    功能：
        发送右夹爪位置命令。
    
    Args:
        robot；value为夹爪目标值。
    
    Returns:
        None。
    
    调用时机：
        抓取前打开、抓取点闭合、放置处打开。
    
    上下游逻辑：
        动作后固定等待0.7秒。
    
    注意事项与调试重点：
        程序未独立检测是否真正夹住产品。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    req=agibot_gdk.JointStates();req.group="right_tool";req.target_type="omnipicker"
    # 【原第1220行】计算/保存 j：agibot_gdk.JointState()。 计算/保存 j.position：value。 计算/保存 req.states：[j]。 计算/保存 req.nums：1。
    # 【本行拆解教学：分号语句】
    # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
    # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
    # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
    j=agibot_gdk.JointState();j.position=value;req.states=[j];req.nums=1
    # 【原第1221行】条件判断：robot.move_ee_pos(req) != 0；满足时执行下面缩进代码，否则走else（若有）。 抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
    if robot.move_ee_pos(req)!=0: raise RuntimeError("右夹爪控制失败")
    # 【原第1222行】暂停当前线程：time.sleep(0.7)；此等待不代表已验证机器人到位。
    time.sleep(0.7)


# 【原第1225行】总调度：初始化→启动检查→Waypoint→稳定检测→高度/姿态规划→抓取→腰部释放返回→循环→资源清理。
# ======================================================================================
# 【函数教学卡】main()
# 1. 功能：90版总调度器，决定所有对象初始化、启动路径、循环、视觉、抓取、放置、回位和资源释放。
# 2. 输入：无命令行参数，主要读取全局参数。
# 3. 输出：通常无return；状态写入报告。
# 4. 谁调用：文件末尾if __name__=="__main__"时调用。
# 5. 结果流向：main内部调用所有核心函数。
# 6. G2实际意义：真实运动总入口。
# 7. 调试重点：先从main阅读，遇到函数名再跳到定义；异常时finally只释放资源，不会自动回HOME。
# 8. Python基础：def表示定义函数；括号内是参数；缩进区域是函数体；定义时不会立刻执行。
# ======================================================================================
def main():
    # 【原第1226行】计算/保存 report：{'program': '90_g2_pick_wait_until_product_stable_full_yolo.py', 'cycle_count': CYCLE_COUNT, 'enable_real_motion': ENABLE_REAL_MOTION, 'enable_angle_a…。
    """【函数教学卡：超详细版】
    
    功能：
        程序总入口，串联初始化、视觉、抓取、放置、回位和多循环。
    
    Args:
        无参数；读取大量全局配置。
    
    Returns:
        None；运行状态写入JSON报告。
    
    调用时机：
        文件末尾__main__判断调用。
    
    上下游逻辑：
        内部调用全部核心函数。
    
    注意事项与调试重点：
        真实运动入口；异常finally只清理资源，不自动回HOME。
    
    Python初学者提示：
        def只是在这里定义函数；只有其他代码执行“函数名(...)”时，函数体才真正运行。
        Args是输入，Returns是输出；None表示函数不向调用者返回业务结果。
    """
    report={
        # 【原第1227行】继续上方跨行表达式的参数/数据项；所属模块：main。
        "program":"90_g2_pick_wait_until_product_stable_full_yolo.py",
        # 【原第1228行】继续上方跨行表达式的参数/数据项；所属模块：main。
        "cycle_count":CYCLE_COUNT,
        # 【原第1229行】继续上方跨行表达式的参数/数据项；所属模块：main。
        "enable_real_motion":ENABLE_REAL_MOTION,
        # 【原第1230行】继续上方跨行表达式的参数/数据项；所属模块：main。
        "enable_angle_alignment":ENABLE_ANGLE_ALIGNMENT,
        # 【原第1231行】继续上方跨行表达式的参数/数据项；所属模块：main。
        "enable_xy_and_pregrasp":ENABLE_XY_AND_PREGRASP,
        # 【原第1232行】继续上方跨行表达式的参数/数据项；所属模块：main。
        "enable_gripper_and_pick":ENABLE_GRIPPER_AND_PICK,
        # 【原第1233行】继续上方跨行表达式的参数/数据项；所属模块：main。
        "qt_font":QT_FONT_RESULT,
        # 【原第1234行】继续上方跨行表达式的参数/数据项；所属模块：main。
        "cycles": (),
        # 【原第1235行】继续上方跨行表达式的参数/数据项；所属模块：main。
        "joint_stages":[],
        # 【原第1236行】继续上方跨行表达式的参数/数据项；所属模块：main。
        "cartesian_stages":[],
        # 【原第1237行】继续上方跨行表达式的参数/数据项；所属模块：main。
        "status":"INITIALIZED",
    # 【原第1238行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
    }
    # 【原第1239行】计算/保存 initialized：False。
    initialized=False
    # 【原第1240行】计算/保存 camera：None。
    camera=None
    # 【原第1241行】开始异常处理块：try执行正常操作，except处理异常，finally执行清理（若有）。
    try:
        # 【原第1242行】条件判断：not Path(MODEL_PATH).exists()；满足时执行下面缩进代码，否则走else（若有）。
        if not Path(MODEL_PATH).exists():
            # 【原第1243行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
            raise RuntimeError(f"模型不存在: {MODEL_PATH}")
        # 【原第1244行】条件判断：agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess；满足时执行下面缩进代码，否则走else（若有）。
        if agibot_gdk.gdk_init()!=agibot_gdk.GDKRes.kSuccess:
            # 【原第1245行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
            raise RuntimeError("GDK初始化失败")
        # 【原第1246行】计算/保存 initialized：True。
        initialized=True
        # 【原第1247行】计算/保存 robot：agibot_gdk.Robot()。
        robot=agibot_gdk.Robot()
        # 【原第1248行】计算/保存 tf_api：agibot_gdk.TF()。
        tf_api=agibot_gdk.TF()
        # 【原第1249行】计算/保存 camera：agibot_gdk.Camera()。
        camera=agibot_gdk.Camera()
        # 【原第1250行】计算/保存 model：YOLO(MODEL_PATH)。
        model=YOLO(MODEL_PATH)
        # 【原第1251行】条件判断：SHOW_YOLO_WINDOW；满足时执行下面缩进代码，否则走else（若有）。
        if SHOW_YOLO_WINDOW:
            # 【原第1252行】开始异常处理块：try执行正常操作，except处理异常，finally执行清理（若有）。
            try:
                # 【原第1253行】执行调用/更新：cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)。
                cv2.namedWindow(WINDOW_NAME,cv2.WINDOW_NORMAL)
                # 【原第1254行】执行调用/更新：cv2.resizeWindow(WINDOW_NAME, FULL_UI_WINDOW_WIDTH, FULL_UI_WINDOW_HEIGHT)。
                cv2.resizeWindow(WINDOW_NAME,FULL_UI_WINDOW_WIDTH,FULL_UI_WINDOW_HEIGHT)
                # 【原第1255行】执行调用/更新：cv2.moveWindow(WINDOW_NAME, 10, 10)。
                cv2.moveWindow(WINDOW_NAME,10,10)
            # 【原第1256行】异常分支：捕获对应异常，执行下面缩进代码；保持原处理方式。
            except cv2.error as exc:
                # 【原第1257行】打印当前进度或诊断信息到控制台；不发送机器人命令。
                print(f"[V90窗口创建失败] {exc}")
        # 【原第1258行】执行调用/更新：refresh_full_yolo_window('STARTUP', '启动独立90版')。
        refresh_full_yolo_window("STARTUP","启动独立90版")
        # 【原第1259行】暂停当前线程：time.sleep(3)；此等待不代表已验证机器人到位。
        time.sleep(3)

        # 【原第1261行】计算/保存 current：get_positions(robot, RIGHT_JOINTS)。
        current=get_positions(robot,RIGHT_JOINTS)
        # 【原第1262行】计算/保存 (name, index, error)：identify_start(current)。
        name,index,error=identify_start(current)
        # 【原第1263行】启动时读取的左臂保持位姿，双臂末端请求使用此参考。
        left=wait_pose(tf_api,LEFT_FRAME)
        # 【原第1264行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print("="*78)
        # 【原第1265行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print("90_g2_pick_wait_until_product_stable_full_yolo.py")
        # 【原第1266行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print(f"循环次数={CYCLE_COUNT}, 起始姿态={name}, 最大关节差={error:.4f}rad")
        # 【原第1267行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print("流程: 抓取 -> 抬升 -> 腰部右转80° -> 释放 -> 腰部反转80°")
        # 【原第1268行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print("      -> 右臂回WAYPOINT_2 -> WAYPOINT_1 -> 放入下一件 -> 重复")
        # 【原第1269行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print("释放和腰部正反转期间不发送双臂运动命令，并逐段验证双臂保持。")
        # 【原第1270行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print("="*78)
        # 【原第1271行】条件判断：not ENABLE_REAL_MOTION；满足时执行下面缩进代码，否则走else（若有）。
        if not ENABLE_REAL_MOTION:
            # 【原第1272行】计算/保存 report['status']：'DRY_RUN_PASS'。
            report["status"]="DRY_RUN_PASS"
            # 【原第1273行】调用save保存当前报告；便于定位本轮检测或运动问题。
            save(report)
            # 【原第1274行】打印当前进度或诊断信息到控制台；不发送机器人命令。
            print("DRY RUN通过")
            # 【原第1275行】返回结果给调用者；结束当前函数。
            return

        # 【原第1277行】计算/保存 base：torque_baseline(robot)。
        base=torque_baseline(robot)
        # 首轮先进入已验证视觉工作位。
        # 【原第1279行】条件判断：index == 0；满足时执行下面缩进代码，否则走else（若有）。
        if index==0:
            # 【原第1280行】执行调用/更新：move_arm_once(robot, tf_api, 'waypoint_1', WAYPOINT_1, base, report)。
            move_arm_once(robot,tf_api,"waypoint_1",WAYPOINT_1,base,report)
            # 【原第1281行】执行调用/更新：ensure_head(robot, report)。
            ensure_head(robot,report)
            # 【原第1282行】执行调用/更新：move_arm_once(robot, tf_api, 'waypoint_2', WAYPOINT_2, base, report)。
            move_arm_once(robot,tf_api,"waypoint_2",WAYPOINT_2,base,report)
        # 【原第1283行】条件判断：index == 1；满足时执行下面缩进代码，否则走else（若有）。
        elif index==1:
            # 【原第1284行】执行调用/更新：ensure_head(robot, report)。
            ensure_head(robot,report)
            # 【原第1285行】执行调用/更新：move_arm_once(robot, tf_api, 'waypoint_2', WAYPOINT_2, base, report)。
            move_arm_once(robot,tf_api,"waypoint_2",WAYPOINT_2,base,report)
        # 【原第1286行】否则分支：前面的条件不成立时执行这里。
        else:
            # 【原第1287行】执行调用/更新：ensure_head(robot, report)。
            ensure_head(robot,report)

        # 【原第1289行】循环：依次从 range(1, CYCLE_COUNT + 1) 取值到 cycle_index，执行下面缩进块。
        for cycle_index in range(1,CYCLE_COUNT+1):
            # 【原第1290行】打印当前进度或诊断信息到控制台；不发送机器人命令。
            print("\n"+"#"*78)
            # 【原第1291行】打印当前进度或诊断信息到控制台；不发送机器人命令。
            print(f"开始第 {cycle_index}/{CYCLE_COUNT} 次抓取循环")
            # 【原第1292行】打印当前进度或诊断信息到控制台；不发送机器人命令。
            print("#"*78)
            # 【原第1293行】计算/保存 report['current_cycle_index']：cycle_index。
            report["current_cycle_index"]=cycle_index
            # 【原第1294行】计算/保存 UI_STATE['cycle']：cycle_index。
            UI_STATE["cycle"]=cycle_index
            # 【原第1295行】执行调用/更新：refresh_full_yolo_window('CYCLE START', f'第{cycle_index}/{CYCLE_COUNT}轮')。
            refresh_full_yolo_window("CYCLE START",f"第{cycle_index}/{CYCLE_COUNT}轮")
            # 【原第1296行】计算/保存 report['joint_stages']：[]。
            report["joint_stages"]=[]
            # 【原第1297行】计算/保存 report['cartesian_stages']：[]。
            report["cartesian_stages"]=[]
            # 【原第1298行】计算/保存 report['angle_stages']：[]。
            report["angle_stages"]=[]
            # 【原第1299行】计算/保存 report['vertical_orientation_stages']：[]。
            report["vertical_orientation_stages"]=[]
            # 【原第1300行】计算/保存 report['waist_motion_segments']：[]。
            report["waist_motion_segments"]=[]
            # 【原第1301行】计算/保存 cycle_start_time：time.time()。
            cycle_start_time=time.time()

            # 第2、3轮从WAYPOINT_1自动进入WAYPOINT_2后再识别。
            # 【原第1304行】条件判断：cycle_index > 1；满足时执行下面缩进代码，否则走else（若有）。
            if cycle_index>1:
                # 【原第1305行】执行调用/更新：ensure_head(robot, report)。
                ensure_head(robot,report)
                # 【原第1306行】执行调用/更新：move_arm_once(robot, tf_api, 'waypoint_2', WAYPOINT_2, base, report)。
                move_arm_once(robot,tf_api,"waypoint_2",WAYPOINT_2,base,report)

            # 【原第1308行】计算/保存 wp2tcp：wait_pose(tf_api, TCP_FRAME)。
            wp2tcp=wait_pose(tf_api,TCP_FRAME)
            # 【原第1309行】计算/保存 wp2end：wait_pose(tf_api, RIGHT_FRAME)。
            wp2end=wait_pose(tf_api,RIGHT_FRAME)
            # 【原第1310行】计算/保存 det：detect_product(model, camera, tf_api, report)。
            det=detect_product(model,camera,tf_api,report)
            # 【原第1311行】计算/保存 product：det['base_xyz']。
            product=det["base_xyz"]
            # 【原第1312行】条件判断：det['table_z_m'] is None；满足时执行下面缩进代码，否则走else（若有）。
            if det["table_z_m"] is None:
                # 【原第1313行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
                raise RuntimeError("桌面Z估计失败，禁止下降抓取")

            # 【原第1315行】本轮原始桌面估计，保留用于与锁定基准比较。
            raw_table_z=float(det["table_z_m"])
            # 【原第1316行】声明使用模块级变量：CALIBRATED_TABLE_Z_M，赋值会影响后续循环。
            global CALIBRATED_TABLE_Z_M
            # 【原第1317行】条件判断：LOCK_TABLE_Z_ACROSS_CYCLES；满足时执行下面缩进代码，否则走else（若有）。
            if LOCK_TABLE_Z_ACROSS_CYCLES:
                # 【原第1318行】条件判断：CALIBRATED_TABLE_Z_M is None；满足时执行下面缩进代码，否则走else（若有）。
                if CALIBRATED_TABLE_Z_M is None:
                    # 【原第1319行】桌面基准；None时首轮稳定检测后锁定，后续复用。
                    CALIBRATED_TABLE_Z_M=raw_table_z
                    # 【原第1320行】打印当前进度或诊断信息到控制台；不发送机器人命令。
                    print(f"[V90] 锁定三循环桌面Z={CALIBRATED_TABLE_Z_M:.5f}m")
                # 【原第1321行】条件判断：abs(raw_table_z - CALIBRATED_TABLE_Z_M) > TABLE_Z_SANITY_DELTA_M；满足时执行下面缩进代码，否则走else（若有）。
                elif abs(raw_table_z-CALIBRATED_TABLE_Z_M)>TABLE_Z_SANITY_DELTA_M:
                    # 【原第1322行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
                    raise RuntimeError(
                        # 【原第1323行】继续上方跨行表达式的参数/数据项；所属模块：main。
                        f"当前桌面Z与锁定值差异过大: raw={raw_table_z:.5f}m, "
                        # 【原第1324行】继续上方跨行表达式的参数/数据项；所属模块：main。
                        f"locked={CALIBRATED_TABLE_Z_M:.5f}m"
                    # 【原第1325行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
                    )
                # 【原第1326行】计算/保存 table_z：CALIBRATED_TABLE_Z_M。
                table_z=CALIBRATED_TABLE_Z_M
            # 【原第1327行】否则分支：前面的条件不成立时执行这里。
            else:
                # 【原第1328行】计算/保存 table_z：raw_table_z。
                table_z=raw_table_z
            # 【原第1329行】原始产品Z减原始桌面Z，仅诊断记录，不决定最终高度。
            measured_product_height=float(product[2]-raw_table_z)
            # 【原第1330行】本轮采用固定40毫米，不使用测量值切换模式。
            effective_product_height=FIXED_PRODUCT_HEIGHT_M
            # 【原第1331行】计算/保存 product_height_source：'fixed_40mm_above_locked_table_z'。
            product_height_source="fixed_40mm_above_locked_table_z"
            # 【原第1332行】计算/保存 product：[float(product[0]), float(product[1]), table_z + FIXED_PRODUCT_HEIGHT_M]。
            product=[float(product[0]),float(product[1]),table_z+FIXED_PRODUCT_HEIGHT_M]
            # 【原第1333行】计算/保存 det['raw_table_z_m']：raw_table_z。
            det["raw_table_z_m"]=raw_table_z
            # 【原第1334行】计算/保存 det['raw_product_z_m']：float(det['base_xyz'][2])。
            det["raw_product_z_m"]=float(det["base_xyz"][2])
            # 【原第1335行】计算/保存 det['table_z_m']：table_z。
            det["table_z_m"]=table_z
            # 【原第1336行】计算/保存 det['base_xyz']：product。
            det["base_xyz"]=product
            # 【原第1337行】计算/保存 det['fixed_product_height_m']：FIXED_PRODUCT_HEIGHT_M。
            det["fixed_product_height_m"]=FIXED_PRODUCT_HEIGHT_M
            # 【原第1338行】计算/保存 det['height_source']：product_height_source。
            det["height_source"]=product_height_source

            # 【原第1340行】产品顶部绝对Base Z：锁定桌面Z加固定40毫米。
            product_top_z=table_z+effective_product_height
            # 【原第1341行】计算/保存 table_floor：table_z + MIN_TCP_ABOVE_TABLE_M。
            table_floor=table_z+MIN_TCP_ABOVE_TABLE_M
            # 【原第1342行】名义抓取TCP Z：产品顶部Z加抓取偏移。
            nominal_grasp_z=product_top_z+GRASP_ABOVE_PRODUCT_TOP_M
            # 【原第1343行】两项取较高值：名义抓取Z与桌面TCP下限。
            safe_grasp_z=max(nominal_grasp_z,table_floor)
            # 【原第1344行】计算/保存 grasp_z_source：'product_top_z_plus_offset' if nominal_grasp_z >= table_floor else 'table_safety_floor'。
            grasp_z_source=(
                # 【原第1345行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "product_top_z_plus_offset"
                # 【原第1346行】继续上方跨行表达式的参数/数据项；所属模块：main。
                if nominal_grasp_z>=table_floor
                # 【原第1347行】否则分支：前面的条件不成立时执行这里。
                else "table_safety_floor"
            # 【原第1348行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )
            # 【原第1349行】预抓取XYZ，XY沿产品，Z取顶部+120毫米与抓取点+60毫米较高者。
            pre=[
                # 【原第1350行】继续上方跨行表达式的参数/数据项；所属模块：main。
                product[0],product[1],
                # 【原第1351行】继续上方跨行表达式的参数/数据项；所属模块：main。
                max(
                    # 【原第1352行】继续上方跨行表达式的参数/数据项；所属模块：main。
                    product_top_z+PREGRASP_ABOVE_PRODUCT_M,
                    # 【原第1353行】继续上方跨行表达式的参数/数据项；所属模块：main。
                    safe_grasp_z+0.060,
                # 【原第1354行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
                ),
            # 【原第1355行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            ]
            # 【原第1356行】最终抓取TCP XYZ，XY沿产品，Z使用safe_grasp_z。
            grasp=[product[0],product[1],safe_grasp_z]
            # 【原第1357行】计算/保存 dx：pre[0] - wp2tcp.position[0]。
            dx=pre[0]-wp2tcp.position[0]
            # 【原第1358行】计算/保存 dy：pre[1] - wp2tcp.position[1]。
            dy=pre[1]-wp2tcp.position[1]
            # 【原第1359行】计算/保存 dz：pre[2] - wp2tcp.position[2]。
            dz=pre[2]-wp2tcp.position[2]

            # 【原第1361行】执行调用/更新：report.update({'detection': det, 'product': product, 'pregrasp': pre, 'grasp': grasp, 'table_z_m': table_z, 'measured_product_height_m': measured_product_height, 'effective_product。
            report.update({
                # 【原第1362行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "detection":det,
                # 【原第1363行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "product":product,
                # 【原第1364行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "pregrasp":pre,
                # 【原第1365行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "grasp":grasp,
                # 【原第1366行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "table_z_m":table_z,
                # 【原第1367行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "measured_product_height_m":measured_product_height,
                # 【原第1368行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "effective_product_height_m":effective_product_height,
                # 【原第1369行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "product_height_source":product_height_source,
                # 【原第1370行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "product_top_z":product_top_z,
                # 【原第1371行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "table_floor_tcp_z":table_floor,
                # 【原第1372行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "nominal_grasp_z":nominal_grasp_z,
                # 【原第1373行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "safe_grasp_z":safe_grasp_z,
                # 【原第1374行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "grasp_z_source":grasp_z_source,
            # 【原第1375行】继续上方跨行表达式的参数/数据项；所属模块：main。
            })
            # 【原第1376行】调用save保存当前报告；便于定位本轮检测或运动问题。
            save(report)
            # 【原第1377行】打印当前进度或诊断信息到控制台；不发送机器人命令。
            print(f"产品={np.round(product,5).tolist()}, PreGrasp={np.round(pre,5).tolist()}")
            # 【原第1378行】打印当前进度或诊断信息到控制台；不发送机器人命令。
            print(
                # 【原第1379行】继续上方跨行表达式的参数/数据项；所属模块：main。
                f"桌面Z={table_z:.5f}m, 有效产品高度={effective_product_height:.5f}m, "
                # 【原第1380行】继续上方跨行表达式的参数/数据项；所属模块：main。
                f"来源={product_height_source}"
            # 【原第1381行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )
            # 【原第1382行】打印当前进度或诊断信息到控制台；不发送机器人命令。
            print(f"最终Grasp Z={safe_grasp_z:.5f}m, 来源={grasp_z_source}")

            # 【原第1384行】条件判断：math.hypot(dx, dy) > MAX_XY_FROM_WP2_M or -dz > MAX_PREGRASP_DESCENT_M；满足时执行下面缩进代码，否则走else（若有）。
            if math.hypot(dx,dy)>MAX_XY_FROM_WP2_M or -dz>MAX_PREGRASP_DESCENT_M:
                # 【原第1385行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
                raise RuntimeError("产品超出工作窗口")
            # 【原第1386行】条件判断：not det['angle_valid']；满足时执行下面缩进代码，否则走else（若有）。
            if not det["angle_valid"]:
                # 【原第1387行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
                raise RuntimeError("产品边缘角度样本不足，禁止执行抓取")

            # 【原第1389行】计算/保存 raw_yaw：det['yaw_rad']。
            raw_yaw=det["yaw_rad"]
            # 【原第1390行】计算/保存 yaw_candidates：[raw_yaw, raw_yaw + math.pi, raw_yaw - math.pi]。
            yaw_candidates=[raw_yaw,raw_yaw+math.pi,raw_yaw-math.pi]
            # 【原第1391行】计算/保存 q_candidates：[vertical_tool_orientation(y + math.radians(GRIPPER_YAW_OFFSET_DEG)) for y in yaw_candidates]。
            q_candidates=[vertical_tool_orientation(y+math.radians(GRIPPER_YAW_OFFSET_DEG)) for y in yaw_candidates]
            # 【原第1392行】计算/保存 candidate_records：[{'yaw_rad': y, 'yaw_deg': math.degrees(y), 'orientation_change_deg': q_error_deg(wp2end.orientation, q)} for y, q in zip(yaw_candidates, q_candidates…。
            candidate_records=[
                # 【原第1393行】继续上方跨行表达式的参数/数据项；所属模块：main。
                {"yaw_rad":y,"yaw_deg":math.degrees(y),
                 # 【原第1394行】继续上方跨行表达式的参数/数据项；所属模块：main。
                 "orientation_change_deg":q_error_deg(wp2end.orientation,q)}
                # 【原第1395行】继续上方跨行表达式的参数/数据项；所属模块：main。
                for y,q in zip(yaw_candidates,q_candidates)
            # 【原第1396行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            ]
            # 【原第1397行】选择与当前WP2姿态四元数角差最小的等价长轴候选；不等于全路径可达性验证。
            best_index=min(range(len(q_candidates)),key=lambda i:candidate_records[i]["orientation_change_deg"])
            # 【原第1398行】计算/保存 yaw：yaw_candidates[best_index]。 计算/保存 target_q：q_candidates[best_index]。
            # 【本行拆解教学：分号语句】
            # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
            # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
            # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
            yaw=yaw_candidates[best_index]; target_q=q_candidates[best_index]
            # 【原第1399行】计算/保存 det['raw_yaw_rad']：raw_yaw。 计算/保存 det['raw_yaw_deg']：math.degrees(raw_yaw)。
            # 【本行拆解教学：分号语句】
            # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
            # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
            # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
            det["raw_yaw_rad"]=raw_yaw; det["raw_yaw_deg"]=math.degrees(raw_yaw)
            # 【原第1400行】计算/保存 det['yaw_rad']：yaw。 计算/保存 det['yaw_deg']：math.degrees(yaw)。
            # 【本行拆解教学：分号语句】
            # 这一物理行使用英文分号“;”连续写了多条Python语句。Python会严格从左到右依次执行。
            # 分号只是把多行压缩到同一行，不表示并行执行，也不会同时运行。
            # 阅读建议：把每个分号想象成换行，按“第1句→第2句→第3句”的顺序逐句理解。
            det["yaw_rad"]=yaw; det["yaw_deg"]=math.degrees(yaw)
            # 【原第1401行】计算/保存 det['axis_equivalent_candidates']：candidate_records。
            det["axis_equivalent_candidates"]=candidate_records
            # 【原第1402行】计算/保存 det['axis_selection_source']：'minimum_orientation_change_over_yaw_plus_minus_180'。
            det["axis_selection_source"]="minimum_orientation_change_over_yaw_plus_minus_180"
            # 【原第1403行】执行调用/更新：UI_STATE.update({'yaw_deg': math.degrees(yaw), 'table_z_m': table_z, 'product_xyz': product, 'pregrasp_xyz': pre, 'grasp_xyz': grasp})。
            UI_STATE.update({"yaw_deg":math.degrees(yaw),"table_z_m":table_z,
                             # 【原第1404行】继续上方跨行表达式的参数/数据项；所属模块：main。
                             "product_xyz":product,"pregrasp_xyz":pre,"grasp_xyz":grasp})
            # 【原第1405行】执行调用/更新：refresh_full_yolo_window('TARGET LOCKED', '固定40mm高度 + 180度最短长轴')。
            refresh_full_yolo_window("TARGET LOCKED","固定40mm高度 + 180度最短长轴")
            # 【原第1406行】计算/保存 workspace_xy_radius：math.hypot(product[0] - wp2tcp.position[0], product[1] - wp2tcp.position[1])。
            workspace_xy_radius=math.hypot(
                # 【原第1407行】继续上方跨行表达式的参数/数据项；所属模块：main。
                product[0]-wp2tcp.position[0],
                # 【原第1408行】继续上方跨行表达式的参数/数据项；所属模块：main。
                product[1]-wp2tcp.position[1],
            # 【原第1409行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )
            # 【原第1410行】条件判断：workspace_xy_radius > PRE_ORIENTATION_MAX_XY_MOVE_M；满足时执行下面缩进代码，否则走else（若有）。
            if workspace_xy_radius>PRE_ORIENTATION_MAX_XY_MOVE_M:
                # 【原第1411行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
                raise RuntimeError(
                    # 【原第1412行】继续上方跨行表达式的参数/数据项；所属模块：main。
                    f"产品相对WP2横向距离{workspace_xy_radius:.4f}m超过可靠范围"
                # 【原第1413行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
                )

            # 【原第1415行】条件判断：not ENABLE_XY_AND_PREGRASP；满足时执行下面缩进代码，否则走else（若有）。
            if not ENABLE_XY_AND_PREGRASP:
                # 【原第1416行】计算/保存 report['status']：'XY_AND_VERTICAL_PLAN_READY'。
                report["status"]="XY_AND_VERTICAL_PLAN_READY"
                # 【原第1417行】调用save保存当前报告；便于定位本轮检测或运动问题。
                save(report)
                # 【原第1418行】返回结果给调用者；结束当前函数。
                return

            # 高位粗对准 -> 固定TCP锚点垂直姿态 -> 小范围精修。
            # 【原第1421行】执行调用/更新：move_tcp_closed_loop(robot, tf_api, left, [pre[0] - wp2tcp.position[0], pre[1] - wp2tcp.position[1], 0.0], wp2end.orientation, base, report, 'coarse_align_xy_before_orientation')。
            move_tcp_closed_loop(
                # 【原第1422行】继续上方跨行表达式的参数/数据项；所属模块：main。
                robot,tf_api,left,
                # 【原第1423行】继续上方跨行表达式的参数/数据项；所属模块：main。
                [pre[0]-wp2tcp.position[0],pre[1]-wp2tcp.position[1],0.0],
                # 【原第1424行】继续上方跨行表达式的参数/数据项；所属模块：main。
                wp2end.orientation,base,report,
                # 【原第1425行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "coarse_align_xy_before_orientation",
            # 【原第1426行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )
            # 【原第1427行】条件判断：not ENABLE_ANGLE_ALIGNMENT；满足时执行下面缩进代码，否则走else（若有）。
            if not ENABLE_ANGLE_ALIGNMENT:
                # 【原第1428行】计算/保存 report['status']：'COARSE_XY_ALIGNED_VERTICAL_DISABLED'。
                report["status"]="COARSE_XY_ALIGNED_VERTICAL_DISABLED"
                # 【原第1429行】调用save保存当前报告；便于定位本轮检测或运动问题。
                save(report)
                # 【原第1430行】返回结果给调用者；结束当前函数。
                return

            # 【原第1432行】计算/保存 (aligned_tcp, aligned_end)：align_orientation_target(robot, tf_api, left, target_q, base, report)。
            aligned_tcp,aligned_end=align_orientation_target(
                # 【原第1433行】继续上方跨行表达式的参数/数据项；所属模块：main。
                robot,tf_api,left,target_q,base,report
            # 【原第1434行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )
            # 【原第1435行】计算/保存 q：aligned_end.orientation。
            q=aligned_end.orientation
            # 【原第1436行】计算/保存 residual_xy：math.hypot(pre[0] - aligned_tcp.position[0], pre[1] - aligned_tcp.position[1])。
            residual_xy=math.hypot(
                # 【原第1437行】继续上方跨行表达式的参数/数据项；所属模块：main。
                pre[0]-aligned_tcp.position[0],
                # 【原第1438行】继续上方跨行表达式的参数/数据项；所属模块：main。
                pre[1]-aligned_tcp.position[1],
            # 【原第1439行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )
            # 【原第1440行】打印当前进度或诊断信息到控制台；不发送机器人命令。
            print(f"垂直姿态后XY残差={residual_xy:.4f}m")
            # 【原第1441行】条件判断：residual_xy > POST_ORIENTATION_MAX_XY_RESIDUAL_M；满足时执行下面缩进代码，否则走else（若有）。
            if residual_xy>POST_ORIENTATION_MAX_XY_RESIDUAL_M:
                # 【原第1442行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
                raise RuntimeError("垂直姿态后XY残差超限")

            # 【原第1444行】计算/保存 (aligned_tcp, _)：move_tcp_closed_loop(robot, tf_api, left, [pre[0] - aligned_tcp.position[0], pre[1] - aligned_tcp.position[1], 0.0], q, base, report, 'fine_align_xy_a…。
            aligned_tcp,_=move_tcp_closed_loop(
                # 【原第1445行】继续上方跨行表达式的参数/数据项；所属模块：main。
                robot,tf_api,left,
                # 【原第1446行】继续上方跨行表达式的参数/数据项；所属模块：main。
                [pre[0]-aligned_tcp.position[0],pre[1]-aligned_tcp.position[1],0.0],
                # 【原第1447行】继续上方跨行表达式的参数/数据项；所属模块：main。
                q,base,report,"fine_align_xy_after_orientation",
            # 【原第1448行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )
            # 【原第1449行】计算/保存 cur：wait_pose(tf_api, TCP_FRAME)。
            cur=wait_pose(tf_api,TCP_FRAME)
            # 【原第1450行】执行调用/更新：move_tcp_closed_loop(robot, tf_api, left, [0.0, 0.0, pre[2] - cur.position[2]], q, base, report, 'descend_pregrasp')。
            move_tcp_closed_loop(
                # 【原第1451行】继续上方跨行表达式的参数/数据项；所属模块：main。
                robot,tf_api,left,
                # 【原第1452行】继续上方跨行表达式的参数/数据项；所属模块：main。
                [0.0,0.0,pre[2]-cur.position[2]],
                # 【原第1453行】继续上方跨行表达式的参数/数据项；所属模块：main。
                q,base,report,"descend_pregrasp",
            # 【原第1454行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )
            # 【原第1455行】条件判断：not ENABLE_GRIPPER_AND_PICK；满足时执行下面缩进代码，否则走else（若有）。
            if not ENABLE_GRIPPER_AND_PICK:
                # 【原第1456行】计算/保存 report['status']：'PREGRASP_REACHED'。
                report["status"]="PREGRASP_REACHED"
                # 【原第1457行】调用save保存当前报告；便于定位本轮检测或运动问题。
                save(report)
                # 【原第1458行】返回结果给调用者；结束当前函数。
                return

            # 【原第1460行】调用夹爪控制函数；参数沿用原程序，不更改开合逻辑。
            set_gripper(robot,-0.785)
            # 【原第1461行】计算/保存 cur：wait_pose(tf_api, TCP_FRAME)。
            cur=wait_pose(tf_api,TCP_FRAME)
            # 【原第1462行】执行调用/更新：move_tcp_closed_loop(robot, tf_api, left, [0.0, 0.0, grasp[2] - cur.position[2]], q, base, report, 'descend_grasp')。
            move_tcp_closed_loop(
                # 【原第1463行】继续上方跨行表达式的参数/数据项；所属模块：main。
                robot,tf_api,left,
                # 【原第1464行】继续上方跨行表达式的参数/数据项；所属模块：main。
                [0.0,0.0,grasp[2]-cur.position[2]],
                # 【原第1465行】继续上方跨行表达式的参数/数据项；所属模块：main。
                q,base,report,"descend_grasp",
            # 【原第1466行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )
            # 【原第1467行】调用夹爪控制函数；参数沿用原程序，不更改开合逻辑。
            set_gripper(robot,0.0)
            # 【原第1468行】执行调用/更新：move_tcp_closed_loop(robot, tf_api, left, [0.0, 0.0, LIFT_AFTER_GRASP_M], q, base, report, 'lift_10cm')。
            move_tcp_closed_loop(
                # 【原第1469行】继续上方跨行表达式的参数/数据项；所属模块：main。
                robot,tf_api,left,
                # 【原第1470行】继续上方跨行表达式的参数/数据项；所属模块：main。
                [0.0,0.0,LIFT_AFTER_GRASP_M],
                # 【原第1471行】继续上方跨行表达式的参数/数据项；所属模块：main。
                q,base,report,"lift_10cm",
            # 【原第1472行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            )
            # 【原第1473行】打印当前进度或诊断信息到控制台；不发送机器人命令。
            print(f"第{cycle_index}轮抓取并抬升10cm完成")

            # 【原第1475行】条件判断：ENABLE_WAIST_TURN_AND_RELEASE；满足时执行下面缩进代码，否则走else（若有）。
            if ENABLE_WAIST_TURN_AND_RELEASE:
                # 【原第1476行】执行调用/更新：turn_release_and_return_waist(robot, report)。
                turn_release_and_return_waist(robot,report)

            # 腰部已回原角度；再按用户指定顺序回WAYPOINT_2和WAYPOINT_1。
            # 【原第1479行】执行调用/更新：move_arm_once(robot, tf_api, 'waypoint_2', WAYPOINT_2, base, report)。
            move_arm_once(robot,tf_api,"waypoint_2",WAYPOINT_2,base,report)
            # 【原第1480行】执行调用/更新：move_arm_once(robot, tf_api, 'waypoint_1', WAYPOINT_1, base, report)。
            move_arm_once(robot,tf_api,"waypoint_1",WAYPOINT_1,base,report)

            # 【原第1482行】计算/保存 cycle_result：{'cycle_index': cycle_index, 'start_time_epoch_s': cycle_start_time, 'end_time_epoch_s': time.time(), 'status': 'COMPLETED_RETURNED_WAYPOINT1', 'produ…。
            cycle_result={
                # 【原第1483行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "cycle_index":cycle_index,
                # 【原第1484行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "start_time_epoch_s":cycle_start_time,
                # 【原第1485行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "end_time_epoch_s":time.time(),
                # 【原第1486行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "status":"COMPLETED_RETURNED_WAYPOINT1",
                # 【原第1487行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "product":copy.deepcopy(product),
                # 【原第1488行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "pregrasp":copy.deepcopy(pre),
                # 【原第1489行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "grasp":copy.deepcopy(grasp),
                # 【原第1490行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "detection":copy.deepcopy(det),
                # 【原第1491行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "joint_stages":copy.deepcopy(report.get("joint_stages",[])),
                # 【原第1492行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "cartesian_stages":copy.deepcopy(report.get("cartesian_stages",[])),
                # 【原第1493行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "vertical_orientation_stages":copy.deepcopy(
                    # 【原第1494行】继续上方跨行表达式的参数/数据项；所属模块：main。
                    report.get("vertical_orientation_stages",[])
                # 【原第1495行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
                ),
                # 【原第1496行】继续上方跨行表达式的参数/数据项；所属模块：main。
                "waist_motion_segments":copy.deepcopy(
                    # 【原第1497行】继续上方跨行表达式的参数/数据项；所属模块：main。
                    report.get("waist_motion_segments",[])
                # 【原第1498行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
                ),
            # 【原第1499行】结束上方跨行的参数、列表或字典；仍属于同一个表达式。
            }
            # 【原第1500行】执行调用/更新：report['cycles'].append(cycle_result)。
            report["cycles"].append(cycle_result)
            # 【原第1501行】计算/保存 report['completed_cycles']：cycle_index。
            report["completed_cycles"]=cycle_index
            # 【原第1502行】计算/保存 report['status']：f'CYCLE_{cycle_index}_COMPLETED'。
            report["status"]=f"CYCLE_{cycle_index}_COMPLETED"
            # 【原第1503行】调用save保存当前报告；便于定位本轮检测或运动问题。
            save(report)
            # 【原第1504行】打印当前进度或诊断信息到控制台；不发送机器人命令。
            print(f"第{cycle_index}/{CYCLE_COUNT}轮完成，右臂已回到WAYPOINT_1。")

            # 【原第1506行】条件判断：cycle_index < CYCLE_COUNT；满足时执行下面缩进代码，否则走else（若有）。
            if cycle_index<CYCLE_COUNT:
                # 【原第1507行】条件判断：WAIT_FOR_PRODUCT_CONFIRMATION；满足时执行下面缩进代码，否则走else（若有）。
                if WAIT_FOR_PRODUCT_CONFIRMATION:
                    # 【原第1508行】计算/保存 command：input('请放置下一件产品并确认工作区安全，输入 READY：').strip().upper()。
                    command=input(
                        # 【原第1509行】继续上方跨行表达式的参数/数据项；所属模块：main。
                        "请放置下一件产品并确认工作区安全，输入 READY："
                    # 【原第1510行】继续上方跨行表达式的参数/数据项；所属模块：main。
                    ).strip().upper()
                    # 【原第1511行】条件判断：command != 'READY'；满足时执行下面缩进代码，否则走else（若有）。
                    if command!="READY":
                        # 【原第1512行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
                        raise RuntimeError("未确认下一件产品已放置")
                # 【原第1513行】否则分支：前面的条件不成立时执行这里。
                else:
                    # 【原第1514行】打印当前进度或诊断信息到控制台；不发送机器人命令。
                    print("已回到WAYPOINT_1；下一轮将在WAYPOINT_2持续等待产品稳定后自动抓取。")
                    # 【原第1515行】执行调用/更新：refresh_full_yolo_window('WAIT NEXT PRODUCT', '无需READY；到视觉位后持续等待产品稳定')。
                    refresh_full_yolo_window("WAIT NEXT PRODUCT","无需READY；到视觉位后持续等待产品稳定")

        # 【原第1517行】计算/保存 report['status']：'THREE_CYCLE_PICK_PLACE_COMPLETED'。
        report["status"]="THREE_CYCLE_PICK_PLACE_COMPLETED"
        # 【原第1518行】调用save保存当前报告；便于定位本轮检测或运动问题。
        save(report)
        # 【原第1519行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print("="*78)
        # 【原第1520行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print(f"全部{CYCLE_COUNT}轮抓取、放置、腰部复位和手臂回位完成。")
        # 【原第1521行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print("右臂最终停在WAYPOINT_1。")
        # 【原第1522行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print("="*78)

    # 【原第1524行】异常分支：捕获对应异常，执行下面缩进代码；保持原处理方式。
    except KeyboardInterrupt:
        # 【原第1525行】计算/保存 report['status']：'INTERRUPTED'。
        report["status"]="INTERRUPTED"
        # 【原第1526行】调用save保存当前报告；便于定位本轮检测或运动问题。
        save(report)
    # 【原第1527行】异常分支：捕获对应异常，执行下面缩进代码；保持原处理方式。
    except Exception as exc:
        # 【原第1528行】计算/保存 report['status']：'ERROR'。
        report["status"]="ERROR"
        # 【原第1529行】计算/保存 report['error']：str(exc)。
        report["error"]=str(exc)
        # 【原第1530行】计算/保存 report['failed_cycle_index']：report.get('current_cycle_index')。
        report["failed_cycle_index"]=report.get("current_cycle_index")
        # 【原第1531行】调用save保存当前报告；便于定位本轮检测或运动问题。
        save(report)
        # 【原第1532行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print("="*78)
        # 【原第1533行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print("程序异常停止，机器人可能停在非HOME/WAYPOINT姿态。")
        # 【原第1534行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print("重新运行前必须检查右臂、腰部和夹爪实际状态。")
        # 【原第1535行】打印当前进度或诊断信息到控制台；不发送机器人命令。
        print("="*78)
        # 【原第1536行】抛出异常并退出当前正常流程；由外层except记录或继续向上传递。
        raise
    # 【原第1537行】清理分支：无论正常结束还是异常，均尝试释放窗口、相机和GDK。
    finally:
        # 【原第1538行】开始异常处理块：try执行正常操作，except处理异常，finally执行清理（若有）。
        try:
            # 【原第1539行】执行调用/更新：cv2.destroyAllWindows()。
            cv2.destroyAllWindows()
        # 【原第1540行】异常分支：捕获对应异常，执行下面缩进代码；保持原处理方式。
        except Exception:
            # 【原第1541行】此分支不执行额外操作；原程序选择忽略该异常或情况。
            pass
        # 【原第1542行】条件判断：camera is not None；满足时执行下面缩进代码，否则走else（若有）。
        if camera is not None:
            # 【原第1543行】开始异常处理块：try执行正常操作，except处理异常，finally执行清理（若有）。
            try:
                # 【原第1544行】执行调用/更新：camera.close_camera()。
                camera.close_camera()
            # 【原第1545行】异常分支：捕获对应异常，执行下面缩进代码；保持原处理方式。
            except Exception:
                # 【原第1546行】此分支不执行额外操作；原程序选择忽略该异常或情况。
                pass
        # 【原第1547行】条件判断：initialized；满足时执行下面缩进代码，否则走else（若有）。
        if initialized:
            # 【原第1548行】执行调用/更新：agibot_gdk.gdk_release()。
            agibot_gdk.gdk_release()


# 【原第1551行】条件判断：__name__ == '__main__'；满足时执行下面缩进代码，否则走else（若有）。
if __name__=="__main__":
    # 【原第1552行】执行调用/更新：main()。
    main()

# 83版说明：全自动；产品高度模型40mm；抓取Z基于桌面+有效产品高度。

# 84版: Fast AutoRun + Persistent YOLO Window

# 85版优化：垂直姿态约6大段、腰部4段；高度、目标、容差和安全检查保持84版。

# 86版：三次循环抓取；释放后双臂不动，腰部反转回原位，再回WAYPOINT_2/WAYPOINT_1。

# 90版：独立单文件。产品稳定门控、固定40mm高度、三循环锁定桌面Z、180度等价最短长轴、全程YOLO大屏。
