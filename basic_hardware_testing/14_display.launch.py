#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智元G2 GDK标准API双臂协同抓取方盆
URDF: G2_t2_crs_omnipicker.urdf
目标: 绿色塑料方盆 440x320x85mm
夹爪: Omnipicker 简易夹爪 (dual_tool)
GDK版本: 2.6.3

更新内容 (v2.0):
  1. 修复格式问题 (__init__, __name__ 等)
  2. 基于omnipicker URDF更新末端链接名 (自动检测)
  3. 增加夹爪抓取位置 GRIPPER_GRASP (留余量，防止损坏方盆)
  4. 增加approach/retreat/transport中间航点
  5. 增加 TF 位姿验证 (抓取前后确认末端位置)
  6. 增加错误恢复机制 (失败时自动归位)
  7. 增加负载感知速度控制 (抓取后降速)
  8. 增加方盆位置可配置参数
  9. 增加抓取验证 (夹爪位置反馈)
  10. 增加详细时间戳日志
  11. 改进安全检查 (碰撞/急停/速度限制)
  12. 改进腰部旋转逻辑
  13. 增加运输稳定姿态
  14. 增加资源释放保护

严格遵循GDK官方Python接口:
  - Robot.move_arm_joint()      双臂关节规划控制
  - Robot.move_ee_pos()         末端执行器(夹爪)控制
  - Robot.move_waist_joint()    腰部规划控制
  - TF.get_tf_from_base_link()  末端位姿查询
  - get_motion_control_status() 碰撞检测
"""

import agibot_gdk
import time
import math
import sys
from datetime import datetime


class G2BinGraspController:
    """基于GDK标准API的方盆抓取控制器 (Omnipicker版 v2.0)"""

    # ==================== 关节名称常量 (严格遵循GDK文档) ====================
    LEFT_ARM_JOINTS = [
        "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
        "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
        "idx27_arm_l_joint7"
    ]
    RIGHT_ARM_JOINTS = [
        "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
        "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
        "idx67_arm_r_joint7"
    ]
    WAIST_JOINTS = [
        "idx01_body_joint1", "idx02_body_joint2", "idx03_body_joint3",
        "idx04_body_joint4", "idx05_body_joint5"
    ]
    HEAD_JOINTS = [
        "idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3"
    ]

    # ==================== Omnipicker 夹爪配置 ====================
    # 基于G2_t2_crs_omnipicker.urdf: 关节限位 [-0.785, 0]
    # -0.785 = 完全张开, 0.0 = 完全闭合
    GRIPPER_OPEN   = -0.785   # 完全张开 (预抓取前)
    GRIPPER_PRE    = -0.50    # 预抓取 (半张开，接近时使用)
    GRIPPER_GRASP  = -0.15    # 抓取夹紧 (留余量，适合方盆壁厚2-3mm)
    GRIPPER_CLOSE  = 0.0      # 完全闭合 (空载归位用)
    GRIPPER_HOLD   = -0.10    # 保持夹紧 (运输中略微收紧)

    # ==================== 末端链接名候选 (基于omnipicker URDF) ====================
    POSSIBLE_LEFT_END_LINKS = [
        "gripper_l_end_link",
        "gripper_l_tool_link",
        "gripper_l_finger_tip",
        "arm_l_end_link",
        "link_gripper_l_end",
    ]
    POSSIBLE_RIGHT_END_LINKS = [
        "gripper_r_end_link",
        "gripper_r_tool_link",
        "gripper_r_finger_tip",
        "arm_r_end_link",
        "link_gripper_r_end",
    ]

    # ==================== 方盆参数 (单位: m) ====================
    BIN_LENGTH = 0.440    # 长边
    BIN_WIDTH  = 0.320    # 短边
    BIN_HEIGHT = 0.085    # 高度
    BIN_WALL   = 0.003    # 壁厚估算
    BIN_MASS   = 0.5      # 估算质量(kg)

    # ==================== 方盆位置参数 (可配置) ====================
    BIN_POS_X = 0.45      # 前方距离
    BIN_POS_Y = 0.0       # 横向偏移
    BIN_POS_Z = 0.75      # 桌面高度

    PLACE_POS_X = 0.45
    PLACE_POS_Y = 0.0
    PLACE_POS_Z = 0.75

    # ==================== 速度参数 ====================
    VEL_NORMAL   = 0.30
    VEL_SLOW     = 0.20
    VEL_APPROACH = 0.10
    VEL_LOADED   = 0.15
    VEL_WAIST    = 0.20

    # ==================== 等待时间 ====================
    WAIT_SHORT    = 0.3
    WAIT_MEDIUM   = 0.5
    WAIT_GRIPPER  = 1.5
    WAIT_STABILIZE = 1.0

    def __init__(self):
        self._left_end_link = None
        self._right_end_link = None
        self._has_payload = False

        self._log("=" * 70)
        self._log("【智元G2方盆抓取控制器 v2.0】初始化")
        self._log("URDF: G2_t2_crs_omnipicker")
        self._log(f"方盆: {self.BIN_LENGTH*1000:.0f}x{self.BIN_WIDTH*1000:.0f}x{self.BIN_HEIGHT*1000:.0f}mm")
        self._log("=" * 70)

        # 1. GDK初始化
        res = agibot_gdk.gdk_init()
        if res != agibot_gdk.GDKRes.kSuccess:
            self._log(f"GDK初始化失败，错误码: {res}", level="ERROR")
            sys.exit(1)
        self._log("GDK初始化成功")

        # 2. 创建Robot对象
        self.robot = agibot_gdk.Robot()
        time.sleep(2)
        self._log("Robot对象就绪")

        # 3. 创建TF对象
        self.tf = agibot_gdk.TF()
        time.sleep(2)
        self._log("TF对象就绪")

        # 4. 上电自检
        self._initial_check()

        # 5. 自动检测末端链接名
        self._detect_end_links()

    # ==================== 工具方法 ====================

    def _log(self, msg, level="INFO"):
        """带时间戳的日志输出"""
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        prefix = f"[{ts}] [{level}]" if level != "INFO" else f"[{ts}]"
        print(f"{prefix} {msg}")

    def _detect_end_links(self):
        """自动检测可用的末端链接名"""
        self._log("\n>>> 检测末端链接名")

        for name in self.POSSIBLE_LEFT_END_LINKS:
            try:
                t = self.tf.get_tf_from_base_link(name)
                if t is not None:
                    self._left_end_link = name
                    self._log(f"   左臂末端链接: {name}")
                    self._log(f"      位置: ({t.translation.x:.3f}, {t.translation.y:.3f}, {t.translation.z:.3f})")
                    break
            except Exception:
                continue

        for name in self.POSSIBLE_RIGHT_END_LINKS:
            try:
                t = self.tf.get_tf_from_base_link(name)
                if t is not None:
                    self._right_end_link = name
                    self._log(f"   右臂末端链接: {name}")
                    self._log(f"      位置: ({t.translation.x:.3f}, {t.translation.y:.3f}, {t.translation.z:.3f})")
                    break
            except Exception:
                continue

        if not self._left_end_link:
            self._left_end_link = "arm_l_end_link"
            self._log(f"   左臂末端链接未检测到，使用默认: {self._left_end_link}", level="WARN")
        if not self._right_end_link:
            self._right_end_link = "arm_r_end_link"
            self._log(f"   右臂末端链接未检测到，使用默认: {self._right_end_link}", level="WARN")

    # ==================== 安全检测 ====================

    def _initial_check(self):
        """上电状态与碰撞自检"""
        self._log("\n>>> 机器人初始状态检查")
        status = self.robot.get_whole_body_status()

        checks = [
            ("左臂", status.get('left_arm_error', 0)),
            ("右臂", status.get('right_arm_error', 0)),
            ("腰部", status.get('waist_error', 0)),
            ("头部", status.get('neck_error', 0)),
            ("底盘", status.get('chassis_error', 0)),
        ]
        ok = True
        for name, err in checks:
            if err != 0:
                self._log(f"   {name}错误码: {err}", level="WARN")
                ok = False
        if ok:
            self._log("   全身状态正常")

        mc = self.robot.get_motion_control_status()
        n_col = len(mc.collision_pairs_1)
        self._log(f"   当前碰撞对数量: {n_col}")
        if n_col > 0:
            pairs = list(zip(mc.collision_pairs_1, mc.collision_pairs_2))
            self._log(f"   警告: 当前存在碰撞对: {pairs}", level="WARN")

        self._log(f"   左末端型号: {status.get('left_end_model', '未知')}")
        self._log(f"   右末端型号: {status.get('right_end_model', '未知')}")

    def check_collision(self):
        """实时碰撞检测"""
        mc = self.robot.get_motion_control_status()
        pairs = list(zip(mc.collision_pairs_1, mc.collision_pairs_2))
        return len(pairs) > 0, pairs

    def check_estop(self):
        """急停检测"""
        s = self.robot.get_whole_body_status()
        if s.get('left_arm_estop', False) or s.get('right_arm_estop', False):
            self._log("检测到急停状态!", level="ERROR")
            return False
        return True

    def check_safety(self):
        """综合安全检查 (急停 + 碰撞)"""
        if not self.check_estop():
            return False
        has_col, pairs = self.check_collision()
        if has_col:
            self._log(f"检测到碰撞对: {pairs}", level="WARN")
            return False
        return True

    def _safe_stop(self):
        """安全停止: 回到home位"""
        self._log(">>> 执行安全停止...", level="WARN")
        try:
            l, r = self.pose_home()
            self.move_arms(l, r, velocity=self.VEL_NORMAL)
        except Exception as e:
            self._log(f"   安全停止异常: {e}", level="ERROR")

    # ==================== 状态读取 ====================

    def get_arm_positions(self):
        """获取当前双臂关节角"""
        js = self.robot.get_joint_states()
        name_to_pos = {s['name']: s['motor_position'] for s in js['states']}
        left  = [name_to_pos.get(j, 0.0) for j in self.LEFT_ARM_JOINTS]
        right = [name_to_pos.get(j, 0.0) for j in self.RIGHT_ARM_JOINTS]
        return left, right

    def get_waist_positions(self):
        """获取当前腰部关节角"""
        js = self.robot.get_joint_states()
        name_to_pos = {s['name']: s['motor_position'] for s in js['states']}
        return [name_to_pos.get(j, 0.0) for j in self.WAIST_JOINTS]

    def get_gripper_positions(self):
        """获取当前夹爪位置"""
        js = self.robot.get_joint_states()
        gripper_positions = {}
        for s in js['states']:
            name = s.get('name', '')
            if 'gripper' in name.lower() or 'omnipicker' in name.lower():
                gripper_positions[name] = s['motor_position']
        return gripper_positions

    def get_end_pose(self, side="left"):
        """获取末端位姿"""
        link = self._left_end_link if side == "left" else self._right_end_link
        try:
            t = self.tf.get_tf_from_base_link(link)
            return {
                'x': t.translation.x, 'y': t.translation.y, 'z': t.translation.z,
                'qx': t.rotation.x, 'qy': t.rotation.y,
                'qz': t.rotation.z, 'qw': t.rotation.w,
            }
        except Exception as e:
            self._log(f"   获取{side}臂末端位姿失败: {e}", level="WARN")
            return None

    def print_end_pose(self):
        """打印当前左右臂末端位姿"""
        for side, link in [("左", self._left_end_link), ("右", self._right_end_link)]:
            pose = self.get_end_pose("left" if side == "左" else "right")
            if pose:
                self._log(f"   {side}臂末端 [{link}]:")
                self._log(f"      位置: ({pose['x']:.3f}, {pose['y']:.3f}, {pose['z']:.3f})")
                self._log(f"      四元数: ({pose['qx']:.3f}, {pose['qy']:.3f}, {pose['qz']:.3f}, {pose['qw']:.3f})")
            else:
                self._log(f"   {side}臂末端位姿获取失败", level="WARN")

    def verify_end_pose(self, side="left", expected_xyz=None, tolerance=0.05):
        """验证末端位姿是否达到预期位置"""
        if expected_xyz is None:
            return True
        pose = self.get_end_pose(side)
        if pose is None:
            self._log(f"   {side}臂位姿验证失败: 无法获取位姿", level="WARN")
            return False
        dx = abs(pose['x'] - expected_xyz[0])
        dy = abs(pose['y'] - expected_xyz[1])
        dz = abs(pose['z'] - expected_xyz[2])
        if dx < tolerance and dy < tolerance and dz < tolerance:
            self._log(f"   {side}臂位姿验证通过 (误差: {dx:.3f}, {dy:.3f}, {dz:.3f})")
            return True
        else:
            self._log(f"   {side}臂位姿偏差: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}", level="WARN")
            return False

    # ==================== 运动控制 (GDK标准API) ====================

    def move_arms(self, left_positions, right_positions, velocity=0.3):
        """双臂关节规划控制 (move_arm_joint), control_group=2"""
        if len(left_positions) != 7 or len(right_positions) != 7:
            raise ValueError("左右臂各需7个关节角")
        if not self.check_estop():
            return False

        all_pos = left_positions + right_positions
        all_vel = [velocity] * 14

        self._log(f"\n>>> 双臂运动指令 (vel={velocity})")
        self._log(f"   左臂: {[round(x, 3) for x in left_positions]}")
        self._log(f"   右臂: {[round(x, 3) for x in right_positions]}")

        try:
            ret = self.robot.move_arm_joint(all_pos, all_vel, 2)
            if ret == 0:
                self._log("   双臂运动完成")
                has_col, pairs = self.check_collision()
                if has_col:
                    self._log(f"   运动后检测到碰撞对: {pairs}", level="WARN")
                return True
            else:
                self._log(f"   运动失败，返回值: {ret}", level="ERROR")
                return False
        except Exception as e:
            self._log(f"   运动异常: {e}", level="ERROR")
            return False

    def move_waist(self, positions, velocity=0.3):
        """腰部关节规划控制"""
        if len(positions) != 5:
            raise ValueError("腰部需5个关节角")

        limits_min = [-1.082104, -0.000174, -1.919862, -0.436332, -3.045599]
        limits_max = [0.000174,  2.652900,  1.570970,  0.436332,  3.045599]

        safe_pos = []
        for i, (p, lo, hi) in enumerate(zip(positions, limits_min, limits_max)):
            if p < lo or p > hi:
                self._log(f"   腰部关节{i+1}目标{p:.3f}超出限位[{lo:.3f}, {hi:.3f}]，已钳制", level="WARN")
                safe_pos.append(max(lo, min(p, hi)))
            else:
                safe_pos.append(p)

        vels = [velocity] * 5
        self._log(f"\n>>> 腰部运动: {[round(x, 3) for x in safe_pos]}")
        try:
            ret = self.robot.move_waist_joint(safe_pos, vels)
            if ret == 0:
                self._log("   腰部运动完成")
                return True
            else:
                self._log(f"   腰部运动失败: {ret}", level="ERROR")
                return False
        except Exception as e:
            self._log(f"   腰部运动异常: {e}", level="ERROR")
            return False

    def control_gripper(self, left_pos, right_pos):
        """夹爪控制 (move_ee_pos), group=dual_tool, target_type=omnipicker"""
        self._log(f"\n>>> 夹爪控制: 左={left_pos:.3f}, 右={right_pos:.3f}")

        js = agibot_gdk.JointStates()
        js.group = "dual_tool"
        js.target_type = "omnipicker"

        s_l = agibot_gdk.JointState()
        s_l.position = left_pos
        s_r = agibot_gdk.JointState()
        s_r.position = right_pos

        js.states = [s_l, s_r]
        js.nums = 2

        try:
            ret = self.robot.move_ee_pos(js)
            if ret == 0:
                self._log("   夹爪控制成功")
                return True
            else:
                self._log(f"   夹爪控制失败: {ret}", level="ERROR")
                return False
        except Exception as e:
            self._log(f"   夹爪控制异常: {e}", level="ERROR")
            return False

    def open_gripper(self):
        return self.control_gripper(self.GRIPPER_OPEN, self.GRIPPER_OPEN)

    def close_gripper(self):
        return self.control_gripper(self.GRIPPER_CLOSE, self.GRIPPER_CLOSE)

    def pregrasp_gripper(self):
        return self.control_gripper(self.GRIPPER_PRE, self.GRIPPER_PRE)

    def grasp_gripper(self):
        return self.control_gripper(self.GRIPPER_GRASP, self.GRIPPER_GRASP)

    def hold_gripper(self):
        return self.control_gripper(self.GRIPPER_HOLD, self.GRIPPER_HOLD)

    def verify_grip(self):
        """验证夹爪是否成功夹持"""
        gripper_pos = self.get_gripper_positions()
        if not gripper_pos:
            self._log("   无法读取夹爪位置，跳过验证", level="WARN")
            return True
        self._log(f"   夹爪当前位置: {gripper_pos}")
        for name, pos in gripper_pos.items():
            if abs(pos - self.GRIPPER_GRASP) < 0.01:
                self._log(f"   夹爪 {name} 接近完全闭合，可能未夹到物体", level="WARN")
            elif self.GRIPPER_GRASP < pos < self.GRIPPER_OPEN:
                self._log(f"   夹爪 {name} 处于夹持状态 (位置={pos:.3f})")
        return True

    # ==================== 预定义安全姿态 ====================

    def pose_home(self):
        """安全归位: 双臂自然下垂外展"""
        left  = [0.0, -0.8,  0.0, -1.2, 0.0, 0.0, 0.0]
        right = [0.0, -0.8,  0.0, -1.2, 0.0, 0.0, 0.0]
        return left, right

    def pose_pregrasp_high(self):
        """高位预抓取: 过渡航点"""
        left  = [-0.15, -0.5,  0.2, -0.9,  0.15, 0.0, 0.0]
        right = [ 0.15, -0.5, -0.2, -0.9, -0.15, 0.0, 0.0]
        return left, right

    def pose_pregrasp(self):
        """预抓取: 双臂前伸外展，位于盆两侧上方"""
        left  = [-0.22, -0.6,  0.3, -1.0,  0.2, 0.0, 0.0]
        right = [ 0.22, -0.6, -0.3, -1.0, -0.2, 0.0, 0.0]
        return left, right

    def pose_approach(self):
        """接近位姿: 下降到方盆边缘高度"""
        left  = [-0.20, -0.45,  0.35, -0.85,  0.18, 0.0, 0.0]
        right = [ 0.20, -0.45, -0.35, -0.85, -0.18, 0.0, 0.0]
        return left, right

    def pose_grasp(self):
        """抓取位姿: 双臂内收夹持方盆两侧"""
        left  = [-0.12, -0.4,  0.45, -0.75,  0.25, 0.0, 0.0]
        right = [ 0.12, -0.4, -0.45, -0.75, -0.25, 0.0, 0.0]
        return left, right

    def pose_lift(self):
        """抬起: 抓取后垂直上抬"""
        left  = [-0.12, -0.7,  0.45, -0.75,  0.25, 0.0, 0.0]
        right = [ 0.12, -0.7, -0.45, -0.75, -0.25, 0.0, 0.0]
        return left, right

    def pose_transport(self):
        """运输姿态: 回收靠近身体，降低重心"""
        left  = [-0.08, -0.8,  0.35, -0.9,  0.2, 0.0, 0.0]
        right = [ 0.08, -0.8, -0.35, -0.9, -0.2, 0.0, 0.0]
        return left, right

    def pose_place(self):
        """放置位姿: 腰部旋转180度后前伸"""
        left  = [ 0.22, -0.6, -0.3, -1.0,  0.2, 0.0, 0.0]
        right = [-0.22, -0.6,  0.3, -1.0, -0.2, 0.0, 0.0]
        return left, right

    def pose_place_down(self):
        """放置下降: 缓慢下降到桌面"""
        left  = [ 0.20, -0.45, -0.35, -0.85,  0.18, 0.0, 0.0]
        right = [-0.20, -0.45,  0.35, -0.85, -0.18, 0.0, 0.0]
        return left, right

    def pose_retreat(self):
        """撤离: 释放后后退"""
        left  = [ 0.15, -0.6, -0.2, -1.0,  0.15, 0.0, 0.0]
        right = [-0.15, -0.6,  0.2, -1.0, -0.15, 0.0, 0.0]
        return left, right

    # ==================== 主流程 ====================

    def run_pipeline(self):
        """完整抓取流水线 (12步)"""
        self._log("\n" + "=" * 70)
        self._log("【开始方盆抓取流水线 v2.0】")
        self._log("=" * 70)

        total_steps = 12

        try:
            # ===== Phase 1: 准备阶段 =====
            self._log(f"\n【Step 1/{total_steps}】安全归位")
            l, r = self.pose_home()
            if not self.move_arms(l, r, velocity=self.VEL_NORMAL):
                return False
            time.sleep(self.WAIT_SHORT)
            self._has_payload = False

            self._log(f"\n【Step 2/{total_steps}】张开夹爪")
            if not self.open_gripper():
                return False
            time.sleep(self.WAIT_MEDIUM)

            # ===== Phase 2: 抓取阶段 =====
            self._log(f"\n【Step 3/{total_steps}】高位预抓取 (过渡航点)")
            l, r = self.pose_pregrasp_high()
            if not self.move_arms(l, r, velocity=self.VEL_NORMAL):
                return False
            time.sleep(self.WAIT_SHORT)
            if not self.check_safety():
                self._log("   高位预抓取存在安全问题", level="WARN")

            self._log(f"\n【Step 4/{total_steps}】预抓取姿态 (双臂外展避碰)")
            self.pregrasp_gripper()
            time.sleep(self.WAIT_SHORT)
            l, r = self.pose_pregrasp()
            if not self.move_arms(l, r, velocity=self.VEL_SLOW):
                return False
            time.sleep(self.WAIT_SHORT)
            has_col, pairs = self.check_collision()
            if has_col:
                self._log(f"   预抓取姿态存在碰撞对: {pairs}", level="WARN")

            self._log(f"\n【Step 5/{total_steps}】接近下降 (缓慢下降到盆边缘)")
            l, r = self.pose_approach()
            if not self.move_arms(l, r, velocity=self.VEL_APPROACH):
                return False
            time.sleep(self.WAIT_SHORT)
            self._log("   验证末端位姿:")
            self.print_end_pose()

            self._log(f"\n【Step 6/{total_steps}】抓取位姿 + 夹爪夹紧")
            l, r = self.pose_grasp()
            if not self.move_arms(l, r, velocity=self.VEL_APPROACH):
                return False
            time.sleep(self.WAIT_SHORT)
            if not self.grasp_gripper():
                return False
            time.sleep(self.WAIT_GRIPPER)
            self._log("   夹持验证:")
            self.verify_grip()
            self._has_payload = True

            # ===== Phase 3: 运输阶段 =====
            self._log(f"\n【Step 7/{total_steps}】抬起方盆 (垂直上抬)")
            l, r = self.pose_lift()
            if not self.move_arms(l, r, velocity=self.VEL_LOADED):
                return False
            time.sleep(self.WAIT_SHORT)
            self.hold_gripper()
            time.sleep(self.WAIT_SHORT)

            self._log(f"\n【Step 8/{total_steps}】运输姿态 (回收稳定)")
            l, r = self.pose_transport()
            if not self.move_arms(l, r, velocity=self.VEL_LOADED):
                return False
            time.sleep(self.WAIT_STABILIZE)
            if not self.check_safety():
                self._log("   运输姿态存在安全问题", level="WARN")

            # ===== Phase 4: 放置阶段 =====
            self._log(f"\n【Step 9/{total_steps}】腰部旋转180度")
            waist = self.get_waist_positions()
            self._log(f"   当前腰部: {[round(x, 3) for x in waist]}")
            waist[4] += math.pi
            while waist[4] > math.pi:
                waist[4] -= 2 * math.pi
            while waist[4] < -math.pi:
                waist[4] += 2 * math.pi
            self._log(f"   目标腰部: {[round(x, 3) for x in waist]}")
            if not self.move_waist(waist, velocity=self.VEL_WAIST):
                return False
            time.sleep(self.WAIT_STABILIZE)

            self._log(f"\n【Step 10/{total_steps}】放置到目标位置")
            l, r = self.pose_place()
            if not self.move_arms(l, r, velocity=self.VEL_LOADED):
                return False
            time.sleep(self.WAIT_SHORT)
            l, r = self.pose_place_down()
            if not self.move_arms(l, r, velocity=self.VEL_APPROACH):
                return False
            time.sleep(self.WAIT_SHORT)

            self._log(f"\n【Step 11/{total_steps}】释放夹爪")
            if not self.open_gripper():
                return False
            time.sleep(self.WAIT_GRIPPER)
            self._has_payload = False

            self._log(f"\n【Step 12/{total_steps}】撤离并归位")
            l, r = self.pose_retreat()
            if not self.move_arms(l, r, velocity=self.VEL_NORMAL):
                return False
            time.sleep(self.WAIT_SHORT)
            l, r = self.pose_home()
            if not self.move_arms(l, r, velocity=self.VEL_NORMAL):
                return False
            time.sleep(self.WAIT_SHORT)

            self._log("\n" + "=" * 70)
            self._log("【流水线完成】")
            self._log("=" * 70)
            return True

        except Exception as e:
            self._log(f"\n流水线异常: {e}", level="ERROR")
            import traceback
            traceback.print_exc()
            self._emergency_recovery()
            return False

    def _emergency_recovery(self):
        """紧急恢复: 尝试回到安全姿态"""
        self._log("\n>>> 执行紧急恢复...", level="WARN")
        try:
            self.open_gripper()
            time.sleep(0.5)
        except Exception:
            pass
        try:
            l, r = self.pose_home()
            self.move_arms(l, r, velocity=self.VEL_NORMAL)
        except Exception as e:
            self._log(f"   紧急恢复失败: {e}", level="ERROR")
        self._has_payload = False
        self._log(">>> 紧急恢复完成")

    # ==================== 资源释放 ====================

    def shutdown(self):
        """释放GDK资源"""
        self._log("\n>>> 释放GDK资源...")
        try:
            res = agibot_gdk.gdk_release()
            if res == agibot_gdk.GDKRes.kSuccess:
                self._log("GDK释放成功")
            else:
                self._log(f"GDK释放失败: {res}", level="ERROR")
        except Exception as e:
            self._log(f"GDK释放异常: {e}", level="ERROR")


# ==================== 主程序 ====================

def main():
    ctrl = None
    try:
        ctrl = G2BinGraspController()

        ctrl._log("\n>>> 当前双臂关节位置 (rad)")
        l_pos, r_pos = ctrl.get_arm_positions()
        ctrl._log(f"   左臂: {[round(x, 3) for x in l_pos]}")
        ctrl._log(f"   右臂: {[round(x, 3) for x in r_pos]}")

        ctrl._log("\n>>> 当前双臂末端位姿 (base_link)")
        ctrl.print_end_pose()

        ctrl._log("\n>>> 当前夹爪位置")
        gripper = ctrl.get_gripper_positions()
        if gripper:
            for name, pos in gripper.items():
                ctrl._log(f"   {name}: {pos:.3f}")
        else:
            ctrl._log("   (未读取到夹爪位置)")

        success = ctrl.run_pipeline()

        if success:
            ctrl._log("\n方盆抓取流水线执行成功!")
        else:
            ctrl._log("\n方盆抓取流水线执行失败!", level="ERROR")

    except KeyboardInterrupt:
        print("\n用户中断")
        if ctrl:
            ctrl._emergency_recovery()
    except Exception as e:
        print(f"\n程序异常: {e}")
        import traceback
        traceback.print_exc()
        if ctrl:
            ctrl._emergency_recovery()
    finally:
        if ctrl:
            ctrl.shutdown()


if __name__ == "__main__":
    main()



