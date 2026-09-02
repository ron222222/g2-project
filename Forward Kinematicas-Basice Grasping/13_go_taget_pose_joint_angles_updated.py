#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智元G2 GDK标准API双臂协同抓取方盆
目标: 绿色塑料方盆 440×320×85mm
夹爪: Omnipicker 简易夹爪 (dual_tool)
GDK版本: 2.6.3

基于GDK官方Python接口编写，严格遵循:
- Robot.move_arm_joint() 双臂关节规划控制
- Robot.move_ee_pos() 末端执行器(夹爪)控制
- Robot.move_waist_joint() 腰部规划控制
- TF.get_tf_from_base_link() 末端位姿查询
- get_motion_control_status() 碰撞检测
"""

import agibot_gdk
import time
import math
import sys


class G2BinGraspController:
    """基于GDK标准API的方盆抓取控制器"""

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

    # Omnipicker 夹爪关节限位 (文档: [-0.785, 0], -0.785=张开, 0=闭合)
    GRIPPER_OPEN = -0.785
    GRIPPER_CLOSE = 0.0
    GRIPPER_PRE = -0.35

    # 盆尺寸 (单位: m)
    BIN_LENGTH = 0.440
    BIN_WIDTH = 0.320
    BIN_HEIGHT = 0.085

    def __init__(self):
        print("=" * 70)
        print("【智元G2方盆抓取控制器】初始化")
        print("=" * 70)

        # 1. GDK初始化 (文档要求: 必须先初始化)
        res = agibot_gdk.gdk_init()
        if res != agibot_gdk.GDKRes.kSuccess:
            print(f"❌ GDK初始化失败，错误码: {res}")
            sys.exit(1)
        print("✅ GDK初始化成功")

        # 2. 创建Robot对象 (文档要求: 初始化后等待2秒)
        self.robot = agibot_gdk.Robot()
        time.sleep(2)
        print("✅ Robot对象就绪")

        # 3. 创建TF对象 (用于查询末端位姿，替代手动URDF解析)
        self.tf = agibot_gdk.TF()
        time.sleep(2)
        print("✅ TF对象就绪")

        # 4. 上电自检
        self._initial_check()

    # ==================== 安全检测 ====================

    def _initial_check(self):
        """上电状态与碰撞自检"""
        print("\n>>> 机器人初始状态检查")
        status = self.robot.get_whole_body_status()

        checks = [
            ("左臂", status['left_arm_error']),
            ("右臂", status['right_arm_error']),
            ("腰部", status['waist_error']),
            ("头部", status['neck_error']),
            ("底盘", status['chassis_error']),
        ]
        ok = True
        for name, err in checks:
            if err != 0:
                print(f"   ⚠️ {name}错误码: {err}")
                ok = False
        if ok:
            print("   ✅ 全身状态正常")

        # 碰撞对检查
        mc = self.robot.get_motion_control_status()
        n_col = len(mc.collision_pairs_1)
        print(f"   当前碰撞对数量: {n_col}")
        if n_col > 0:
            pairs = list(zip(mc.collision_pairs_1, mc.collision_pairs_2))
            print(f"   🚨 警告: 当前存在碰撞对: {pairs}")

        # 末端执行器型号确认
        print(f"   左末端型号: {status['left_end_model']}")
        print(f"   右末端型号: {status['right_end_model']}")

    def check_collision(self):
        """实时碰撞检测"""
        mc = self.robot.get_motion_control_status()
        pairs = list(zip(mc.collision_pairs_1, mc.collision_pairs_2))
        return len(pairs) > 0, pairs

    def check_estop(self):
        """急停检测"""
        s = self.robot.get_whole_body_status()
        if s['left_arm_estop'] or s['right_arm_estop']:
            print("🚨 检测到急停状态!")
            return False
        return True

    # ==================== 状态读取 ====================

    def get_arm_positions(self):
        """获取当前双臂关节角 (使用motor_position，文档推荐)"""
        js = self.robot.get_joint_states()
        name_to_pos = {s['name']: s['motor_position'] for s in js['states']}
        left = [name_to_pos[j] for j in self.LEFT_ARM_JOINTS]
        right = [name_to_pos[j] for j in self.RIGHT_ARM_JOINTS]
        return left, right

    def get_waist_positions(self):
        """获取当前腰部关节角"""
        js = self.robot.get_joint_states()
        name_to_pos = {s['name']: s['motor_position'] for s in js['states']}
        return [name_to_pos[j] for j in self.WAIST_JOINTS]

    def print_end_pose(self):
        """打印当前左右臂末端位姿 (base_link坐标系，替代手动URDF计算)"""
        for side, link in [("左", "arm_l_end_link"), ("右", "arm_r_end_link")]:
            try:
                t = self.tf.get_tf_from_base_link(link)
                print(f"   {side}臂末端 [{link}]:")
                print(f"      位置: ({t.translation.x:.3f}, {t.translation.y:.3f}, {t.translation.z:.3f})")
                print(f"      四元数: ({t.rotation.x:.3f}, {t.rotation.y:.3f}, {t.rotation.z:.3f}, {t.rotation.w:.3f})")
            except Exception as e:
                print(f"   ⚠️ 获取{side}臂末端位姿失败: {e}")

    # ==================== 运动控制 (GDK标准API) ====================

    def move_arms(self, left_positions, right_positions, velocity=0.3):
        """
        双臂关节规划控制 (move_arm_joint)
        control_group=2 表示双臂同时控制
        执行到目标位置后接口才返回，带碰撞保护
        """
        if len(left_positions) != 7 or len(right_positions) != 7:
            raise ValueError("左右臂各需7个关节角")

        all_pos = left_positions + right_positions
        all_vel = [velocity] * 14

        print(f"\n>>> 双臂运动指令")
        print(f"   左臂: {[round(x, 3) for x in left_positions]}")
        print(f"   右臂: {[round(x, 3) for x in right_positions]}")

        try:
            ret = self.robot.move_arm_joint(all_pos, all_vel, 2)  # 2=双臂
            if ret == 0:
                print("   ✅ 双臂运动完成")
                return True
            else:
                print(f"   ❌ 运动失败，返回值: {ret}")
                return False
        except Exception as e:
            print(f"   ❌ 运动异常: {e}")
            return False

    def move_waist(self, positions, velocity=0.3):
        """腰部关节规划控制"""
        if len(positions) != 5:
            raise ValueError("腰部需5个关节角")

        # 限位保护 (文档限位)
        limits_min = [-1.082104, -0.000174, -1.919862, -0.436332, -3.045599]
        limits_max = [0.000174, 2.652900, 1.570970, 0.436332, 3.045599]

        safe_pos = []
        for i, (p, lo, hi) in enumerate(zip(positions, limits_min, limits_max)):
            if p < lo or p > hi:
                print(f"   ⚠️ 腰部关节{i+1}目标{p:.3f}超出限位[{lo:.3f}, {hi:.3f}]，已钳制")
                safe_pos.append(max(lo, min(p, hi)))
            else:
                safe_pos.append(p)

        vels = [velocity] * 5
        print(f"\n>>> 腰部运动: {[round(x, 3) for x in safe_pos]}")
        try:
            ret = self.robot.move_waist_joint(safe_pos, vels)
            if ret == 0:
                print("   ✅ 腰部运动完成")
                return True
            else:
                print(f"   ❌ 腰部运动失败: {ret}")
                return False
        except Exception as e:
            print(f"   ❌ 腰部运动异常: {e}")
            return False

    def control_gripper(self, left_pos, right_pos):
        """
        夹爪控制 (move_ee_pos)
        group="dual_tool", target_type="omnipicker"
        """
        print(f"\n>>> 夹爪控制: 左={left_pos:.3f}, 右={right_pos:.3f}")

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
                print("   ✅ 夹爪控制成功")
                return True
            else:
                print(f"   ❌ 夹爪控制失败: {ret}")
                return False
        except Exception as e:
            print(f"   ❌ 夹爪控制异常: {e}")
            return False

    def open_gripper(self):
        return self.control_gripper(self.GRIPPER_OPEN, self.GRIPPER_OPEN)

    def close_gripper(self):
        return self.control_gripper(self.GRIPPER_CLOSE, self.GRIPPER_CLOSE)

    def pregrasp_gripper(self):
        return self.control_gripper(self.GRIPPER_PRE, self.GRIPPER_PRE)

    # ================================================================
    # ==================== 姿态定义 (根据URDF优化) ====================
    # ================================================================
    #
    # 关节顺序 (左臂/右臂各7个):
    # joint1: 肩部旋转 (绕Z轴)    范围: ±3.0718 rad
    # joint2: 肩部俯仰 (绕X轴)    范围: ±2.0595 rad
    # joint3: 肩部偏摆 (绕Z轴)    范围: ±3.0718 rad  ← 控制手臂张开幅度
    # joint4: 肘部俯仰 (绕Z轴)    范围: -2.4959 ~ 1.0123 rad
    # joint5: 前臂旋转 (绕Z轴)    范围: ±3.0718 rad
    # joint6: 腕部俯仰 (绕Z轴)    范围: ±1.0123 rad
    # joint7: 腕部旋转 (绕Z轴)    范围: ±1.5359 rad
    #
    # 关键优化: joint3 从 ±0.60 减小到 ±0.25~0.30，让夹爪更靠近盆子
    # ================================================================

    def pose_home(self):
        """
        安全归位: 双臂自然下垂
        末端位置: 约 (0.45, ±0.15, 0.30)
        """
        left = [
            0.2221,   # joint1: 微内旋
            -0.4771,   # joint2: 肩部下垂
             0.6003,   # joint3: 轻微外展
            -1.2037,   # joint4: 肘部微屈
             1.3068,   # joint5: 前臂中立
             -0.0416,   # joint6: 腕部中立
             1.5350    # joint7: 腕部中立
        ]
        right = [
             -0.2221,   # joint1: 微内旋
            -0.4771,   # joint2: 肩部下垂
            -0.6003,   # joint3: 轻微外展
            -1.2037,   # joint4: 肘部微屈
             -1.3068,   # joint5: 前臂中立
             -0.0416,   # joint6: 腕部中立
             -1.5350    # joint7: 腕部中立
        ]
        return left, right

    def pose_pregrasp(self):
        """
        预抓取: 双臂前伸，夹爪在盆子上方两侧
        末端位置: 约 (0.55, ±0.10, 0.38)
        """
        left = [
            0.2221,   # joint1: 微内旋
            -0.4771,   # joint2: 肩部前倾
             0.6003,   # joint3: 大臂外展 (减小)
            -1.2037,   # joint4: 肘部弯曲
             1.3068,   # joint5: 前臂旋转
             -0.0416,   # joint6: 腕部中立
             1.5350    # joint7: 腕部中立
        ]
        right = [
             -0.2221,   # joint1: 微内旋
            -0.4771,   # joint2: 肩部前倾
            -0.6003,   # joint3: 大臂外展 (减小)
            -1.2037,   # joint4: 肘部弯曲
            -1.3068,   # joint5: 前臂旋转
             -0.0416,   # joint6: 腕部中立
             -1.5350    # joint7: 腕部中立
        ]
        return left, right

    def pose_grasp(self):
        """
        抓取位姿: 双臂内收夹持盆子
        末端位置: 约 (0.55, ±0.08, 0.13)
        """
        left = [
            0.4611,   # joint1: 左臂轻微内旋
            -0.6672,   # joint2: 肩部前倾
             0.6244,   # joint3: 大臂外展 (关键优化: 从0.60减小到0.25)
            -1.1682,   # joint4: 肘部弯曲
             1.1924,   # joint5: 前臂旋转
             -0.0233,   # joint6: 腕部中立
             1.4160    # joint7: 腕部中立
        ]
        right = [
             -0.4611,   # joint1: 右臂轻微内旋
            -0.6672,   # joint2: 肩部前倾
            -0.6244,   # joint3: 大臂外展 (关键优化: 从-0.60减小到-0.25)
            -1.1682,   # joint4: 肘部弯曲
            -1.1924,   # joint5: 前臂旋转
             -0.0233,   # joint6: 腕部中立
             -1.4160    # joint7: 腕部中立
        ]
        return left, right

    def pose_lift(self):
        """
        抬起: 抓取后上抬
        末端位置: 约 (0.55, ±0.08, 0.35)
        """
        left = [
            0.3413,   # joint1: 保持内旋
            -0.6961,   # joint2: 肩部略微抬起
             0.5671,   # joint3: 大臂外展 (保持一致)
            -1.1020,   # joint4: 肘部弯曲
             1.2121,   # joint5: 前臂旋转
             -0.0260,   # joint6: 腕部中立
             1.4346    # joint7: 腕部中立
        ]
        right = [
             -0.3413,   # joint1: 保持内旋
            -0.6961,   # joint2: 肩部略微抬起
            -0.5671,   # joint3: 大臂外展 (保持一致)
            -1.1020,   # joint4: 肘部弯曲
            -1.2121,   # joint5: 前臂旋转
             -0.0260,   # joint6: 腕部中立
             -1.4346    # joint7: 腕部中立
        ]
        return left, right

    def pose_place(self):
        """
        放置位姿: 移动到右侧工作台
        末端位置: 约 (0.60, 0.35, 0.25)
        """
        left = [
             0.4611,   # joint1: 左臂外旋转向右侧
            -0.6672,   # joint2: 肩部前倾
             0.6244,   # joint3: 大臂内收
            -1.1682,   # joint4: 肘部弯曲
             1.1924,   # joint5: 前臂旋转
             -0.0233,   # joint6: 腕部中立
             1.4160    # joint7: 腕部中立
        ]
        right = [
            -0.4611,   # joint1: 右臂内旋
            -0.6672,   # joint2: 肩部前倾
            -0.6244,   # joint3: 大臂外展
            -1.1682,   # joint4: 肘部弯曲
            -1.1924,   # joint5: 前臂旋转
             -0.0233,   # joint6: 腕部中立
             -1.4160    # joint7: 腕部中立
        ]
        return left, right

    # ==================== 主流程 ====================

    def run_pipeline(self):
        """完整抓取流水线: 抓取 -> 旋转 -> 放置"""
        print("\n" + "=" * 70)
        print("【开始方盆抓取流水线】")
        print("=" * 70)

        # Step 1: 安全归位
        print("\n【Step 1/9】安全归位")
        l, r = self.pose_home()
        if not self.move_arms(l, r, velocity=0.3):
            return False
        time.sleep(0.5)

        # Step 2: 张开夹爪
        print("\n【Step 2/9】张开夹爪")
        if not self.open_gripper():
            return False
        time.sleep(0.5)

        # Step 3: 预抓取姿态 (双臂外展避碰)
        print("\n【Step 3/9】预抓取姿态 (双臂外展)")
        l, r = self.pose_pregrasp()
        if not self.move_arms(l, r, velocity=0.2):
            return False
        time.sleep(0.3)

        # 碰撞检查
        has_col, pairs = self.check_collision()
        if has_col:
            print(f"   ⚠️ 警告: 预抓取姿态存在碰撞对: {pairs}")
            print("   请立即停止并调整 pose_pregrasp() 中的关节角!")

        # Step 4: 抓取姿态
        print("\n【Step 4/9】抓取姿态")
        l, r = self.pose_grasp()
        if not self.move_arms(l, r, velocity=0.15):
            return False
        time.sleep(0.3)

        # Step 5: 闭合夹爪
        print("\n【Step 5/9】闭合夹爪")
        if not self.close_gripper():
            return False
        time.sleep(1.0)

        # Step 6: 抬起
        print("\n【Step 6/9】抬起方盆")
        l, r = self.pose_lift()
        if not self.move_arms(l, r, velocity=0.15):
            return False
        time.sleep(0.3)

        # Step 7: 腰部旋转90°
        print("\n【Step 7/9】腰部旋转90°")
        waist = self.get_waist_positions()
        # joint5 旋转90° (π/2)
        waist[4] += math.pi / 2

        # 限位检查
        if waist[4] > 3.045599:
            print(f"   ⚠️ 目标角度 {waist[4]:.3f} 超出上限，钳制到 3.045")
            waist[4] = 3.045599
        elif waist[4] < -3.045599:
            print(f"   ⚠️ 目标角度 {waist[4]:.3f} 超出下限，钳制到 -3.045")
            waist[4] = -3.045599

        if not self.move_waist(waist, velocity=0.2):
            return False
        time.sleep(0.5)

        # Step 8: 放置姿态
        print("\n【Step 8/9】放置到右侧工作台")
        l, r = self.pose_place()
        if not self.move_arms(l, r, velocity=0.2):
            return False
        time.sleep(0.3)

        # Step 9: 释放并撤离
        print("\n【Step 9/9】释放夹爪并撤离")
        if not self.open_gripper():
            return False
        time.sleep(0.5)

        l, r = self.pose_home()
        if not self.move_arms(l, r, velocity=0.3):
            return False

        print("\n" + "=" * 70)
        print("【流水线完成】")
        print("=" * 70)
        return True

    # ==================== 资源释放 ====================

    def shutdown(self):
        """释放GDK资源"""
        print("\n>>> 释放GDK资源...")
        res = agibot_gdk.gdk_release()
        if res == agibot_gdk.GDKRes.kSuccess:
            print("✅ GDK释放成功")
        else:
            print(f"❌ GDK释放失败: {res}")


# ==================== 主程序 ====================

def main():
    ctrl = None
    try:
        ctrl = G2BinGraspController()

        # 打印当前状态供调试参考
        print("\n>>> 当前双臂关节位置 (rad)")
        l_pos, r_pos = ctrl.get_arm_positions()
        print(f"   左臂: {[round(x, 3) for x in l_pos]}")
        print(f"   右臂: {[round(x, 3) for x in r_pos]}")

        print("\n>>> 当前双臂末端位姿 (base_link)")
        ctrl.print_end_pose()

        # 执行流水线
        ctrl.run_pipeline()

    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")
    except Exception as e:
        print(f"\n❌ 程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if ctrl:
            ctrl.shutdown()


if __name__ == "__main__":
    main()