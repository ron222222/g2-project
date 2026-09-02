#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智元 G2 双臂逆运动学抓取测试程序（带完整力矩反馈 + 外力检测 + 自动回位）
"""

import math
import time
import traceback
import json
import os
from datetime import datetime
from dataclasses import dataclass, field
from typing import List, Dict, Optional

import agibot_gdk

# ==================== 常量定义 ====================
LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"
RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
LIFE_TIME = 0.02
MAX_TRANSLATION_STEP_M = 0.001

# 力矩相关参数
TORQUE_THRESHOLD = 5.0
EXTERNAL_FORCE_THRESHOLD = 3.0
BASELINE_SAMPLES = 30
TORQUE_CHECK_INTERVAL = 0.05
SAVE_TORQUE_LOG = True

# 动作参数
APPROACH_DX = 0.030
DESCEND_DZ = -0.020
INWARD_DY = 0.010
LIFT_DZ = 0.050
RETREAT_DX = -0.030

# 抓取检测参数
GRIP_CLOSE_TORQUE_THRESHOLD = 2.0


@dataclass
class PoseData:
    position: List[float]
    orientation: List[float]

    def copy(self):
        return PoseData(self.position.copy(), self.orientation.copy())


@dataclass
class TorqueRecord:
    timestamp: float
    step_name: str
    joint_torques: Dict[str, float]
    end_effector_wrenches: List = field(default_factory=list)


class G2IKGraspController:
    def __init__(self):
        self.robot = None
        self.tf = None
        self.initialized = False
        self.torque_records: List[TorqueRecord] = []
        self.collision_detected = False
        self.grasp_success = False
        self.external_force_detected = False

        # 存储初始位姿（用于恢复）
        self.initial_left_pose: Optional[PoseData] = None
        self.initial_right_pose: Optional[PoseData] = None

        # 力矩基线
        self.torque_baseline: Dict[str, float] = {}
        self.baseline_std: Dict[str, float] = {}
        self.baseline_ready = False

    # ==================== 初始化与关闭 ====================

    def initialize(self):
        result = agibot_gdk.gdk_init()
        if result != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError(f"GDK 初始化失败: {result}")
        self.initialized = True
        self.robot = agibot_gdk.Robot()
        self.tf = agibot_gdk.TF()
        time.sleep(2.0)

        if not self.tf.can_transform("base_link", LEFT_FRAME):
            raise RuntimeError(f"TF 不存在: base_link <- {LEFT_FRAME}")
        if not self.tf.can_transform("base_link", RIGHT_FRAME):
            raise RuntimeError(f"TF 不存在: base_link <- {RIGHT_FRAME}")

        print("✅ GDK、Robot、TF 初始化成功")

        # 记录初始位姿（用于最后恢复）
        self.initial_left_pose = self.read_pose(LEFT_FRAME)
        self.initial_right_pose = self.read_pose(RIGHT_FRAME)
        print("✅ 初始位姿已记录")

        # 建立力矩基线
        self.establish_torque_baseline()
        print("✅ 力矩传感器已就绪")

    def establish_torque_baseline(self):
        print("\n📊 正在建立力矩基线（请保持机器人静止）...")

        samples = []
        for i in range(BASELINE_SAMPLES):
            try:
                torques = self.get_joint_torques()
                samples.append(torques)
                time.sleep(0.05)
                print(f"  采样 {i + 1}/{BASELINE_SAMPLES}", end='\r')
            except Exception as e:
                print(f"\n⚠️ 采样失败: {e}")

        print("\n✅ 基线采样完成，正在计算统计量...")

        all_joints = set()
        for s in samples:
            all_joints.update(s.keys())

        self.torque_baseline = {}
        self.baseline_std = {}

        for joint in all_joints:
            values = [s.get(joint, 0.0) for s in samples]
            mean = sum(values) / len(values)
            variance = sum((v - mean) ** 2 for v in values) / len(values)
            std = math.sqrt(variance)

            self.torque_baseline[joint] = mean
            self.baseline_std[joint] = std

        self.baseline_ready = True
        print("✅ 力矩基线已建立")
        print(f"   检测到 {len(all_joints)} 个关节")
        print(f"   外力检测阈值: {EXTERNAL_FORCE_THRESHOLD} N·m")

    def shutdown(self):
        if SAVE_TORQUE_LOG and self.torque_records:
            self.save_torque_log()

        if self.initialized:
            try:
                agibot_gdk.gdk_release()
                print("✅ GDK 已释放")
            finally:
                self.initialized = False

    # ==================== 力矩获取接口 ====================

    def get_joint_torques(self) -> Dict[str, float]:
        joint_states = self.robot.get_joint_states()
        torques = {}
        for state in joint_states['states']:
            torques[state['name']] = state['effort']
        return torques

    def get_total_torque(self, torques: Dict[str, float]) -> float:
        return sum(abs(t) for t in torques.values())

    # ==================== 外力检测 ====================

    def check_external_force(self, current_torques: Dict) -> bool:
        if not self.baseline_ready:
            return False

        external_force = False
        details = []

        for joint, current_val in current_torques.items():
            if joint in self.torque_baseline:
                baseline = self.torque_baseline[joint]
                std = self.baseline_std.get(joint, 0.1)
                delta = abs(current_val - baseline)

                if delta > EXTERNAL_FORCE_THRESHOLD or delta / (std + 0.001) > 5.0:
                    external_force = True
                    details.append(f"{joint}: Δ={delta:.2f} N·m (基线={baseline:.2f}, 当前={current_val:.2f})")

        if external_force:
            print("\n⚠️  ⚠️  ⚠️  外力/碰撞检测触发！")
            print("    检测到外部推力或碰撞：")
            for d in details[:5]:
                print(f"      {d}")
            if len(details) > 5:
                print(f"      ... 还有 {len(details) - 5} 个关节异常")
            self.external_force_detected = True
            return True

        return False

    def monitor_external_force_continuous(self, duration: float = 0.5):
        if not self.baseline_ready:
            return

        start_time = time.time()
        while time.time() - start_time < duration:
            try:
                current_torques = self.get_joint_torques()
                if self.check_external_force(current_torques):
                    print("🔊 警报：检测到外力干扰！请检查机器人周围环境")
                time.sleep(TORQUE_CHECK_INTERVAL)
            except Exception as e:
                print(f"⚠️ 外力监控异常: {e}")

    # ==================== 碰撞检测 ====================

    def check_collision_during_move(self, step_name: str):
        if not hasattr(self, '_last_torques'):
            self._last_torques = self.get_joint_torques()
            return

        current_torques = self.get_joint_torques()
        collision = False

        for name, new_val in current_torques.items():
            if name in self._last_torques:
                delta = abs(new_val - self._last_torques[name])
                if delta > TORQUE_THRESHOLD:
                    print(f"⚠️  [碰撞检测] {step_name} | 关节 {name} 力矩突变 {delta:.2f} N·m")
                    collision = True

        self._last_torques = current_torques

        if collision:
            self.collision_detected = True
            raise RuntimeError(f"🛑 碰撞检测触发！在 {step_name} 执行过程中检测到异常力矩突变")

    # ==================== 抓取检测 ====================

    def detect_grasp_success(self, open_torques: Dict, close_torques: Dict) -> bool:
        gripper_joints = ['idx31_gripper_l_inner_joint1', 'idx71_gripper_r_inner_joint1']
        torque_changes = []

        for joint in gripper_joints:
            if joint in open_torques and joint in close_torques:
                delta = abs(close_torques[joint] - open_torques[joint])
                torque_changes.append(delta)
                print(f"  {joint}: 力矩变化 {delta:.3f} N·m")

        if not torque_changes:
            return False

        avg_change = sum(torque_changes) / len(torque_changes)
        success = avg_change > GRIP_CLOSE_TORQUE_THRESHOLD
        print(f"  平均力矩变化: {avg_change:.3f} N·m, 抓取{'成功 ✅' if success else '失败 ❌'}")
        return success

    # ==================== 力矩日志保存 ====================

    def record_torques(self, step_name: str):
        try:
            record = TorqueRecord(
                timestamp=time.time(),
                step_name=step_name,
                joint_torques=self.get_joint_torques(),
                end_effector_wrenches=[]
            )
            self.torque_records.append(record)
            return record
        except Exception as e:
            print(f"⚠️ 力矩记录失败: {e}")
            return None

    def save_torque_log(self):
        try:
            log_dir = os.path.expanduser("~/g2-project/torque_logs")
            os.makedirs(log_dir, exist_ok=True)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"torque_log_{timestamp}.json"
            filepath = os.path.join(log_dir, filename)

            data = {
                "timestamp": datetime.now().isoformat(),
                "grasp_success": self.grasp_success,
                "collision_detected": self.collision_detected,
                "external_force_detected": self.external_force_detected,
                "torque_baseline": self.torque_baseline,
                "records": []
            }

            for record in self.torque_records:
                data["records"].append({
                    "timestamp": record.timestamp,
                    "step_name": record.step_name,
                    "joint_torques": record.joint_torques
                })

            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)

            print(f"📁 力矩日志已保存: {filepath}")
        except Exception as e:
            print(f"⚠️ 保存力矩日志失败: {e}")

    # ==================== 运动接口 ====================

    def read_pose(self, frame: str) -> PoseData:
        t = self.tf.get_tf_from_base_link(frame)
        return PoseData(
            [t.translation.x, t.translation.y, t.translation.z],
            [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w],
        )

    @staticmethod
    def normalize_quaternion(q):
        n = math.sqrt(sum(v * v for v in q))
        if n < 1e-9:
            raise ValueError("四元数模长接近 0")
        return [v / n for v in q]

    @classmethod
    def slerp(cls, q0, q1, alpha):
        q0 = cls.normalize_quaternion(q0)
        q1 = cls.normalize_quaternion(q1)
        dot = sum(a * b for a, b in zip(q0, q1))
        if dot < 0.0:
            q1 = [-v for v in q1]
            dot = -dot
        dot = max(-1.0, min(1.0, dot))
        if dot > 0.9995:
            q = [(1.0 - alpha) * a + alpha * b for a, b in zip(q0, q1)]
            return cls.normalize_quaternion(q)
        theta0 = math.acos(dot)
        sin_theta0 = math.sin(theta0)
        theta = theta0 * alpha
        s0 = math.cos(theta) - dot * math.sin(theta) / sin_theta0
        s1 = math.sin(theta) / sin_theta0
        return [s0 * a + s1 * b for a, b in zip(q0, q1)]

    @staticmethod
    def lerp(a, b, alpha):
        return [(1.0 - alpha) * x + alpha * y for x, y in zip(a, b)]

    @staticmethod
    def distance(a, b):
        return math.sqrt(sum((y - x) ** 2 for x, y in zip(a, b)))

    def set_end_pose(self, left: PoseData, right: PoseData):
        req = agibot_gdk.EndEffectorPose()
        req.life_time = LIFE_TIME
        req.group = agibot_gdk.EndEffectorControlGroup.kBothArms

        lp = req.left_end_effector_pose.position
        lq = req.left_end_effector_pose.orientation
        rp = req.right_end_effector_pose.position
        rq = req.right_end_effector_pose.orientation

        lp.x, lp.y, lp.z = left.position
        lq.x, lq.y, lq.z, lq.w = left.orientation
        rp.x, rp.y, rp.z = right.position
        rq.x, rq.y, rq.z, rq.w = right.orientation

        result = self.robot.end_effector_pose_control(req)
        if result != 0:
            raise RuntimeError(f"end_effector_pose_control 返回失败: {result}")

    def move_both(self, goal_left: PoseData, goal_right: PoseData, label: str):
        start_left = self.read_pose(LEFT_FRAME)
        start_right = self.read_pose(RIGHT_FRAME)
        max_distance = max(
            self.distance(start_left.position, goal_left.position),
            self.distance(start_right.position, goal_right.position),
        )
        steps = max(2, int(math.ceil(max_distance / MAX_TRANSLATION_STEP_M)))
        print(f"\n[{label}] 步数={steps}, 最大位移={max_distance * 1000:.1f} mm")
        print(f"  左目标: {[round(v, 4) for v in goal_left.position]}")
        print(f"  右目标: {[round(v, 4) for v in goal_right.position]}")

        self.record_torques(f"{label}_start")
        self._last_torques = self.get_joint_torques()

        for i in range(1, steps + 1):
            a = i / steps
            left = PoseData(
                self.lerp(start_left.position, goal_left.position, a),
                self.slerp(start_left.orientation, goal_left.orientation, a),
            )
            right = PoseData(
                self.lerp(start_right.position, goal_right.position, a),
                self.slerp(start_right.orientation, goal_right.orientation, a),
            )
            self.set_end_pose(left, right)

            self.check_collision_during_move(label)
            time.sleep(DT)

        self.hold(goal_left, goal_right, 0.30)
        self.record_torques(f"{label}_end")

    def hold(self, left: PoseData, right: PoseData, seconds: float):
        cycles = max(1, int(seconds * RATE_HZ))
        for _ in range(cycles):
            self.set_end_pose(left, right)
            time.sleep(DT)

    @staticmethod
    def offset(pose: PoseData, dx=0.0, dy=0.0, dz=0.0):
        out = pose.copy()
        out.position[0] += dx
        out.position[1] += dy
        out.position[2] += dz
        return out

    # ==================== 回位功能（新增） ====================

    def move_to_initial_pose(self):
        """
        将双臂恢复到初始位置
        """
        if self.initial_left_pose is None or self.initial_right_pose is None:
            print("⚠️ 无初始位姿记录，跳过回位")
            return

        print("\n" + "=" * 70)
        print("🔄 正在将双臂恢复到初始位置...")
        print("=" * 70)

        # 先打开夹爪（安全起见）
        self.open_grippers()

        # 移动到初始位姿
        self.move_both(self.initial_left_pose, self.initial_right_pose, "恢复初始位姿")

        print("✅ 双臂已恢复到初始位置")

    # ==================== 夹爪控制 ====================

    def open_grippers(self):
        before_torques = self.get_joint_torques()
        self.record_torques("gripper_open_before")

        print("[夹爪] 正在打开 OmniPicker...")
        joint_states = agibot_gdk.JointStates()
        joint_states.group = "dual_tool"
        joint_states.target_type = "omnipicker"

        left_joint = agibot_gdk.JointState()
        left_joint.position = -0.785
        right_joint = agibot_gdk.JointState()
        right_joint.position = -0.785

        joint_states.states = [left_joint, right_joint]
        joint_states.nums = 2

        result = self.robot.move_ee_pos(joint_states)
        if result != 0:
            raise RuntimeError(f"夹爪打开失败，错误码: {result}")

        time.sleep(0.5)
        self.record_torques("gripper_open_after")
        print("✅ 夹爪已打开")

    def close_grippers(self):
        before_torques = self.get_joint_torques()
        self.record_torques("gripper_close_before")

        print("[夹爪] 正在关闭 OmniPicker...")
        joint_states = agibot_gdk.JointStates()
        joint_states.group = "dual_tool"
        joint_states.target_type = "omnipicker"

        left_joint = agibot_gdk.JointState()
        left_joint.position = 0.0
        right_joint = agibot_gdk.JointState()
        right_joint.position = 0.0

        joint_states.states = [left_joint, right_joint]
        joint_states.nums = 2

        result = self.robot.move_ee_pos(joint_states)
        if result != 0:
            raise RuntimeError(f"夹爪关闭失败，错误码: {result}")

        time.sleep(0.5)
        after_torques = self.get_joint_torques()
        self.record_torques("gripper_close_after")

        self.grasp_success = self.detect_grasp_success(before_torques, after_torques)
        if self.grasp_success:
            print("🎯 检测到成功抓取物体！")
        else:
            print("⚠️ 未检测到抓取物体")

        print("✅ 夹爪已关闭")

    # ==================== 主运行函数 ====================

    def run(self):
        left0 = self.read_pose(LEFT_FRAME)
        right0 = self.read_pose(RIGHT_FRAME)
        print("\n" + "=" * 70)
        print("当前末端位姿（base_link）:")
        print(f"  左: {[round(v, 5) for v in left0.position]} {[round(v, 5) for v in left0.orientation]}")
        print(f"  右: {[round(v, 5) for v in right0.position]} {[round(v, 5) for v in right0.orientation]}")
        print("=" * 70)
        print("\n🚀 开始执行带力矩反馈的抓取动作序列...")
        print(f"⚠️  运动碰撞阈值: {TORQUE_THRESHOLD} N·m")
        print(f"⚠️  外力检测阈值: {EXTERNAL_FORCE_THRESHOLD} N·m")
        print(f"⚠️  抓取检测阈值: {GRIP_CLOSE_TORQUE_THRESHOLD} N·m")
        print("=" * 70)

        self.record_torques("initial")

        # 监控外力
        print("\n🔍 正在监控外部推力（3秒内请推机器人测试报警）...")
        self.monitor_external_force_continuous(3.0)

        self.open_grippers()

        left1 = self.offset(left0, dx=APPROACH_DX)
        right1 = self.offset(right0, dx=APPROACH_DX)
        self.move_both(left1, right1, "双臂前伸")

        left2 = self.offset(left1, dz=DESCEND_DZ)
        right2 = self.offset(right1, dz=DESCEND_DZ)
        self.move_both(left2, right2, "双臂下探")

        left3 = self.offset(left2, dy=-INWARD_DY)
        right3 = self.offset(right2, dy=INWARD_DY)
        self.move_both(left3, right3, "双臂相向内收")

        self.close_grippers()

        left4 = self.offset(left3, dz=LIFT_DZ)
        right4 = self.offset(right3, dz=LIFT_DZ)
        self.move_both(left4, right4, "夹紧后上抬")

        left5 = self.offset(left4, dx=RETREAT_DX)
        right5 = self.offset(right4, dx=RETREAT_DX)
        self.move_both(left5, right5, "抬升后后撤")

        # ========== 新增：执行回位 ==========
        self.move_to_initial_pose()

        print("\n" + "=" * 70)
        print("✅ 逆运动学抓取测试流程结束（已自动回位）")
        print(f"📊 抓取状态: {'成功 🎯' if self.grasp_success else '失败 ❌'}")
        print(f"📊 碰撞检测: {'触发 ⚠️' if self.collision_detected else '正常 ✅'}")
        print(f"📊 外力检测: {'检测到外力 ⚠️' if self.external_force_detected else '正常 ✅'}")
        print(f"📊 力矩记录数: {len(self.torque_records)}")
        print("=" * 70)


def main():
    controller = G2IKGraspController()
    try:
        controller.initialize()
        controller.run()
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")
    except RuntimeError as e:
        if "碰撞" in str(e):
            print(f"\n🛑 {e}")
            # 发生碰撞时也尝试回位
            try:
                print("\n🔄 尝试恢复到初始位置...")
                controller.move_to_initial_pose()
            except:
                pass
        else:
            print(f"\n❌ 运行时错误: {e}")
            traceback.print_exc()
    except Exception as exc:
        print(f"❌ 程序异常: {exc}")
        traceback.print_exc()
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()