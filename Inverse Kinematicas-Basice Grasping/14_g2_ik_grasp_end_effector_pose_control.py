#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智元 G2 双臂逆运动学抓取测试程序
GDK 2.6.3, 使用 Robot.end_effector_pose_control()

重要说明：
1. 本程序从 TF 读取双臂当前末端位姿，并在 base_link 坐标系下生成相对轨迹。
2. end_effector_pose_control() 按 50 Hz 连续发送，位置和姿态均插值，避免阶跃。
3. 该接口无碰撞检测。本程序默认 DRY_RUN=True，只打印轨迹；确认后改为 False。
4. 当前版本先验证双臂 IK 运动。OmniPicker 开合保留为独立函数，调用方式需与你已验证程序一致。
"""

import math
import time
import traceback
from dataclasses import dataclass
from typing import List, Dict

import agibot_gdk

LEFT_FRAME = "arm_l_end_link"
RIGHT_FRAME = "arm_r_end_link"
RATE_HZ = 50.0
DT = 1.0 / RATE_HZ
LIFE_TIME = 0.02
MAX_TRANSLATION_STEP_M = 0.001  # 每帧最大 1 mm
DRY_RUN = True                  # 首次运行必须保持 True

# 相对动作参数，先用保守小距离验证
APPROACH_DX = 0.030             # 双手沿 base_link +X 前进 3 cm
DESCEND_DZ = -0.020             # 双手下降 2 cm
INWARD_DY = 0.010               # 左手 y 减小、右手 y 增大，各内收 1 cm
LIFT_DZ = 0.050                 # 抓紧后上抬 5 cm
RETREAT_DX = -0.030             # 后撤 3 cm


@dataclass
class PoseData:
    position: List[float]       # [x, y, z]
    orientation: List[float]    # quaternion [x, y, z, w]

    def copy(self):
        return PoseData(self.position.copy(), self.orientation.copy())


class G2IKGraspController:
    def __init__(self):
        self.robot = None
        self.tf = None
        self.initialized = False

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
        print("GDK、Robot、TF 初始化成功")

    def shutdown(self):
        if self.initialized:
            try:
                agibot_gdk.gdk_release()
            finally:
                self.initialized = False

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

        if DRY_RUN:
            print("  DRY_RUN=True，未发送运动命令")
            return

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
            time.sleep(DT)
        self.hold(goal_left, goal_right, 0.30)

    def hold(self, left: PoseData, right: PoseData, seconds: float):
        if DRY_RUN:
            return
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

    def open_grippers(self):
        print("[夹爪] 请把已验证程序中的 OmniPicker 打开函数接入这里")
        # 示例位置，不能在未确认 move_ee_pos 数据结构前盲目下发：
        # self.robot.move_ee_pos(...)

    def close_grippers(self):
        print("[夹爪] 请把已验证程序中的 OmniPicker 关闭函数接入这里")
        # self.robot.move_ee_pos(...)

    def run(self):
        left0 = self.read_pose(LEFT_FRAME)
        right0 = self.read_pose(RIGHT_FRAME)
        print("当前末端位姿（base_link）:")
        print("  左:", [round(v, 5) for v in left0.position], [round(v, 5) for v in left0.orientation])
        print("  右:", [round(v, 5) for v in right0.position], [round(v, 5) for v in right0.orientation])
        print(f"安全模式: DRY_RUN={DRY_RUN}")

        self.open_grippers()

        # 1. 前伸接近盆子
        left1 = self.offset(left0, dx=APPROACH_DX)
        right1 = self.offset(right0, dx=APPROACH_DX)
        self.move_both(left1, right1, "双臂前伸")

        # 2. 下探至盆沿
        left2 = self.offset(left1, dz=DESCEND_DZ)
        right2 = self.offset(right1, dz=DESCEND_DZ)
        self.move_both(left2, right2, "双臂下探")

        # 3. 相向内收，左臂 y 减小，右臂 y 增大
        left3 = self.offset(left2, dy=-INWARD_DY)
        right3 = self.offset(right2, dy=INWARD_DY)
        self.move_both(left3, right3, "双臂相向内收")

        self.close_grippers()

        # 4. 上抬
        left4 = self.offset(left3, dz=LIFT_DZ)
        right4 = self.offset(right3, dz=LIFT_DZ)
        self.move_both(left4, right4, "夹紧后上抬")

        # 5. 后撤
        left5 = self.offset(left4, dx=RETREAT_DX)
        right5 = self.offset(right4, dx=RETREAT_DX)
        self.move_both(left5, right5, "抬升后后撤")

        print("\n逆运动学抓取测试流程结束")
        if DRY_RUN:
            print("确认打印出的目标位姿安全后，将 DRY_RUN 改为 False 再进行真机小步测试。")


def main():
    controller = G2IKGraspController()
    try:
        controller.initialize()
        controller.run()
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as exc:
        print(f"程序异常: {exc}")
        traceback.print_exc()
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
