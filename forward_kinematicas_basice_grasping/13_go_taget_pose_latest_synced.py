#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智元G2 双臂抓取方盆并转运到90度方向第二张桌子
GDK: 2.6.3
工况:
- 两张桌子高度均为0.76m，互成90度
- 方盆尺寸440x320x85mm，抓取上边沿
- OmniPicker左右分别下发开合指令并重复确认
- 首次无JSON时仅在末端Z处于0.78~0.88m且左右对称时自动标定；以后先恢复到JSON固定抓取高度
- 腕部joint6增加0.08rad水平修正，使夹爪更接近水平相向
- 使用实测安全腰部姿态，仅相对转动joint5；禁止猜测躯干全零姿态
- 使用腰部joint5连续旋转90度
- 任务开始前先执行4步复位到实机标定的0.816m末端高度
- 放置时分段下降，释放后先向上向外撤离，避免挤压箱子
- 每次任务结束后再次复位到同一指定高度
"""

import agibot_gdk
import time
import math
import sys
import json
import os


class G2BinGraspController:
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

    # 禁止猜测躯干安全角度。程序启动后会读取并保存已由官方方式恢复的当前腰部姿态。
    SAFE_WAIST = None
    WAIST_CONTROL_PERIOD = 0.012
    WAIST_RATE_HZ = 100.0
    MIN_SAFE_END_Z = 0.70
    GRIPPER_TYPE = "omnipicker"
    CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "g2_safe_waist_reference.json")
    CALIBRATE_ONLY = False
    AUTO_CALIBRATE_IF_MISSING = True
    WAIST_TOLERANCE = 0.03
    CALIBRATION_Z_MIN = 0.78
    CALIBRATION_Z_MAX = 0.88
    CALIBRATION_Z_SYMMETRY = 0.03

    GRIPPER_OPEN = -0.785
    GRIPPER_CLOSE = 0.0
    GRIPPER_PRE = -0.35

    # 实机夹爪略向下偏，针对腕部俯仰joint6增加约4.6度水平修正。
    # 若现场仍向下偏，可在0.02~0.10 rad范围内微调；若向上偏则减小。
    TOOL_LEVEL_CORRECTION = 0.08

    ARM_MIN = [-3.071796, -2.059505, -3.071796, -2.495838,
               -3.071796, -1.012308, -1.535907]
    ARM_MAX = [3.071796, 2.059505, 3.071796, 1.012308,
               3.071796, 1.012308, 1.535907]

    # 第二张桌子在机器人右侧时为True；在左侧改为False
    TURN_CLOCKWISE = True
    TURN_ANGLE_RAD = math.pi / 2.0
    TURN_ANGULAR_SPEED = 0.20

    # 每次任务开始前的机械臂复位。实机截图显示该姿态下arm_end_link高度约0.816m。
    # RESET_LIFT先把双臂抬到约0.995m安全高度，再进入RESET_HOME。
    RESET_END_HEIGHT = 0.816

    def __init__(self):
        print("=" * 70)
        print("【智元G2方盆抓取控制器】初始化")
        print("=" * 70)

        res = agibot_gdk.gdk_init()
        if res != agibot_gdk.GDKRes.kSuccess:
            print(f"GDK初始化失败，错误码: {res}")
            sys.exit(1)
        print("GDK初始化成功")

        self.robot = agibot_gdk.Robot()
        time.sleep(2)
        print("Robot对象就绪")

        self.tf = agibot_gdk.TF()
        time.sleep(2)
        print("TF对象就绪")

        self.pnc = agibot_gdk.Pnc()
        time.sleep(2)
        print("PNC对象就绪")
        print("末端配置: dual_tool / omnipicker / 范围[-0.785, 0.0]")

        self._initial_check()

    def _initial_check(self):
        print("\n>>> 机器人初始状态检查")
        try:
            status = self.robot.get_motion_control_status()
            collision_pairs = getattr(status, "collision_pairs", [])
            print(f"当前碰撞对数量: {len(collision_pairs)}")
        except Exception as e:
            print(f"状态检查提示: {e}")

        try:
            end_state = self.robot.get_end_state()
            print(f"左末端型号: {end_state.get('left_end_type', 'unknown')}")
            print(f"右末端型号: {end_state.get('right_end_type', 'unknown')}")
        except Exception as e:
            print(f"末端状态读取提示: {e}")

    def get_arm_positions(self):
        data = self.robot.get_joint_states()
        states = data["states"] if isinstance(data, dict) else data.states
        pos = {}
        for state in states:
            if isinstance(state, dict):
                name = state.get("name")
                value = state.get("motor_position", state.get("position", 0.0))
            else:
                name = getattr(state, "name", "")
                value = getattr(state, "motor_position",
                                getattr(state, "position", 0.0))
            pos[name] = value
        left = [pos.get(name, 0.0) for name in self.LEFT_ARM_JOINTS]
        right = [pos.get(name, 0.0) for name in self.RIGHT_ARM_JOINTS]
        return left, right

    def get_waist_positions(self):
        data = self.robot.get_joint_states()
        states = data["states"] if isinstance(data, dict) else data.states
        pos = {}
        for state in states:
            if isinstance(state, dict):
                name = state.get("name")
                value = state.get("motor_position", state.get("position", 0.0))
            else:
                name = getattr(state, "name", "")
                value = getattr(state, "motor_position", getattr(state, "position", 0.0))
            pos[name] = value
        return [pos.get(name, 0.0) for name in self.WAIST_JOINTS]

    def move_waist_smooth(self, target, duration=2.0):
        """100Hz腰部伺服，一次连续动作，不拆成多个规划Step。"""
        start = self.get_waist_positions()
        steps = max(2, int(duration * self.WAIST_RATE_HZ))
        dt = 1.0 / self.WAIST_RATE_HZ
        begin = time.time()
        try:
            for i in range(steps):
                t = float(i + 1) / steps
                current = [a + t * (b - a) for a, b in zip(start, target)]
                ret = self.robot.move_waist_joint_servo(
                    current, self.WAIST_CONTROL_PERIOD)
                if ret != 0:
                    print(f"腰部伺服失败，步数={i}, 返回={ret}")
                    return False
                expected = begin + (i + 1) * dt
                remain = expected - time.time()
                if remain > 0:
                    time.sleep(remain)
            print(f"腰部运动完成: {[round(x, 3) for x in target]}")
            return True
        except Exception as e:
            print(f"腰部运动异常: {e}")
            return False

    def get_end_z(self, link_name):
        transform = self.tf.get_tf_from_base_link(link_name)
        p = getattr(transform, "translation", None)
        if p is None:
            p = getattr(transform, "position", None)
        if p is None:
            raise RuntimeError("Transform中未找到translation/position")
        return float(p.z)

    def save_current_waist_reference(self):
        """只在末端高度处于安全标定窗口且左右接近一致时保存当前腰部姿态。"""
        left_z = self.get_end_z("arm_l_end_link")
        right_z = self.get_end_z("arm_r_end_link")
        print(f"标定检查: 左Z={left_z:.3f}m, 右Z={right_z:.3f}m")
        height_ok = (self.CALIBRATION_Z_MIN <= left_z <= self.CALIBRATION_Z_MAX and
                     self.CALIBRATION_Z_MIN <= right_z <= self.CALIBRATION_Z_MAX)
        symmetry_ok = abs(left_z - right_z) <= self.CALIBRATION_Z_SYMMETRY
        if not height_ok or not symmetry_ok:
            print("当前姿态不满足自动标定条件，程序不会保存，也不会发送运动命令。")
            print(f"要求: 两侧Z在[{self.CALIBRATION_Z_MIN:.2f}, {self.CALIBRATION_Z_MAX:.2f}]m，"
                  f"左右高度差<={self.CALIBRATION_Z_SYMMETRY:.2f}m")
            return False
        waist = self.get_waist_positions()
        payload = {
            "waist": waist,
            "left_end_z": left_z,
            "right_end_z": right_z,
            "gripper_type": self.GRIPPER_TYPE
        }
        with open(self.CALIBRATION_FILE, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        self.SAFE_WAIST = [float(x) for x in waist]
        print(f"已保存抓取高度腰部标定: {self.CALIBRATION_FILE}")
        print(f"腰部角度: {[round(x, 5) for x in waist]}")
        return True

    def load_safe_waist_reference(self):
        """读取已保存的抓取高度，禁止把运行时任意高度误当作安全目标。"""
        if not os.path.exists(self.CALIBRATION_FILE):
            print("未找到腰部高度标定文件。")
            if self.AUTO_CALIBRATE_IF_MISSING:
                print("启用首次自动标定：先验证当前末端高度，满足条件后保存并继续。")
                return self.save_current_waist_reference()
            print("自动标定已关闭，程序不会发送运动命令。")
            return False
        try:
            with open(self.CALIBRATION_FILE, "r", encoding="utf-8") as f:
                payload = json.load(f)
            waist = payload.get("waist", [])
            if len(waist) != 5:
                print("标定文件中的waist必须包含5个关节角。")
                return False
            self.SAFE_WAIST = [float(x) for x in waist]
            print(f"读取抓取高度腰部标定: {[round(x, 5) for x in self.SAFE_WAIST]}")
            return True
        except Exception as e:
            print(f"腰部标定文件读取失败: {e}")
            return False

    def verify_waist_reached(self):
        actual = self.get_waist_positions()
        errors = [abs(a - b) for a, b in zip(actual, self.SAFE_WAIST)]
        print(f"腰部到位误差: {[round(x, 4) for x in errors]}")
        return max(errors) <= self.WAIST_TOLERANCE

    def print_end_pose(self):
        """兼容GDK Transform的translation/rotation字段及部分版本的position/orientation字段。"""
        for link_name, label in [
            ("arm_l_end_link", "左臂末端"),
            ("arm_r_end_link", "右臂末端")
        ]:
            try:
                transform = self.tf.get_tf_from_base_link(link_name)
                p = getattr(transform, "translation", None)
                q = getattr(transform, "rotation", None)
                if p is None:
                    p = getattr(transform, "position", None)
                if q is None:
                    q = getattr(transform, "orientation", None)
                if p is None or q is None:
                    print(f"{label} [{link_name}] Transform字段无法识别，跳过显示")
                    continue
                print(f"{label} [{link_name}]:")
                print(f"  位置: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")
                print(f"  四元数: ({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})")
            except Exception as e:
                print(f"{label}位姿读取失败: {e}")

    def check_collision(self):
        try:
            status = self.robot.get_motion_control_status()
            pairs = getattr(status, "collision_pairs", [])
            return len(pairs) > 0, pairs
        except Exception as e:
            print(f"碰撞状态读取提示: {e}")
            return False, []

    def _clamp_arm(self, positions):
        return [max(lo, min(value, hi)) for value, lo, hi in
                zip(positions, self.ARM_MIN, self.ARM_MAX)]

    def move_arms(self, left_positions, right_positions, velocity=0.10):
        if len(left_positions) != 7 or len(right_positions) != 7:
            print("双臂目标必须各包含7个关节角")
            return False

        left = self._clamp_arm(left_positions)
        right = self._clamp_arm(right_positions)
        target = left + right
        velocities = [velocity] * 14

        print("\n>>> 双臂运动")
        print(f"左臂: {[round(x, 4) for x in left]}")
        print(f"右臂: {[round(x, 4) for x in right]}")
        try:
            # GDK 2.6.3: 第三个参数为control_group，2表示双臂
            ret = self.robot.move_arm_joint(target, velocities, 2)
            if ret == 0:
                print("双臂运动完成")
                return True
            print(f"双臂运动失败: {ret}")
            return False
        except Exception as e:
            print(f"双臂运动异常: {e}")
            return False

    def move_arms_interpolated(self, start_left, start_right,
                               end_left, end_right,
                               steps=10, velocity=0.08):
        """分段插值，降低大步关节插值导致双臂瞬时内收的风险。"""
        for i in range(1, steps + 1):
            t = i / float(steps)
            left = [a + t * (b - a) for a, b in zip(start_left, end_left)]
            right = [a + t * (b - a) for a, b in zip(start_right, end_right)]
            if not self.move_arms(left, right, velocity):
                return False
            time.sleep(0.10)
        return True

    def _control_one_omnipicker(self, group, position):
        """单独控制一侧OmniPicker，避免dual_tool在部分固件上只返回成功但末端无动作。"""
        js = agibot_gdk.JointStates()
        js.group = group
        js.target_type = "omnipicker"
        state = agibot_gdk.JointState()
        state.position = max(self.GRIPPER_OPEN, min(position, self.GRIPPER_CLOSE))
        js.states = [state]
        js.nums = 1
        ret = self.robot.move_ee_pos(js)
        return ret == 0

    def control_gripper(self, left_pos, right_pos):
        """左右OmniPicker分别下发，并重复一次确保实机执行。"""
        print(f"\n>>> OmniPicker控制: 左={left_pos:.3f}, 右={right_pos:.3f}")
        try:
            for attempt in range(2):
                left_ok = self._control_one_omnipicker("left_tool", left_pos)
                time.sleep(0.15)
                right_ok = self._control_one_omnipicker("right_tool", right_pos)
                time.sleep(0.35)
                if left_ok and right_ok:
                    print(f"OmniPicker第{attempt + 1}次指令完成")
                else:
                    print(f"OmniPicker第{attempt + 1}次指令返回失败")
                    return False
            return True
        except Exception as e:
            print(f"OmniPicker控制异常: {e}")
            return False

    def open_gripper(self):
        return self.control_gripper(self.GRIPPER_OPEN, self.GRIPPER_OPEN)

    def close_gripper(self):
        return self.control_gripper(self.GRIPPER_CLOSE, self.GRIPPER_CLOSE)

    def pregrasp_gripper(self):
        return self.control_gripper(self.GRIPPER_PRE, self.GRIPPER_PRE)

    def _apply_tool_level_correction(self, left, right):
        """joint6为腕部俯仰；左右使用同方向的小角度修正，使夹爪更接近水平。"""
        left = list(left)
        right = list(right)
        left[5] += self.TOOL_LEVEL_CORRECTION
        right[5] += self.TOOL_LEVEL_CORRECTION
        return self._clamp_arm(left), self._clamp_arm(right)

    # 实机初始姿态
    def pose_home(self):
        left = [1.5710, -1.5710, -1.5710, -1.5710, 0.0, 0.0, 0.0]
        right = [-1.5710, -1.5710, 1.5710, -1.5710, 0.0, 0.0, 0.0]
        return left, right

    # 高于盆沿约100mm，左右更加张开
    def pose_pregrasp(self):
        left = [1.0384, -0.4499, -0.8836, -1.3103,
                -1.2244, -0.0975, -1.5350]
        right = [-1.0384, -0.4499, 0.8836, -1.3103,
                 1.2244, -0.0975, 1.5350]
        return self._apply_tool_level_correction(left, right)

    # 76cm桌面上85mm盆子的上边沿，夹爪水平相向
    def pose_grasp(self):
        left = [1.0775, -0.6531, -0.7560, -1.2266,
                -1.1235, -0.0809, -1.4215]
        right = [-1.0775, -0.6531, 0.7560, -1.2266,
                 1.1235, -0.0809, 1.4215]
        return self._apply_tool_level_correction(left, right)

    # 保持夹持宽度，向上抬升约150mm
    def pose_lift(self):
        left = [1.0203, -0.4381, -0.9223, -1.5762,
                -1.1871, -0.0952, -1.3054]
        right = [-1.0203, -0.4381, 0.9223, -1.5762,
                 1.1871, -0.0952, 1.3054]
        return self._apply_tool_level_correction(left, right)

    # 第二张桌子同高，因此使用与抓取相同的放置端点
    def pose_place(self):
        return self.pose_grasp()

    def stop_chassis(self):
        twist = agibot_gdk.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        try:
            self.pnc.move_chassis(twist)
        except Exception:
            pass

    def rotate_chassis_90(self):
        direction = -1.0 if self.TURN_CLOCKWISE else 1.0
        duration = self.TURN_ANGLE_RAD / self.TURN_ANGULAR_SPEED

        twist = agibot_gdk.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = direction * self.TURN_ANGULAR_SPEED

        turn_name = "右" if self.TURN_CLOCKWISE else "左"
        print(f"\n>>> 底盘原地向{turn_name}旋转90°")
        try:
            self.pnc.request_chassis_control(1)
            time.sleep(0.20)
            start = time.time()
            while time.time() - start < duration:
                self.pnc.move_chassis(twist)
                time.sleep(0.10)
            self.stop_chassis()
            time.sleep(0.80)
            print("底盘90°旋转指令完成")
            return True
        except Exception as e:
            self.stop_chassis()
            print(f"底盘旋转失败: {e}")
            return False

    def reset_before_task(self):
        """无论启动时躯干在哪个高度，都先恢复到已标定的抓取高度。"""
        print("\n【Reset 1/4】读取固定抓取高度标定")
        if not self.load_safe_waist_reference():
            return False

        print("\n【Reset 2/4】停止底盘并完全张开OmniPicker")
        self.stop_chassis()
        if not self.open_gripper():
            return False
        time.sleep(0.50)

        print("\n【Reset 3/4】躯干恢复到已标定抓取高度")
        if not self.move_waist_smooth(self.SAFE_WAIST, duration=3.0):
            return False
        if not self.verify_waist_reached():
            print("腰部未达到标定高度，停止任务。")
            return False

        print("\n【Reset 4/4】双臂一次规划到复位姿态")
        home_left, home_right = self.pose_home()
        if not self.move_arms(home_left, home_right, velocity=0.18):
            return False
        time.sleep(0.50)
        return True

    def run_pipeline(self):
        print("\n" + "=" * 70)
        print("【开始任务前快速复位】")
        print("=" * 70)
        if not self.reset_before_task():
            return False

        home_l, home_r = self.pose_home()
        pre_l, pre_r = self.pose_pregrasp()
        grasp_l, grasp_r = self.pose_grasp()
        lift_l, lift_r = self.pose_lift()
        place_l, place_r = self.pose_place()

        print("\n【Step 1/7】一次规划到预抓取姿态")
        if not self.move_arms(pre_l, pre_r, velocity=0.25):
            return False

        print("\n【Step 2/7】一次规划到盆上边沿")
        if not self.move_arms(grasp_l, grasp_r, velocity=0.18):
            return False

        print("\n【Step 3/7】闭合左右OmniPicker并等待夹紧")
        if not self.close_gripper():
            return False
        time.sleep(1.20)

        print("\n【Step 4/7】一次规划抬升")
        if not self.move_arms(lift_l, lift_r, velocity=0.20):
            return False

        print("\n【Step 5/7】保持实测躯干姿态，仅腰部joint5连续转动90°")
        turn_target = list(self.SAFE_WAIST)
        turn_target[4] = max(-3.045599, min(3.045599,
                             self.SAFE_WAIST[4] - math.pi / 2.0))
        if not self.move_waist_smooth(turn_target, duration=2.5):
            return False

        print("\n【Step 6/7】一次规划下降到第二张76cm桌面")
        if not self.move_arms(place_l, place_r, velocity=0.16):
            return False

        print("\n【Step 7/7】释放、撤离并复位")
        if not self.open_gripper():
            return False
        time.sleep(1.20)
        # 先一次规划到宽间距高位，避免挤压盆体
        if not self.move_arms(pre_l, pre_r, velocity=0.20):
            return False
        # 回到启动时保存的实测安全腰部姿态，不使用猜测的全零姿态
        if not self.move_waist_smooth(self.SAFE_WAIST, duration=2.5):
            return False
        if not self.move_arms(home_l, home_r, velocity=0.25):
            return False

        print("\n" + "=" * 70)
        print("【流水线完成：腰部回到实测安全参考，双臂已复位】")
        print("=" * 70)
        return True

    def shutdown(self):
        print("\n>>> 停止底盘并释放GDK资源")
        self.stop_chassis()
        res = agibot_gdk.gdk_release()
        if res == agibot_gdk.GDKRes.kSuccess:
            print("GDK释放成功")
        else:
            print(f"GDK释放失败: {res}")


def main():
    ctrl = None
    try:
        ctrl = G2BinGraspController()

        print("\n>>> 当前双臂关节位置(rad)")
        left, right = ctrl.get_arm_positions()
        print(f"左臂: {[round(x, 3) for x in left]}")
        print(f"右臂: {[round(x, 3) for x in right]}")

        print("\n>>> 当前双臂末端位姿(base_link)")
        ctrl.print_end_pose()

        if ctrl.CALIBRATE_ONLY:
            print("\n【仅标定模式】只读取和保存，不发送运动命令")
            ctrl.save_current_waist_reference()
        else:
            ctrl.run_pipeline()

    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if ctrl:
            ctrl.shutdown()


if __name__ == "__main__":
    main()
