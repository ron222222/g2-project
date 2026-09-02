#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智元G2双臂OmniPicker抓盆完整程序

动作：
1. 严格闭环恢复到已标定腰部抓取高度
2. 双臂进入预抓取姿态
3. 底盘前进0.50m
4. 双臂到盆沿并夹紧
5. 抬升后底盘后退0.50m
6. 腰部joint5相对旋转90度
7. 放置、松开、撤离并复位

工况：
- 桌面高0.76m
- 盆440x320x85mm，抓取上边沿
- 左右末端抓取间距在上一版基础上总共加宽约2cm
- OmniPicker位置范围[-0.785, 0]，-0.785打开，0关闭
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

    GRIPPER_OPEN = -0.785
    GRIPPER_CLOSE = 0.0
    GRIPPER_PRE = -0.35
    TOOL_LEVEL_CORRECTION = 0.08

    # 抓取时左右各再向外约1cm，对joint3做小幅对称外展修正。
    # 按约0.314m有效臂长进行一阶估算，左右joint3各修正0.032rad，目标约每侧外移1cm。
    GRASP_WIDEN_JOINT3 = 0.032

    ARM_MIN = [-3.071796, -2.059505, -3.071796, -2.495838,
               -3.071796, -1.012308, -1.535907]
    ARM_MAX = [3.071796, 2.059505, 3.071796, 1.012308,
               3.071796, 1.012308, 1.535907]

    WAIST_MIN = [-1.082104, -0.000174, -1.919862, -0.436332, -3.045599]
    WAIST_MAX = [0.000174, 2.652900, 1.570970, 0.436332, 3.045599]
    WAIST_TOLERANCE = 0.03
    WAIST_CONTROL_PERIOD = 0.012
    WAIST_RATE_HZ = 100.0
    WAIST_MAX_ATTEMPTS = 4

    CALIBRATION_FILE = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "g2_safe_waist_reference.json"
    )
    AUTO_CALIBRATE_IF_MISSING = True
    CALIBRATE_ONLY = False
    CALIBRATION_Z_MIN = 0.78
    CALIBRATION_Z_MAX = 0.88
    CALIBRATION_Z_SYMMETRY = 0.03

    CHASSIS_FORWARD_M = 0.50
    CHASSIS_BACKWARD_M = -0.50
    CHASSIS_MIN_TRAVEL_M = 0.45
    CHASSIS_MAX_TRAVEL_M = 0.56
    CHASSIS_MAX_LATERAL_M = 0.10
    CHASSIS_TIMEOUT_S = 20.0

    # 第二张桌子在右侧使用-90度；在左侧改为+math.pi/2
    WAIST_TURN_DELTA = -math.pi / 2.0

    def __init__(self):
        print("=" * 72)
        print("【智元G2 OmniPicker双臂抓盆控制器】初始化")
        print("=" * 72)

        res = agibot_gdk.gdk_init()
        if res != agibot_gdk.GDKRes.kSuccess:
            print(f"GDK初始化失败: {res}")
            sys.exit(1)

        self.robot = agibot_gdk.Robot()
        time.sleep(2)
        self.tf = agibot_gdk.TF()
        time.sleep(2)
        self.pnc = agibot_gdk.Pnc()
        time.sleep(2)
        self.slam = agibot_gdk.Slam()
        time.sleep(2)

        self.safe_waist = None
        print("GDK、Robot、TF、PNC、SLAM对象就绪")
        print("末端配置: left_tool/right_tool + omnipicker")

    @staticmethod
    def _states_from_data(data):
        return data["states"] if isinstance(data, dict) else data.states

    @staticmethod
    def _state_name_value(state):
        if isinstance(state, dict):
            return state.get("name"), state.get(
                "motor_position", state.get("position", 0.0))
        return (
            getattr(state, "name", ""),
            getattr(state, "motor_position", getattr(state, "position", 0.0))
        )

    def _joint_map(self):
        data = self.robot.get_joint_states()
        result = {}
        for state in self._states_from_data(data):
            name, value = self._state_name_value(state)
            result[name] = float(value)
        return result

    def get_arm_positions(self):
        values = self._joint_map()
        return (
            [values.get(name, 0.0) for name in self.LEFT_ARM_JOINTS],
            [values.get(name, 0.0) for name in self.RIGHT_ARM_JOINTS]
        )

    def get_waist_positions(self):
        values = self._joint_map()
        return [values.get(name, 0.0) for name in self.WAIST_JOINTS]

    def get_end_z(self, link_name):
        transform = self.tf.get_tf_from_base_link(link_name)
        p = getattr(transform, "translation", None)
        if p is None:
            p = getattr(transform, "position", None)
        if p is None:
            raise RuntimeError("Transform中未找到translation/position")
        return float(p.z)

    def print_end_pose(self):
        for link, label in [
            ("arm_l_end_link", "左末端"),
            ("arm_r_end_link", "右末端")
        ]:
            try:
                transform = self.tf.get_tf_from_base_link(link)
                p = getattr(transform, "translation", None)
                q = getattr(transform, "rotation", None)
                if p is None:
                    p = getattr(transform, "position", None)
                if q is None:
                    q = getattr(transform, "orientation", None)
                print(f"{label}: position=({p.x:.3f},{p.y:.3f},{p.z:.3f}), "
                      f"q=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})")
            except Exception as e:
                print(f"{label}位姿读取失败: {e}")

    def _clamp_arm(self, positions):
        return [max(lo, min(v, hi)) for v, lo, hi in
                zip(positions, self.ARM_MIN, self.ARM_MAX)]

    def _clamp_waist(self, positions):
        return [max(lo, min(v, hi)) for v, lo, hi in
                zip(positions, self.WAIST_MIN, self.WAIST_MAX)]

    def move_arms(self, left, right, velocity=0.15):
        left = self._clamp_arm(left)
        right = self._clamp_arm(right)
        target = left + right
        velocities = [velocity] * 14
        try:
            ret = self.robot.move_arm_joint(target, velocities, 2)
            if ret == 0:
                return True
            print(f"双臂运动返回失败: {ret}")
            return False
        except Exception as e:
            print(f"双臂运动异常: {e}")
            return False

    def move_waist_smooth(self, target, duration=3.0):
        target = self._clamp_waist(target)
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
                    print(f"腰部伺服失败: step={i}, ret={ret}")
                    return False
                remaining = begin + (i + 1) * dt - time.time()
                if remaining > 0:
                    time.sleep(remaining)
            return True
        except Exception as e:
            print(f"腰部运动异常: {e}")
            return False

    def move_waist_to_exact_target(self, target):
        """保持0.03rad严格容差，多轮闭环，不靠重启继续逼近。"""
        previous_error = None
        for attempt in range(1, self.WAIST_MAX_ATTEMPTS + 1):
            actual = self.get_waist_positions()
            errors = [abs(a - b) for a, b in zip(actual, target)]
            max_error = max(errors)
            print(f"腰部闭环{attempt}/{self.WAIST_MAX_ATTEMPTS}: "
                  f"{[round(x, 4) for x in errors]}")
            if max_error <= self.WAIST_TOLERANCE:
                print("腰部已严格到达固定抓取高度")
                return True

            duration = max(2.5, min(5.0, 1.5 + max_error / 0.10))
            if not self.move_waist_smooth(target, duration):
                return False
            time.sleep(0.8)

            after = self.get_waist_positions()
            after_errors = [abs(a - b) for a, b in zip(after, target)]
            after_max = max(after_errors)
            if after_max <= self.WAIST_TOLERANCE:
                print("腰部已严格到达固定抓取高度")
                return True
            if previous_error is not None and after_max >= previous_error - 0.005:
                print("腰部误差未继续收敛，停止任务，不放宽容差")
                return False
            previous_error = after_max

        print("腰部未在闭环轮数内达到固定抓取高度")
        return False

    def save_current_waist_reference(self):
        left_z = self.get_end_z("arm_l_end_link")
        right_z = self.get_end_z("arm_r_end_link")
        height_ok = (
            self.CALIBRATION_Z_MIN <= left_z <= self.CALIBRATION_Z_MAX and
            self.CALIBRATION_Z_MIN <= right_z <= self.CALIBRATION_Z_MAX
        )
        symmetry_ok = abs(left_z - right_z) <= self.CALIBRATION_Z_SYMMETRY
        if not height_ok or not symmetry_ok:
            print(f"当前姿态不满足标定条件: 左Z={left_z:.3f}, 右Z={right_z:.3f}")
            return False
        waist = self.get_waist_positions()
        data = {
            "waist": waist,
            "left_end_z": left_z,
            "right_end_z": right_z,
            "gripper_type": "omnipicker"
        }
        with open(self.CALIBRATION_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        self.safe_waist = waist
        print(f"已保存腰部标定: {[round(x, 5) for x in waist]}")
        return True

    def load_waist_reference(self):
        if not os.path.exists(self.CALIBRATION_FILE):
            if self.AUTO_CALIBRATE_IF_MISSING:
                print("未找到标定文件，尝试基于当前安全姿态首次标定")
                return self.save_current_waist_reference()
            return False
        try:
            with open(self.CALIBRATION_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            waist = data.get("waist", [])
            if len(waist) != 5:
                return False
            self.safe_waist = [float(x) for x in waist]
            print(f"读取固定抓取高度: {[round(x, 5) for x in self.safe_waist]}")
            return True
        except Exception as e:
            print(f"读取标定文件失败: {e}")
            return False

    def _control_one_omnipicker(self, group, position):
        js = agibot_gdk.JointStates()
        js.group = group
        js.target_type = "omnipicker"
        state = agibot_gdk.JointState()
        state.position = max(self.GRIPPER_OPEN, min(position, self.GRIPPER_CLOSE))
        js.states = [state]
        js.nums = 1
        return self.robot.move_ee_pos(js) == 0

    def control_gripper(self, left_pos, right_pos):
        try:
            for attempt in range(2):
                if not self._control_one_omnipicker("left_tool", left_pos):
                    return False
                time.sleep(0.15)
                if not self._control_one_omnipicker("right_tool", right_pos):
                    return False
                time.sleep(0.35)
                print(f"OmniPicker第{attempt + 1}次指令完成")
            return True
        except Exception as e:
            print(f"OmniPicker控制异常: {e}")
            return False

    def open_gripper(self):
        return self.control_gripper(self.GRIPPER_OPEN, self.GRIPPER_OPEN)

    def close_gripper(self):
        return self.control_gripper(self.GRIPPER_CLOSE, self.GRIPPER_CLOSE)

    def _apply_tool_level(self, left, right):
        left = list(left)
        right = list(right)
        left[5] += self.TOOL_LEVEL_CORRECTION
        right[5] += self.TOOL_LEVEL_CORRECTION
        return self._clamp_arm(left), self._clamp_arm(right)

    def _apply_grasp_widen(self, left, right):
        left = list(left)
        right = list(right)
        left[2] -= self.GRASP_WIDEN_JOINT3
        right[2] += self.GRASP_WIDEN_JOINT3
        return self._clamp_arm(left), self._clamp_arm(right)

    def pose_home(self):
        return (
            [1.5710, -1.5710, -1.5710, -1.5710, 0.0, 0.0, 0.0],
            [-1.5710, -1.5710, 1.5710, -1.5710, 0.0, 0.0, 0.0]
        )

    def pose_pregrasp(self):
        left = [1.0384, -0.4499, -0.8836, -1.3103, -1.2244, -0.0975, -1.5350]
        right = [-1.0384, -0.4499, 0.8836, -1.3103, 1.2244, -0.0975, 1.5350]
        left, right = self._apply_tool_level(left, right)
        return self._apply_grasp_widen(left, right)

    def pose_grasp(self):
        left = [1.0775, -0.6531, -0.7560, -1.2266, -1.1235, -0.0809, -1.4215]
        right = [-1.0775, -0.6531, 0.7560, -1.2266, 1.1235, -0.0809, 1.4215]
        left, right = self._apply_tool_level(left, right)
        return self._apply_grasp_widen(left, right)

    def pose_lift(self):
        left = [1.0203, -0.4381, -0.9223, -1.5762, -1.1871, -0.0952, -1.3054]
        right = [-1.0203, -0.4381, 0.9223, -1.5762, 1.1871, -0.0952, 1.3054]
        left, right = self._apply_tool_level(left, right)
        return self._apply_grasp_widen(left, right)

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

    def move_chassis_relative(self, distance_m):
        """
        正数前进、负数后退。使用base_link相对移动，里程计验证纵向距离、
        横向漂移和打滑。注意：SDK说明relative_move前需在G02 Pad完成重定位。
        """
        direction = "前进" if distance_m > 0 else "后退"
        try:
            start_odom = self.slam.get_odom_info()
            sx = float(start_odom.pose.position.x)
            sy = float(start_odom.pose.position.y)
            qz = float(start_odom.pose.orientation.z)
            qw = float(start_odom.pose.orientation.w)
            start_yaw = 2.0 * math.atan2(qz, qw)

            target = agibot_gdk.NaviReq()
            target.target.position.x = float(distance_m)
            target.target.position.y = 0.0
            target.target.position.z = 0.0
            target.target.orientation.x = 0.0
            target.target.orientation.y = 0.0
            target.target.orientation.z = 0.0
            target.target.orientation.w = 1.0

            print(f"底盘{direction}{abs(distance_m):.2f}m")
            self.pnc.relative_move(target)

            begin = time.time()
            motion_seen = False
            stationary_count = 0
            while time.time() - begin < self.CHASSIS_TIMEOUT_S:
                odom = self.slam.get_odom_info()
                dx_world = float(odom.pose.position.x) - sx
                dy_world = float(odom.pose.position.y) - sy
                longitudinal = (math.cos(start_yaw) * dx_world +
                                math.sin(start_yaw) * dy_world)
                lateral = (-math.sin(start_yaw) * dx_world +
                           math.cos(start_yaw) * dy_world)
                signed_progress = longitudinal if distance_m > 0 else -longitudinal

                if bool(getattr(odom, "is_sliping", False)):
                    print("检测到底盘打滑，停止任务")
                    self.stop_chassis()
                    return False

                if not bool(odom.is_stationary):
                    motion_seen = True
                    stationary_count = 0
                elif motion_seen:
                    stationary_count += 1

                if abs(lateral) > self.CHASSIS_MAX_LATERAL_M:
                    print(f"横向漂移过大: {lateral:.3f}m，停止任务")
                    self.stop_chassis()
                    return False

                if signed_progress > self.CHASSIS_MAX_TRAVEL_M:
                    print(f"底盘移动超过上限: {signed_progress:.3f}m，停止任务")
                    self.stop_chassis()
                    return False

                if stationary_count >= 5:
                    print(f"底盘{direction}停止，纵向={signed_progress:.3f}m，"
                          f"横向={lateral:.3f}m")
                    if signed_progress < self.CHASSIS_MIN_TRAVEL_M:
                        print("实际纵向位移不足0.45m，停止任务")
                        return False
                    return True
                time.sleep(0.10)

            print(f"底盘{direction}等待超时")
            self.stop_chassis()
            return False
        except Exception as e:
            self.stop_chassis()
            print(f"底盘{direction}异常: {e}")
            return False

    def ensure_no_reported_collision(self, stage):
        try:
            status = self.robot.get_motion_control_status()
            pairs = getattr(status, "collision_pairs", [])
            if pairs:
                print(f"{stage}检测到碰撞对: {pairs}，停止任务")
                return False
            return True
        except Exception as e:
            print(f"{stage}碰撞状态读取失败: {e}，停止任务")
            return False

    def reset_before_task(self):
        print("\n【Reset 1/4】读取固定抓取高度")
        if not self.load_waist_reference():
            return False

        print("\n【Reset 2/4】停止底盘并张开OmniPicker")
        self.stop_chassis()
        if not self.open_gripper():
            return False

        print("\n【Reset 3/4】严格闭环恢复躯干抓取高度")
        if not self.move_waist_to_exact_target(self.safe_waist):
            return False

        print("\n【Reset 4/4】双臂归位")
        left, right = self.pose_home()
        return self.move_arms(left, right, velocity=0.18)

    def run_pipeline(self):
        if not self.reset_before_task():
            return False

        home_l, home_r = self.pose_home()
        pre_l, pre_r = self.pose_pregrasp()
        grasp_l, grasp_r = self.pose_grasp()
        lift_l, lift_r = self.pose_lift()
        place_l, place_r = self.pose_place()

        print("\n【Step 1/9】双臂到加宽后的预抓取姿态")
        if not self.move_arms(pre_l, pre_r, velocity=0.20):
            return False

        print("\n【Step 2/9】保持预抓取姿态，底盘前进0.50m")
        if not self.move_chassis_relative(self.CHASSIS_FORWARD_M):
            return False
        time.sleep(0.5)

        print("\n【Step 3/9】双臂到加宽后的盆沿抓取姿态")
        if not self.move_arms(grasp_l, grasp_r, velocity=0.15):
            return False

        print("\n【Step 4/9】闭合左右OmniPicker")
        if not self.close_gripper():
            return False
        time.sleep(1.2)

        print("\n【Step 5/9】保持加宽间距并抬升")
        if not self.move_arms(lift_l, lift_r, velocity=0.16):
            return False

        print("\n【Step 6/9】保持夹紧和抬升，底盘后退0.50m")
        if not self.move_chassis_relative(self.CHASSIS_BACKWARD_M):
            return False
        time.sleep(0.5)

        print("\n【Step 7/9】保持躯干高度，腰部joint5旋转90度")
        turn_target = list(self.safe_waist)
        turn_target[4] = max(
            self.WAIST_MIN[4],
            min(self.WAIST_MAX[4], self.safe_waist[4] + self.WAIST_TURN_DELTA)
        )
        if not self.move_waist_smooth(turn_target, duration=2.5):
            return False

        print("\n【Step 8/9】下降到第二张桌面")
        if not self.move_arms(place_l, place_r, velocity=0.14):
            return False

        print("\n【Step 9/9】松开、撤离并复位")
        if not self.open_gripper():
            return False
        time.sleep(1.2)
        if not self.move_arms(pre_l, pre_r, velocity=0.18):
            return False
        if not self.move_waist_to_exact_target(self.safe_waist):
            return False
        if not self.move_arms(home_l, home_r, velocity=0.18):
            return False

        print("\n【完整流程完成】")
        return True

    def shutdown(self):
        self.stop_chassis()
        result = agibot_gdk.gdk_release()
        print("GDK释放成功" if result == agibot_gdk.GDKRes.kSuccess else
              f"GDK释放失败: {result}")


def main():
    controller = None
    try:
        controller = G2BinGraspController()
        print("\n当前双臂末端位姿:")
        controller.print_end_pose()

        if controller.CALIBRATE_ONLY:
            print("\n【仅标定模式】不会发送运动指令")
            controller.save_current_waist_reference()
        else:
            controller.run_pipeline()
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if controller is not None:
            controller.shutdown()


if __name__ == "__main__":
    main()
