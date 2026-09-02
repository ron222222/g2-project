import agibot_gdk
import time
import math


def main():
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("❌ GDK 初始化失败")
        return
    print("✅ GDK 初始化成功")

    robot = agibot_gdk.Robot()
    time.sleep(2)
    print("✅ Robot 对象创建成功")

    # ========== 全部 22 个关节定义 ==========
    WAIST_JOINTS = [
        "idx01_body_joint1", "idx02_body_joint2", "idx03_body_joint3",
        "idx04_body_joint4", "idx05_body_joint5"
    ]
    HEAD_JOINTS = [
        "idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3"
    ]
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

    ALL_JOINTS = WAIST_JOINTS + HEAD_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

    JOINT_LIMITS = {
        "idx01_body_joint1": (-1.082104, 0.000174),
        "idx02_body_joint2": (-0.000174, 2.652900),
        "idx03_body_joint3": (-1.919862, 1.570970),
        "idx04_body_joint4": (-0.436332, 0.436332),
        "idx05_body_joint5": (-3.045599, 3.045599),
        "idx11_head_joint1": (-1.570970, 1.570970),
        "idx12_head_joint2": (-0.349240, 0.349240),
        "idx13_head_joint3": (-0.534773, 0.534773),
        "idx21_arm_l_joint1": (-3.071796, 3.071796),
        "idx22_arm_l_joint2": (-2.059505, 2.059505),
        "idx23_arm_l_joint3": (-3.071796, 3.071796),
        "idx24_arm_l_joint4": (-2.495838, 1.012308),
        "idx25_arm_l_joint5": (-3.071796, 3.071796),
        "idx26_arm_l_joint6": (-1.012308, 1.012308),
        "idx27_arm_l_joint7": (-1.535907, 1.535907),
        "idx61_arm_r_joint1": (-3.071796, 3.071796),
        "idx62_arm_r_joint2": (-2.059505, 2.059505),
        "idx63_arm_r_joint3": (-3.071796, 3.071796),
        "idx64_arm_r_joint4": (-2.495838, 1.012308),
        "idx65_arm_r_joint5": (-3.071796, 3.071796),
        "idx66_arm_r_joint6": (-1.012308, 1.012308),
        "idx67_arm_r_joint7": (-1.535907, 1.535907),
    }

    def check_limits(joint_names, positions):
        """检查关节是否超出限位"""
        for name, pos in zip(joint_names, positions):
            min_v, max_v = JOINT_LIMITS[name]
            if pos < min_v or pos > max_v:
                print(f"  ⚠️ 关节 {name} 超出限位: {pos:.3f} 不在 [{min_v:.3f}, {max_v:.3f}]")
                return False
        return True

    def move_all_joints(waist, head, left_arm, right_arm, velocity=0.3, lifetime=3.0):
        """控制全部 22 个关节"""
        all_positions = list(waist) + list(head) + list(left_arm) + list(right_arm)

        if not check_limits(ALL_JOINTS, all_positions):
            print("  ❌ 限位检查失败，跳过此动作")
            return False

        req = agibot_gdk.JointControlReq()
        req.joint_names = ALL_JOINTS
        req.joint_positions = all_positions
        req.joint_velocities = [velocity] * 22
        req.life_time = lifetime
        req.detail = "全身22关节运动"

        try:
            result = robot.joint_control_request(req)
            if result == 0:
                print("  ✅ 动作发送成功")
                return True
            else:
                print(f"  ⚠️ 动作返回码: {result}")
                return False
        except Exception as e:
            print(f"  ❌ 动作失败: {e}")
            return False

    def control_gripper(side, open_ratio):
        """控制夹爪: side='left'/'right', open_ratio=0.0~1.0"""
        joint_states = agibot_gdk.JointStates()
        joint_states.group = f"{side}_tool"
        joint_states.target_type = "omnipicker"
        pos = -0.785 * max(0.0, min(1.0, open_ratio))
        js = agibot_gdk.JointState()
        js.position = pos
        joint_states.states = [js]
        joint_states.nums = 1

        try:
            robot.move_ee_pos(joint_states)
            state = "打开" if open_ratio > 0.5 else "关闭"
            print(f"  🤏 {side}夹爪{state} (位置: {pos:.3f})")
            return True
        except Exception as e:
            print(f"  ❌ {side}夹爪控制失败: {e}")
            return False

    def control_both_grippers(left_open, right_open):
        """同时控制两个夹爪"""
        control_gripper("left", left_open)
        time.sleep(0.3)
        control_gripper("right", right_open)
        time.sleep(0.3)

    # ========== 13 个姿态定义（全部修复重心+自碰撞） ==========
    # 核心原则：
    # 1. 腰部前倾/后仰时，手臂必须后摆或侧平举（不能前伸）
    # 2. 双臂上举时，joint1 间距 ≥ 1.5，且手臂内收
    # 3. idx01_body_joint1 始终 ≤ 0，idx02_body_joint2 始终 ≥ 0

    # 【姿态0】全身归零
    pose_home = {
        "waist": [0.0, 0.0, 0.0, 0.0, 0.0],
        "head": [0.0, 0.0, 0.0],
        "left_arm": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "right_arm": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "left_gripper": 0.0,
        "right_gripper": 0.0
    }

    # 【姿态1】双臂侧平举(T字形) + 头部左看 + 夹爪打开
    # 安全：腰部直立，手臂侧平举，重心在支撑面中心
    pose_t_pose = {
        "waist": [0.0, 0.0, 0.0, 0.0, 0.0],
        "head": [0.5, 0.0, 0.0],
        "left_arm": [1.57, 0.0, 0.0, -0.3, 0.0, 0.0, 0.0],
        "right_arm": [-1.57, 0.0, 0.0, -0.3, 0.0, 0.0, 0.0],
        "left_gripper": 1.0,
        "right_gripper": 1.0
    }

    # 【姿态2】双臂前伸（腰部必须直立！）+ 夹爪打开
    # 修复：腰部直立(0)，手臂前伸但幅度减小，避免重心前移
    pose_front_reach = {
        "waist": [0.0, 0.0, 0.0, 0.0, 0.0],  # 腰部严格直立
        "head": [0.0, 0.1, 0.0],
        "left_arm": [0.2, -0.6, 0.3, -0.9, 0.1, 0.3, 0.2],
        "right_arm": [-0.2, -0.6, -0.3, -0.9, -0.1, -0.3, -0.2],
        "left_gripper": 1.0,
        "right_gripper": 1.0
    }

    # 【姿态3】双臂上举（修复自碰撞：大间距+内收）+ 夹爪打开
    # 修复：joint1 间距 1.6，joint2 内收(-1.2)，link5 不外扩
    pose_wave_up = {
        "waist": [0.0, 0.0, 0.0, 0.0, 0.0],
        "head": [0.0, -0.15, 0.0],
        "left_arm": [1.6, -1.2, 0.2, -0.5, 0.0, 0.3, 0.0],
        "right_arm": [-1.6, -1.2, -0.2, -0.5, 0.0, -0.3, 0.0],
        "left_gripper": 1.0,
        "right_gripper": 1.0
    }

    # 【姿态4】双臂抱圆 + 腰部微侧弯 + 夹爪关闭
    # 安全：手臂在身前抱圆，重心居中
    pose_circle = {
        "waist": [0.0, 0.0, 0.15, 0.0, 0.0],
        "head": [0.0, 0.1, 0.0],
        "left_arm": [0.4, -0.4, 0.5, -1.5, 0.2, 0.3, 0.1],
        "right_arm": [-0.4, -0.4, -0.5, -1.5, -0.2, -0.3, -0.1],
        "left_gripper": 0.0,
        "right_gripper": 0.0
    }

    # 【姿态5】左臂高举+右臂低摆 + 左开右闭
    # 安全：单臂上举，另一臂下垂，重心偏移小
    pose_asymmetric = {
        "waist": [0.0, 0.0, 0.0, 0.0, -0.3],
        "head": [-0.2, -0.1, 0.0],
        "left_arm": [0.0, -1.4, 0.3, -0.3, 0.4, 0.2, 0.2],
        "right_arm": [-0.3, 0.2, -0.3, -0.8, -0.2, -0.2, -0.1],
        "left_gripper": 1.0,
        "right_gripper": 0.0
    }

    # 【姿态6】双臂后伸扩胸 + 夹爪打开
    # 安全：手臂后伸，重心后移但在支撑面内
    pose_expand_chest = {
        "waist": [0.0, 0.0, 0.0, 0.0, 0.0],
        "head": [0.0, -0.05, 0.0],
        "left_arm": [0.5, 0.3, 0.0, -0.2, 0.0, 0.0, 0.0],
        "right_arm": [-0.5, 0.3, 0.0, -0.2, 0.0, 0.0, 0.0],
        "left_gripper": 1.0,
        "right_gripper": 1.0
    }

    # 【姿态7】双臂内旋拧转 + 夹爪关闭
    # 安全：手臂在身前，joint3 内旋，不扩展
    pose_twist = {
        "waist": [0.0, 0.0, 0.0, 0.0, 0.0],
        "head": [0.0, 0.05, 0.0],
        "left_arm": [0.1, -0.2, 0.8, -0.8, 0.3, 0.5, 0.3],
        "right_arm": [-0.1, -0.2, -0.8, -0.8, -0.3, -0.5, -0.3],
        "left_gripper": 0.0,
        "right_gripper": 0.0
    }

    # 【姿态8】全身舒展（腰部微前倾+手臂上举外展）
    # 修复：腰部前倾仅0.1，手臂上举但joint1间距大(1.5)，避免碰撞
    pose_stretch = {
        "waist": [0.0, 0.1, 0.0, 0.0, 0.0],
        "head": [0.0, -0.1, 0.0],
        "left_arm": [1.5, -1.3, 0.1, -0.3, 0.1, 0.3, 0.1],
        "right_arm": [-1.5, -1.3, -0.1, -0.3, -0.1, -0.3, -0.1],
        "left_gripper": 1.0,
        "right_gripper": 1.0
    }

    # 【姿态9】鞠躬礼（修复重心：腰部前倾+手臂强烈后摆）
    # 修复：腰部前倾0.2，手臂后摆-0.5补偿重心前移
    pose_bow = {
        "waist": [0.0, 0.2, 0.0, 0.0, 0.0],
        "head": [0.0, 0.25, 0.0],
        "left_arm": [-0.5, 0.2, 0.0, -0.4, 0.0, 0.0, 0.0],   # 强烈后摆
        "right_arm": [0.5, 0.2, 0.0, -0.4, 0.0, 0.0, 0.0],
        "left_gripper": 0.0,
        "right_gripper": 0.0
    }

    # 【姿态10】左右张望（腰部扭转+头部转动+双臂自然下垂后摆）
    # 安全：手臂后摆，重心稳定
    pose_look_around = {
        "waist": [0.0, 0.0, 0.0, 0.0, 0.6],
        "head": [0.6, 0.0, 0.0],
        "left_arm": [-0.2, -0.1, 0.0, -0.3, 0.0, 0.0, 0.0],
        "right_arm": [0.2, -0.1, 0.0, -0.3, 0.0, 0.0, 0.0],
        "left_gripper": 0.5,
        "right_gripper": 0.5
    }

    # 【姿态11】夹爪演示1：双臂前伸 + 左开右闭
    # 安全：腰部直立，小幅前伸
    pose_gripper_demo1 = {
        "waist": [0.0, 0.0, 0.0, 0.0, 0.0],
        "head": [0.0, 0.0, 0.0],
        "left_arm": [0.15, -0.5, 0.2, -0.8, 0.1, 0.2, 0.1],
        "right_arm": [-0.15, -0.5, -0.2, -0.8, -0.1, -0.2, -0.1],
        "left_gripper": 1.0,   # 左开
        "right_gripper": 0.0   # 右闭
    }

    # 【姿态12】夹爪演示2：双臂前伸 + 左闭右开
    pose_gripper_demo2 = {
        "waist": [0.0, 0.0, 0.0, 0.0, 0.0],
        "head": [0.0, 0.0, 0.0],
        "left_arm": [0.15, -0.5, 0.2, -0.8, 0.1, 0.2, 0.1],
        "right_arm": [-0.15, -0.5, -0.2, -0.8, -0.1, -0.2, -0.1],
        "left_gripper": 0.0,   # 左闭
        "right_gripper": 1.0   # 右开
    }

    print("\n🎬 开始全身 22 关节 + 夹爪运动演示")
    print("=" * 70)
    print("关节构成: 腰部5轴 + 头部3轴 + 左臂7轴 + 右臂7轴 = 22轴")
    print("末端执行器: 左omnipicker + 右omnipicker")
    print("=" * 70)

    poses = [
        ("[1/13] 全身归零 + 夹爪关闭", pose_home, 0.2, 2.0),
        ("[2/13] 双臂侧平举 + 头部左看 + 夹爪打开", pose_t_pose, 0.25, 3.0),
        ("[3/13] 双臂前伸（腰部直立）+ 夹爪打开", pose_front_reach, 0.25, 3.0),
        ("[4/13] 双臂上举（大间距修复版）+ 夹爪打开", pose_wave_up, 0.25, 3.0),
        ("[5/13] 双臂抱圆 + 夹爪关闭", pose_circle, 0.25, 3.0),
        ("[6/13] 左臂高举+右臂低摆 + 左开右闭", pose_asymmetric, 0.25, 3.0),
        ("[7/13] 双臂后伸扩胸 + 夹爪打开", pose_expand_chest, 0.25, 3.0),
        ("[8/13] 双臂内旋拧转 + 夹爪关闭", pose_twist, 0.25, 3.0),
        ("[9/13] 全身舒展（微前倾）+ 夹爪打开", pose_stretch, 0.25, 3.0),
        ("[10/13] 鞠躬礼（后摆平衡）+ 夹爪关闭", pose_bow, 0.2, 3.0),
        ("[11/13] 左右张望 + 夹爪半开", pose_look_around, 0.25, 3.0),
        ("[12/13] 夹爪演示: 左开右闭", pose_gripper_demo1, 0.25, 2.0),
        ("[13/13] 夹爪演示: 左闭右开", pose_gripper_demo2, 0.25, 2.0),
    ]

    # 执行所有姿态
    for title, pose, vel, life in poses:
        print(f"\n{title}...")
        print(f"   腰部: {[round(v,2) for v in pose['waist']]}")
        print(f"   头部: {[round(v,2) for v in pose['head']]}")
        print(f"   左臂: {[round(v,2) for v in pose['left_arm']]}")
        print(f"   右臂: {[round(v,2) for v in pose['right_arm']]}")
        print(f"   左夹爪: {pose['left_gripper']:.1f} | 右夹爪: {pose['right_gripper']:.1f}")

        control_both_grippers(pose["left_gripper"], pose["right_gripper"])
        move_all_joints(
            pose["waist"], pose["head"],
            pose["left_arm"], pose["right_arm"],
            velocity=vel, lifetime=life
        )
        time.sleep(life + 0.5)

    # 连续波浪运动（保守版）
    print("\n🌊 进入全身连续波浪运动模式...")
    print("   按 Ctrl+C 停止")
    print("-" * 70)

    start_time = time.time()
    try:
        while time.time() - start_time < 20:
            t = time.time() - start_time

            # 腰部：小幅度安全波动
            waist = [
                0.0,                               # idx01: 固定0
                0.1 + 0.08 * math.sin(0.3 * t),    # idx02: 小幅度前倾
                0.1 * math.sin(0.25 * t),          # idx03: 侧弯
                0.03 * math.sin(0.4 * t),         # idx04: 微倾
                0.2 * math.sin(0.2 * t),           # idx05: 扭转
            ]

            # 头部：左右看+点头
            head = [
                0.3 * math.sin(0.5 * t),
                0.1 * math.sin(0.7 * t),
                0.05 * math.sin(0.9 * t),
            ]

            # 左臂：小幅度波浪（避免前伸过多）
            left_arm = [
                0.2 * math.sin(0.4 * t),           # joint1: 小幅度
                -0.5 + 0.3 * math.sin(0.6 * t),   # joint2: 肩肘波动
                0.2 * math.sin(0.8 * t),           # joint3
                -0.6 + 0.3 * math.sin(1.0 * t),   # joint4
                0.15 * math.sin(1.2 * t),          # joint5
                0.3 * math.sin(1.4 * t),           # joint6
                0.1 * math.sin(1.6 * t),           # joint7
            ]

            # 右臂：相位偏移
            right_arm = [
                0.2 * math.sin(0.4 * t + math.pi),
                -0.5 + 0.3 * math.sin(0.6 * t + math.pi),
                0.2 * math.sin(0.8 * t + math.pi),
                -0.6 + 0.3 * math.sin(1.0 * t + math.pi),
                0.15 * math.sin(1.2 * t + math.pi),
                0.3 * math.sin(1.4 * t + math.pi),
                0.1 * math.sin(1.6 * t + math.pi),
            ]

            # 夹爪周期性开合
            gripper_open = (math.sin(0.5 * t) + 1.0) / 2.0

            move_all_joints(waist, head, left_arm, right_arm, velocity=0.4, lifetime=0.5)
            control_both_grippers(gripper_open, 1.0 - gripper_open)
            time.sleep(0.4)

    except KeyboardInterrupt:
        print("\n⏹️ 用户中断")

    # 回到初始位置
    print("\n🏠 回到初始位置...")
    control_both_grippers(0.0, 0.0)
    move_all_joints(
        pose_home["waist"], pose_home["head"],
        pose_home["left_arm"], pose_home["right_arm"],
        velocity=0.2, lifetime=3.0
    )
    time.sleep(3.0)

    print("\n" + "=" * 70)
    print("🎉 全身 22 关节 + 夹爪运动演示完成！")

    if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
        print("❌ GDK 释放失败")
    else:
        print("✅ GDK 释放成功")


if __name__ == "__main__":
    main()