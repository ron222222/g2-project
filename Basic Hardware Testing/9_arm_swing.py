import agibot_gdk
import time


def main():
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("❌ GDK 初始化失败")
        return
    print("✅ GDK 初始化成功")

    robot = agibot_gdk.Robot()
    time.sleep(2)
    print("✅ Robot 对象创建成功")

    all_arm_joints = [
        "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
        "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
        "idx27_arm_l_joint7",
        "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
        "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
        "idx67_arm_r_joint7"
    ]

    # ===== 安全姿态定义 =====

    # 初始姿态
    pose_home = [0.0] * 14

    # 左臂挥手（已验证安全）
    pose_wave_left = [
        0.0, -1.0, 0.0, -1.5, 0.0, 0.0, 0.0,  # 左臂举起
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  # 右臂下垂
    ]

    # 右臂挥手（已验证安全）
    pose_wave_right = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # 左臂下垂
        0.0, -1.0, 0.0, -1.5, 0.0, 0.0, 0.0  # 右臂举起
    ]

    # 【安全】双臂交替举起（一高一低，避免夹爪碰撞）
    # 左臂高举 + 右臂低举（高度差足够大）
    pose_left_high_right_low = [
        # 左臂：高举过顶
        0.0,  # joint1
        -1.5,  # joint2: 举到最高
        0.0,  # joint3
        -0.5,  # joint4: 肘微弯（手向上）
        0.0, 0.0, 0.0,
        # 右臂：只举到腰部
        0.0,  # joint1
        -0.3,  # joint2: 只抬一点
        0.0,  # joint3
        -0.5,  # joint4: 肘弯曲
        0.0, 0.0, 0.0
    ]

    # 【安全】双臂镜像侧举（V字形，夹爪朝外）
    # 关键：joint6 让夹爪朝外，避免相向碰撞
    pose_v_shape = [
        # 左臂：向左上方展开，夹爪朝外
        1.0,  # joint1: 肩外旋（向左）
        -1.2,  # joint2: 举起
        0.0,  # joint3
        -1.0,  # joint4: 肘弯曲
        0.0,  # joint5
        1.0,  # joint6: 腕旋转（夹爪朝外）
        0.0,  # joint7
        # 右臂：向右上方展开，夹爪朝外
        -1.0,  # joint1: 肩外旋（向右）
        -1.2,  # joint2: 举起
        0.0,  # joint3
        -1.0,  # joint4: 肘弯曲
        0.0,  # joint5
        -1.0,  # joint6: 腕旋转（夹爪朝外）
        0.0  # joint7
    ]

    # 【安全】左臂敬礼 + 右臂叉腰（完全不同的区域）
    pose_salute_handonhip = [
        # 左臂：敬礼（手在额头）
        0.3,  # joint1: 轻微内收
        -1.0,  # joint2: 举起
        0.5,  # joint3: 前臂向内
        -1.5,  # joint4: 肘弯曲
        0.0,  # joint5
        0.5,  # joint6: 手腕弯曲
        0.0,  # joint7
        # 右臂：叉腰（手在身体侧面）
        -0.5,  # joint1: 外展
        -0.5,  # joint2: 微抬
        0.0,  # joint3
        -1.0,  # joint4: 肘弯曲
        0.0,  # joint5
        0.0,  # joint6
        0.0  # joint7
    ]

    def move_to_pose(joint_positions, velocity=0.3, lifetime=3.0):
        req = agibot_gdk.JointControlReq()
        req.joint_names = all_arm_joints
        req.joint_positions = joint_positions
        req.joint_velocities = [velocity] * 14
        req.life_time = lifetime
        req.detail = "双臂协调控制"

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

    print("\n🎬 开始双臂协调动作演示（避免碰撞版）")
    print("=" * 50)

    # 动作 1: 回初始位置
    print("\n[1/6] 回初始位置...")
    move_to_pose(pose_home, velocity=0.2, lifetime=2.0)
    time.sleep(2.5)

    # 动作 2: 左臂挥手
    print("\n[2/6] 左臂挥手...")
    for i in range(2):
        move_to_pose(pose_wave_left, velocity=0.5, lifetime=1.0)
        time.sleep(1.2)
        move_to_pose(pose_home, velocity=0.5, lifetime=1.0)
        time.sleep(1.2)

    # 动作 3: 右臂挥手
    print("\n[3/6] 右臂挥手...")
    for i in range(2):
        move_to_pose(pose_wave_right, velocity=0.5, lifetime=1.0)
        time.sleep(1.2)
        move_to_pose(pose_home, velocity=0.5, lifetime=1.0)
        time.sleep(1.2)

    # 动作 4: 【安全】双臂一高一低
    print("\n[4/6] 双臂一高一低...")
    move_to_pose(pose_left_high_right_low, velocity=0.3, lifetime=3.0)
    time.sleep(3.0)

    # 动作 5: 【安全】V字形展开
    print("\n[5/6] 双臂V字形展开...")
    move_to_pose(pose_v_shape, velocity=0.3, lifetime=3.0)
    time.sleep(3.0)

    # 动作 6: 【安全】敬礼+叉腰
    print("\n[6/6] 左臂敬礼 + 右臂叉腰...")
    move_to_pose(pose_salute_handonhip, velocity=0.3, lifetime=3.0)
    time.sleep(3.0)

    # 回到初始位置
    print("\n🏠 回到初始位置...")
    move_to_pose(pose_home, velocity=0.2, lifetime=3.0)
    time.sleep(3.0)

    print("\n" + "=" * 50)
    print("🎉 动作演示完成！")

    if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
        print("❌ GDK 释放失败")
    else:
        print("✅ GDK 释放成功")


if __name__ == "__main__":
    main()