import agibot_gdk
import time


def main():
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("❌ GDK 初始化失败")
        exit(1)
    print("✅ GDK 初始化成功")

    robot = agibot_gdk.Robot()
    print("✅ 机器人对象创建成功")
    time.sleep(2)

    # ===== 获取关节状态（修正为 dict 访问方式）=====
    try:
        joint_states = robot.get_joint_states()
        print(f"✅ 成功读取关节状态！共 {joint_states['nums']} 个关节")

        # 先打印前5个看看结构
        for i, state in enumerate(joint_states['states'][:5]):
            # state 可能是对象或 dict，两种都试试
            if isinstance(state, dict):
                print(f"  关节 {i}: {state['name']} = {state['position']:.3f} rad")
            else:
                print(f"  关节 {i}: {state.name} = {state.position:.3f} rad")
    except RuntimeError as e:
        print(f"⚠️ 读取关节状态失败: {e}")

    # ===== 发送关节控制请求 =====
    req = agibot_gdk.JointControlReq()
    req.joint_names = ["idx11_head_joint1"]
    req.joint_positions = [0.0]
    req.joint_velocities = [0.0]
    req.life_time = 5.0
    req.detail = "测试头部关节"

    result = robot.joint_control_request(req)
    if result == agibot_gdk.GDKRes.kSuccess:
        print("✅ 关节控制请求发送成功")
    else:
        print(f"⚠️ 关节控制请求失败，返回值: {result}")

    # 释放资源
    if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
        print("❌ GDK 释放失败")
    else:
        print("✅ GDK 释放成功")


if __name__ == "__main__":
    main()