import agibot_gdk
import time
import math

def main():
    # ===== 1. 初始化 GDK =====
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("❌ GDK 初始化失败")
        exit(1)
    print("✅ GDK 初始化成功")

    # ===== 2. 创建机器人 =====
    robot = agibot_gdk.Robot()
    print("✅ 机器人对象创建成功")
    time.sleep(2)  # 等待 DDS 连接

    # ===== 3. 先读取当前头部状态 =====
    try:
        joint_states = robot.get_joint_states()
        for state in joint_states['states']:
            if state['name'] == 'idx11_head_joint1':
                print(f"📍 当前头部俯仰角: {state['position']:.3f} rad")
                break
    except RuntimeError as e:
        print(f"⚠️ 读取状态失败: {e}")

    # ===== 4. 正弦点头运动 =====
    print("\n🚀 开始头部点头运动...")
    print("   按 Ctrl+C 停止\n")

    # 运动参数（安全范围）
    amplitude = 0.3      # 幅度 ±0.3 rad（约 ±17°），不要改太大！
    frequency = 0.5      # 频率 0.5 Hz（每秒半个周期，较慢）
    duration = 10.0      # 总时长 10 秒
    dt = 0.05            # 控制周期 50ms（20Hz 发送指令）

    start_time = time.time()
    try:
        while time.time() - start_time < duration:
            t = time.time() - start_time
            angle = amplitude * math.sin(2 * math.pi * frequency * t)

            # 构造控制请求
            req = agibot_gdk.JointControlReq()
            req.joint_names = ["idx11_head_joint1"]
            req.joint_positions = [angle]
            req.joint_velocities = [0.5]   # 速度限制 0.5 rad/s
            req.life_time = 0.1            # 生命周期 100ms，短一点更平滑
            req.detail = "头部点头"

            result = robot.joint_control_request(req)
            if result != 0:
                print(f"⚠️ 控制失败，返回值: {result}")
                break

            # 每 1 秒打印一次当前角度
            if int(t) > int(t - dt):
                print(f"  t={t:.1f}s, 目标角度={angle:.3f} rad")

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n⏹️ 用户中断")

    # ===== 5. 回到中间位置（安全复位）=====
    print("\n🔄 复位到中间位置...")
    req = agibot_gdk.JointControlReq()
    req.joint_names = ["idx11_head_joint1"]
    req.joint_positions = [0.0]
    req.joint_velocities = [0.3]
    req.life_time = 2.0
    req.detail = "复位"
    robot.joint_control_request(req)
    time.sleep(1.5)

    # ===== 6. 释放资源 =====
    if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
        print("❌ GDK 释放失败")
    else:
        print("✅ GDK 释放成功")
    print("\n🎉 程序结束")

if __name__ == "__main__":
    main()


''' 安全参数说明（不要改！）

参数	值	说明
amplitude = 0.3	±17°	头部俯仰安全范围，不要大于 0.5
frequency = 0.5	0.5 Hz	较慢的点头速度，不要大于 1.0
dt = 0.05	50ms	每 50ms 发一次指令，不要小于 0.02
life_time = 0.1	100ms	指令有效期短，断联自动停止
🎯 如果你想摇头（左右转）
把 idx11_head_joint1 改成 idx12_head_joint2：
Python

req.joint_names = ["idx12_head_joint2"]  # 偏航关节：摇头

如果想同时点头+摇头，用列表控制两个关节：
Python

req.joint_names = ["idx11_head_joint1", "idx12_head_joint2"]
req.joint_positions = [nod_angle, shake_angle]


'''
