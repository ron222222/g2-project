import agibot_gdk
import time

# 22个关节名称（按用户指定顺序）
JOINT_NAMES = [
    "idx01_body_joint1", "idx02_body_joint2", "idx03_body_joint3",
    "idx04_body_joint4", "idx05_body_joint5",
    "idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3",
    "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
    "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
    "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
    "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]

def main():
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK 初始化失败")
        return
    print("GDK 初始化成功")

    robot = agibot_gdk.Robot()
    time.sleep(2)
    print("Robot 对象创建成功")

    js = robot.get_joint_states()
    print("关节总数: " + str(js["nums"]))

    # 建立名称到位置的映射
    state_map = {}
    for state in js["states"]:
        state_map[state["name"]] = state["position"]

    # 按 JOINT_NAMES 顺序提取当前弧度
    current_positions = []
    for name in JOINT_NAMES:
        if name in state_map:
            current_positions.append(state_map[name])
        else:
            current_positions.append(0.0)

    # 按用户指定格式输出（保留2位小数）
    print("")
    print("当前22个关节弧度值：")
    print("")
    print("        %.2f, %.2f, %.2f, %.2f, %.2f," % tuple(current_positions[0:5]))
    print("        %.2f, %.2f, %.2f," % tuple(current_positions[5:8]))
    print("        %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f," % tuple(current_positions[8:15]))
    print("        %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f," % tuple(current_positions[15:22]))
    print("")

    # 同时输出高精度版本（4位小数）
    print("高精度版本：")
    print("")
    print("        %.4f, %.4f, %.4f, %.4f, %.4f," % tuple(current_positions[0:5]))
    print("        %.4f, %.4f, %.4f," % tuple(current_positions[5:8]))
    print("        %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f," % tuple(current_positions[8:15]))
    print("        %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f," % tuple(current_positions[15:22]))
    print("")

    # 打印末端执行器信息
    try:
        status = robot.get_whole_body_status()
        print("左末端型号: " + str(status["left_end_model"]))
        print("右末端型号: " + str(status["right_end_model"]))
    except Exception as e:
        print("无法获取末端信息: " + str(e))

    if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
        print("GDK 释放失败")
    else:
        print("GDK 释放成功")

if __name__ == "__main__":
    main()