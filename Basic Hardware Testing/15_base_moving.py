#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G2机器人底盘向右移动20cm
使用PNC模块的relative_move接口
"""

import agibot_gdk
import time
import sys


class ChassisController:
    """G2底盘控制器"""

    def __init__(self):
        print("=" * 60)
        print("G2底盘移动控制器")
        print("=" * 60)

        # 初始化GDK
        res = agibot_gdk.gdk_init()
        if res != agibot_gdk.GDKRes.kSuccess:
            print(f"❌ GDK初始化失败: {res}")
            sys.exit(1)
        print("✅ GDK初始化成功")

        # 创建PNC对象（用于导航和底盘控制）
        self.pnc = agibot_gdk.Pnc()
        time.sleep(2)
        print("✅ PNC对象就绪")

        # 创建Robot对象（用于状态查询）
        self.robot = agibot_gdk.Robot()
        time.sleep(1)
        print("✅ Robot对象就绪")

        self._check_status()

    def _check_status(self):
        """检查机器人状态"""
        try:
            status = self.robot.get_whole_body_status()
            print(f"\n>>> 机器人状态")
            print(f"   底盘错误码: {status.get('chassis_error', '未知')}")

            # 检查是否有错误
            if status.get('chassis_error', 0) != 0:
                print(f"   ⚠️ 底盘存在错误码: {status['chassis_error']}")
        except Exception as e:
            print(f"   ⚠️ 获取状态失败: {e}")

    def move_relative(self, x, y, z=0.0, yaw=0.0):
        """
        相对移动（base_link坐标系）

        Args:
            x: 前后移动（米），正=前进
            y: 左右移动（米），正=左移
            z: 上下移动（米），暂不支持
            yaw: 旋转（弧度），正=逆时针
        """
        print(f"\n>>> 相对移动指令")
        print(f"   X (前后): {x:.3f} m")
        print(f"   Y (左右): {y:.3f} m")
        print(f"   旋转: {yaw:.3f} rad")

        # 创建导航请求
        target = agibot_gdk.NaviReq()
        target.target.position.x = x
        target.target.position.y = y
        target.target.position.z = z
        target.target.orientation.x = 0.0
        target.target.orientation.y = 0.0
        target.target.orientation.z = 0.0
        target.target.orientation.w = 1.0

        try:
            # 执行相对移动（带简单停障，无避障）
            self.pnc.relative_move(target)
            print("   ✅ 移动指令已发送")
            return True
        except Exception as e:
            print(f"   ❌ 移动失败: {e}")
            return False

    def get_task_status(self):
        """获取当前任务状态"""
        try:
            task = self.pnc.get_task_state()
            state_map = {
                0: "空闲",
                1: "启动中",
                2: "运行中",
                3: "暂停中",
                4: "已暂停",
                5: "恢复中",
                6: "取消中",
                7: "已取消",
                8: "失败",
                9: "成功"
            }
            state_name = state_map.get(task.state, f"未知({task.state})")
            print(f"\n>>> 任务状态")
            print(f"   状态: {state_name}")
            print(f"   任务ID: {task.id}")
            print(f"   消息: {task.message}")
            return task
        except Exception as e:
            print(f"   ⚠️ 获取任务状态失败: {e}")
            return None

    def wait_for_completion(self, timeout=10.0):
        """
        等待移动完成

        Args:
            timeout: 超时时间（秒）

        Returns:
            bool: 是否成功完成
        """
        print(f"\n>>> 等待移动完成（超时: {timeout}s）...")
        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                task = self.pnc.get_task_state()
                if task.state == 9:  # 成功
                    print("   ✅ 移动完成")
                    return True
                elif task.state == 8:  # 失败
                    print(f"   ❌ 移动失败: {task.message}")
                    return False
                elif task.state in [7, 6]:  # 已取消/取消中
                    print(f"   ⚠️ 任务已取消")
                    return False
            except Exception as e:
                print(f"   ⚠️ 状态查询异常: {e}")

            time.sleep(0.2)

        print(f"   ⚠️ 等待超时")
        return False

    def cancel_task(self):
        """取消当前任务"""
        try:
            task = self.pnc.get_task_state()
            if task.id > 0:
                self.pnc.cancel_task(task.id)
                print(f"   ✅ 已取消任务 {task.id}")
                return True
        except Exception as e:
            print(f"   ⚠️ 取消任务失败: {e}")
        return False

    def shutdown(self):
        """释放GDK"""
        print("\n>>> 释放GDK...")
        res = agibot_gdk.gdk_release()
        if res == agibot_gdk.GDKRes.kSuccess:
            print("✅ 释放成功")
        else:
            print(f"❌ 释放失败: {res}")


def main():
    """主函数"""
    controller = ChassisController()

    try:
        # 向右移动20cm (0.2米)
        # 注意：在base_link坐标系中，Y正方向是左侧
        # 所以向右移动是负Y
        MOVE_DISTANCE = -0.2  # 向右20cm

        print("\n" + "=" * 60)
        print(f"执行: 向右移动 {abs(MOVE_DISTANCE * 100):.0f} cm")
        print("=" * 60)

        # 执行移动
        if controller.move_relative(x=0.0, y=MOVE_DISTANCE):
            # 等待移动完成
            controller.wait_for_completion(timeout=10.0)

        # 打印当前任务状态
        controller.get_task_status()

        print("\n" + "=" * 60)
        print("程序结束")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")
        print("正在取消任务...")
        controller.cancel_task()
    except Exception as e:
        print(f"\n❌ 异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()