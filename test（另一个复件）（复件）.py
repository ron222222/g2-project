#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""智元G2双臂OmniPicker抓盆：PNC数字状态完整同步版。"""
import agibot_gdk
import time
import math
import json
import os
import traceback

class G2BinGraspController:
    LEFT=["idx21_arm_l_joint1","idx22_arm_l_joint2","idx23_arm_l_joint3","idx24_arm_l_joint4","idx25_arm_l_joint5","idx26_arm_l_joint6","idx27_arm_l_joint7"]
    RIGHT=["idx61_arm_r_joint1","idx62_arm_r_joint2","idx63_arm_r_joint3","idx64_arm_r_joint4","idx65_arm_r_joint5","idx66_arm_r_joint6","idx67_arm_r_joint7"]
    WAIST=["idx01_body_joint1","idx02_body_joint2","idx03_body_joint3","idx04_body_joint4","idx05_body_joint5"]
    ARM_MIN=[-3.071796,-2.059505,-3.071796,-2.495838,-3.071796,-1.012308,-1.535907]
    ARM_MAX=[3.071796,2.059505,3.071796,1.012308,3.071796,1.012308,1.535907]
    WAIST_MIN=[-1.082104,-0.000174,-1.919862,-0.436332,-3.045599]
    WAIST_MAX=[0.000174,2.652900,1.570970,0.436332,3.045599]

    GRIPPER_OPEN=-0.785
    GRIPPER_CLOSE=0.0
    TOOL_LEVEL_CORRECTION=0.08
    GRASP_WIDEN_J3=0.032
    POST_GRIP_TIGHTEN_J3=0.016

    WAIST_TOL=0.03
    WAIST_PERIOD=0.012
    WAIST_HZ=100.0
    WAIST_ATTEMPTS=4
    CAL_FILE=os.path.join(os.path.dirname(os.path.abspath(__file__)),"g2_safe_waist_reference.json")

    FORWARD=0.50
    BACKWARD=-0.50
    TURN_DELTA=-math.pi/2.0

    TERMINAL_STATES={0,7,8,9}
    RUNNING_STATES={1,2,3,4,5,6}
    TASK_START_TIMEOUT=5.0
    TASK_FINISH_TIMEOUT=20.0
    TASK_POLL=0.10
    TASK_RETRY=3

    def __init__(self):
        print("="*72)
        print("【版本】pnc-numeric-dual-gripper-fixed / NO-SLAM-ODOM")
        print("="*72)
        if agibot_gdk.gdk_init()!=agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("GDK初始化失败")
        self.robot=agibot_gdk.Robot(); time.sleep(2)
        self.tf=agibot_gdk.TF(); time.sleep(1)
        self.pnc=agibot_gdk.Pnc(); time.sleep(2)
        self.safe_waist=None
        print("Robot、TF、PNC就绪；不创建Slam，不调用get_odom_info")

    def _joint_map(self):
        data=self.robot.get_joint_states()
        states=data["states"] if isinstance(data,dict) else data.states
        result={}
        for state in states:
            if isinstance(state,dict):
                name=state.get("name"); value=state.get("motor_position",state.get("position",0.0))
            else:
                name=getattr(state,"name",""); value=getattr(state,"motor_position",getattr(state,"position",0.0))
            result[name]=float(value)
        return result

    def waist_positions(self):
        values=self._joint_map()
        return [values.get(name,0.0) for name in self.WAIST]

    def clamp_arm(self,q):
        return [max(lo,min(value,hi)) for value,lo,hi in zip(q,self.ARM_MIN,self.ARM_MAX)]

    def clamp_waist(self,q):
        return [max(lo,min(value,hi)) for value,lo,hi in zip(q,self.WAIST_MIN,self.WAIST_MAX)]

    def move_arms(self,left,right,velocity):
        left=self.clamp_arm(left); right=self.clamp_arm(right)
        try:
            result=self.robot.move_arm_joint(left+right,[velocity]*14,2)
            if result!=0:
                print(f"双臂运动失败: {result}")
                return False
            return True
        except Exception as error:
            print(f"双臂运动异常: {error}")
            return False

    def move_waist_smooth(self,target,duration):
        target=self.clamp_waist(target)
        start=self.waist_positions()
        steps=max(2,int(duration*self.WAIST_HZ))
        interval=1.0/self.WAIST_HZ
        begin=time.time()
        try:
            for index in range(steps):
                ratio=float(index+1)/steps
                current=[a+ratio*(b-a) for a,b in zip(start,target)]
                if self.robot.move_waist_joint_servo(current,self.WAIST_PERIOD)!=0:
                    return False
                remaining=begin+(index+1)*interval-time.time()
                if remaining>0:
                    time.sleep(remaining)
            return True
        except Exception as error:
            print(f"腰部运动异常: {error}")
            return False

    def move_waist_exact(self,target):
        previous=None
        for attempt in range(1,self.WAIST_ATTEMPTS+1):
            errors=[abs(a-b) for a,b in zip(self.waist_positions(),target)]
            maximum=max(errors)
            print(f"腰部闭环{attempt}/{self.WAIST_ATTEMPTS}: {[round(x,4) for x in errors]}")
            if maximum<=self.WAIST_TOL:
                return True
            duration=max(2.5,min(5.0,1.5+maximum/0.10))
            if not self.move_waist_smooth(target,duration):
                return False
            time.sleep(0.6)
            after=max(abs(a-b) for a,b in zip(self.waist_positions(),target))
            if after<=self.WAIST_TOL:
                return True
            if previous is not None and after>=previous-0.005:
                print("腰部误差不再收敛")
                return False
            previous=after
        return False

    def load_waist_reference(self):
        try:
            with open(self.CAL_FILE,"r",encoding="utf-8") as file:
                data=json.load(file)
            waist=data.get("waist",[])
            if len(waist)!=5:
                raise ValueError("标定文件waist不是5个值")
            self.safe_waist=[float(x) for x in waist]
            print(f"固定抓取高度: {[round(x,5) for x in self.safe_waist]}")
            return True
        except Exception as error:
            print(f"读取腰部标定失败: {error}")
            return False

    def dual_gripper_command(self,left_position,right_position):
        """按GDK dual_tool格式一次同步控制两侧OmniPicker。"""
        request=agibot_gdk.JointStates()
        request.group="dual_tool"
        request.target_type="omnipicker"
        left_state=agibot_gdk.JointState()
        right_state=agibot_gdk.JointState()
        left_state.position=max(self.GRIPPER_OPEN,min(left_position,self.GRIPPER_CLOSE))
        right_state.position=max(self.GRIPPER_OPEN,min(right_position,self.GRIPPER_CLOSE))
        request.states=[left_state,right_state]
        request.nums=2
        return self.robot.move_ee_pos(request)==0

    def single_gripper_command(self,group,position):
        """单侧补发命令，用于增强某一侧未动作时的兼容性。"""
        request=agibot_gdk.JointStates()
        request.group=group
        request.target_type="omnipicker"
        state=agibot_gdk.JointState()
        state.position=max(self.GRIPPER_OPEN,min(position,self.GRIPPER_CLOSE))
        request.states=[state]
        request.nums=1
        return self.robot.move_ee_pos(request)==0

    def gripper(self,position):
        """
        先用dual_tool同步控制左右夹爪，再分别向左、右单侧补发一次。
        重点修复左侧OmniPicker偶发不动作。
        """
        try:
            print(f"OmniPicker目标位置: {position:.3f}")
            if not self.dual_gripper_command(position,position):
                print("dual_tool同步命令返回失败")
                return False
            time.sleep(0.25)

            left_ok=self.single_gripper_command("left_tool",position)
            time.sleep(0.20)
            right_ok=self.single_gripper_command("right_tool",position)
            time.sleep(0.25)

            if not left_ok:
                print("左侧OmniPicker补发命令返回失败")
                return False
            if not right_ok:
                print("右侧OmniPicker补发命令返回失败")
                return False

            print("左右OmniPicker同步命令及单侧补发完成")
            return True
        except Exception as error:
            print(f"OmniPicker控制异常: {error}")
            return False

    def open_gripper(self):
        return self.gripper(self.GRIPPER_OPEN)

    def close_gripper(self):
        return self.gripper(self.GRIPPER_CLOSE)

    def level(self,left,right):
        left=list(left); right=list(right)
        left[5]+=self.TOOL_LEVEL_CORRECTION; right[5]+=self.TOOL_LEVEL_CORRECTION
        return self.clamp_arm(left),self.clamp_arm(right)

    def widen(self,left,right):
        left=list(left); right=list(right)
        left[2]-=self.GRASP_WIDEN_J3; right[2]+=self.GRASP_WIDEN_J3
        return self.clamp_arm(left),self.clamp_arm(right)

    def tighten(self,left,right):
        left=list(left); right=list(right)
        left[2]+=self.POST_GRIP_TIGHTEN_J3; right[2]-=self.POST_GRIP_TIGHTEN_J3
        return self.clamp_arm(left),self.clamp_arm(right)

    def pose_home(self):
        return [1.571,-1.571,-1.571,-1.571,0,0,0],[-1.571,-1.571,1.571,-1.571,0,0,0]

    def pose_pregrasp(self):
        left=[1.0384,-.4499,-.8836,-1.3103,-1.2244,-.0975,-1.5350]
        right=[-1.0384,-.4499,.8836,-1.3103,1.2244,-.0975,1.5350]
        return self.widen(*self.level(left,right))

    def pose_grasp(self):
        left=[1.0775,-.6531,-.7560,-1.2266,-1.1235,-.0809,-1.4215]
        right=[-1.0775,-.6531,.7560,-1.2266,1.1235,-.0809,1.4215]
        return self.widen(*self.level(left,right))

    def pose_grasp_tight(self):
        return self.tighten(*self.pose_grasp())

    def pose_lift_tight(self):
        left=[1.0203,-.4381,-.9223,-1.5762,-1.1871,-.0952,-1.3054]
        right=[-1.0203,-.4381,.9223,-1.5762,1.1871,-.0952,1.3054]
        return self.tighten(*self.widen(*self.level(left,right)))

    def read_task_state(self):
        task=self.pnc.get_task_state()
        return int(task.state),int(task.id),str(task.message),int(task.type)

    def wait_until_terminal(self,timeout,cancel_on_timeout=False):
        begin=time.time(); last=None
        while time.time()-begin<timeout:
            try:
                last=self.read_task_state()
                state,task_id,message,task_type=last
                if state in self.TERMINAL_STATES:
                    print(f"PNC终态: state={state}, id={task_id}, type={task_type}, message={message}")
                    return True
            except Exception as error:
                print(f"读取PNC任务状态失败: {error}")
                return False
            time.sleep(self.TASK_POLL)
        print(f"等待PNC终态超时，最后状态: {last}")
        if cancel_on_timeout and last:
            state,task_id,_,_=last
            if state not in self.TERMINAL_STATES and task_id>0:
                try:
                    self.pnc.cancel_task(task_id)
                    return self.wait_until_terminal(5.0,False)
                except Exception as error:
                    print(f"取消PNC任务失败: {error}")
        return False

    def wait_for_new_task_cycle(self,before_id,before_state):
        """避免发送后立即读到旧的state=0并误判完成。"""
        begin=time.time(); started=False; current_id=before_id
        while time.time()-begin<self.TASK_START_TIMEOUT:
            try:
                state,task_id,message,task_type=self.read_task_state()
                current_id=task_id
                if task_id!=before_id or state in self.RUNNING_STATES:
                    started=True
                    print(f"PNC新任务已启动: state={state}, id={task_id}, type={task_type}, message={message}")
                    break
                if task_id!=before_id and state in self.TERMINAL_STATES:
                    return True
            except Exception as error:
                print(f"读取PNC任务启动状态失败: {error}")
                return False
            time.sleep(self.TASK_POLL)
        if not started:
            print("发送relative_move后未检测到新任务启动")
            return False
        return self.wait_until_terminal(self.TASK_FINISH_TIMEOUT,True)

    def relative_move(self,distance):
        direction="前进" if distance>0 else "后退"
        request=agibot_gdk.NaviReq()
        request.target.position.x=float(distance)
        request.target.position.y=0.0; request.target.position.z=0.0
        request.target.orientation.x=0.0; request.target.orientation.y=0.0
        request.target.orientation.z=0.0; request.target.orientation.w=1.0

        if not self.wait_until_terminal(self.TASK_FINISH_TIMEOUT,True):
            return False
        before_state,before_id,_,_=self.read_task_state()

        for attempt in range(1,self.TASK_RETRY+1):
            try:
                print(f"底盘{direction}{abs(distance):.2f}m，发送{attempt}/{self.TASK_RETRY}")
                self.pnc.relative_move(request)
                return self.wait_for_new_task_cycle(before_id,before_state)
            except Exception as error:
                text=str(error)
                print(f"底盘{direction}异常: {text}")
                if "NOT IN IDLE" in text.upper() and attempt<self.TASK_RETRY:
                    if not self.wait_until_terminal(self.TASK_FINISH_TIMEOUT,True):
                        return False
                    before_state,before_id,_,_=self.read_task_state()
                    time.sleep(0.20)
                    continue
                return False
        return False

    def stop_chassis(self):
        try:
            twist=agibot_gdk.Twist()
            twist.linear.x=0.0; twist.linear.y=0.0; twist.angular.z=0.0
            self.pnc.move_chassis(twist)
        except Exception:
            pass

    def run(self):
        if not self.load_waist_reference(): return False
        self.stop_chassis()
        if not self.open_gripper(): return False
        if not self.move_waist_exact(self.safe_waist): return False

        home_l,home_r=self.pose_home()
        pre_l,pre_r=self.pose_pregrasp()
        grasp_l,grasp_r=self.pose_grasp()
        tight_l,tight_r=self.pose_grasp_tight()
        lift_l,lift_r=self.pose_lift_tight()

        print("Step1 预抓取")
        if not self.move_arms(pre_l,pre_r,.20): return False
        print("Step2 前进0.50m")
        if not self.relative_move(self.FORWARD): return False
        print("Step3 到盆沿")
        if not self.move_arms(grasp_l,grasp_r,.15): return False
        print("Step4 dual_tool同步闭合，左/右补发，再额外内收约1cm")
        if not self.close_gripper(): return False
        if not self.move_arms(tight_l,tight_r,.10): return False
        print("Step5 抬升")
        if not self.move_arms(lift_l,lift_r,.18): return False
        print("Step6 后退0.50m")
        if not self.relative_move(self.BACKWARD): return False
        print("Step7 腰部立即旋转90度")
        turn=list(self.safe_waist)
        turn[4]=max(self.WAIST_MIN[4],min(self.WAIST_MAX[4],turn[4]+self.TURN_DELTA))
        if not self.move_waist_smooth(turn,2.0): return False
        print("Step8 放置")
        if not self.move_arms(tight_l,tight_r,.15): return False
        print("Step9 松开、撤离、复位")
        if not self.open_gripper(): return False
        if not self.move_arms(pre_l,pre_r,.18): return False
        if not self.move_waist_exact(self.safe_waist): return False
        return self.move_arms(home_l,home_r,.18)

    def shutdown(self):
        self.stop_chassis()
        agibot_gdk.gdk_release()


def main():
    controller=None
    try:
        controller=G2BinGraspController()
        controller.run()
    except KeyboardInterrupt:
        print("用户中断")
    except Exception as error:
        print(f"程序异常: {error}")
        traceback.print_exc()
    finally:
        if controller:
            controller.shutdown()

if __name__=="__main__":
    main()
