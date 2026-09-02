#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""智元G2双臂OmniPicker抓盆，PNC任务同步快速版。"""
import agibot_gdk
import time, math, sys, json, os

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
    GRASP_WIDEN_J3=0.032          # 抓取前总间距比旧版宽约2cm
    POST_GRIP_TIGHTEN_J3=0.016   # 夹爪闭合后双臂总共再内收约1cm

    WAIST_TOL=0.03
    WAIST_PERIOD=0.012
    WAIST_HZ=100.0
    WAIST_ATTEMPTS=4
    CAL_FILE=os.path.join(os.path.dirname(os.path.abspath(__file__)),"g2_safe_waist_reference.json")

    FORWARD=0.50
    BACKWARD=-0.50
    TURN_DELTA=-math.pi/2
    TASK_TIMEOUT=20.0
    TASK_POLL=0.10
    TASK_FALLBACK_WAIT=3.0

    def __init__(self):
        print("="*72)
        print("【版本】task-sync-fast-tight1cm / NO-SLAM-ODOM")
        print("="*72)
        if agibot_gdk.gdk_init()!=agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("GDK初始化失败")
        self.robot=agibot_gdk.Robot(); time.sleep(2)
        self.tf=agibot_gdk.TF(); time.sleep(1)
        self.pnc=agibot_gdk.Pnc(); time.sleep(2)
        self.safe_waist=None
        print("Robot、TF、PNC就绪；不创建Slam，不调用get_odom_info")

    def _joint_map(self):
        d=self.robot.get_joint_states(); states=d["states"] if isinstance(d,dict) else d.states
        out={}
        for st in states:
            if isinstance(st,dict): n=st.get("name"); v=st.get("motor_position",st.get("position",0.0))
            else: n=getattr(st,"name",""); v=getattr(st,"motor_position",getattr(st,"position",0.0))
            out[n]=float(v)
        return out

    def waist_pos(self):
        m=self._joint_map(); return [m.get(n,0.0) for n in self.WAIST]

    def clamp_arm(self,q): return [max(a,min(v,b)) for v,a,b in zip(q,self.ARM_MIN,self.ARM_MAX)]
    def clamp_waist(self,q): return [max(a,min(v,b)) for v,a,b in zip(q,self.WAIST_MIN,self.WAIST_MAX)]

    def move_arms(self,l,r,velocity):
        l=self.clamp_arm(l); r=self.clamp_arm(r)
        try:
            ret=self.robot.move_arm_joint(l+r,[velocity]*14,2)
            if ret!=0: print(f"双臂运动失败:{ret}"); return False
            return True
        except Exception as e: print(f"双臂运动异常:{e}"); return False

    def move_waist_smooth(self,target,duration):
        target=self.clamp_waist(target); start=self.waist_pos(); n=max(2,int(duration*self.WAIST_HZ)); dt=1/self.WAIST_HZ; t0=time.time()
        try:
            for i in range(n):
                t=(i+1)/n; q=[a+t*(b-a) for a,b in zip(start,target)]
                if self.robot.move_waist_joint_servo(q,self.WAIST_PERIOD)!=0: return False
                left=t0+(i+1)*dt-time.time()
                if left>0: time.sleep(left)
            return True
        except Exception as e: print(f"腰部运动异常:{e}"); return False

    def move_waist_exact(self,target):
        previous=None
        for k in range(1,self.WAIST_ATTEMPTS+1):
            err=[abs(a-b) for a,b in zip(self.waist_pos(),target)]; mx=max(err)
            print(f"腰部闭环{k}/{self.WAIST_ATTEMPTS}: {[round(x,4) for x in err]}")
            if mx<=self.WAIST_TOL: return True
            if not self.move_waist_smooth(target,max(2.5,min(5.0,1.5+mx/0.10))): return False
            time.sleep(0.6)
            after=max(abs(a-b) for a,b in zip(self.waist_pos(),target))
            if after<=self.WAIST_TOL: return True
            if previous is not None and after>=previous-0.005:
                print("腰部误差不再收敛"); return False
            previous=after
        return False

    def load_waist(self):
        try:
            with open(self.CAL_FILE,"r",encoding="utf-8") as f: d=json.load(f)
            q=d.get("waist",[])
            if len(q)!=5: raise ValueError("waist不是5个值")
            self.safe_waist=[float(x) for x in q]; print(f"固定抓取高度:{[round(x,5) for x in q]}"); return True
        except Exception as e: print(f"读取腰部标定失败:{e}"); return False

    def one_gripper(self,group,pos):
        js=agibot_gdk.JointStates(); js.group=group; js.target_type="omnipicker"
        st=agibot_gdk.JointState(); st.position=max(self.GRIPPER_OPEN,min(pos,self.GRIPPER_CLOSE))
        js.states=[st]; js.nums=1
        return self.robot.move_ee_pos(js)==0

    def gripper(self,pos):
        try:
            # 一次完整左右指令已足够，取消重复两轮造成的额外等待
            ok1=self.one_gripper("left_tool",pos); time.sleep(0.10)
            ok2=self.one_gripper("right_tool",pos); time.sleep(0.20)
            return ok1 and ok2
        except Exception as e: print(f"OmniPicker异常:{e}"); return False

    def open_gripper(self): return self.gripper(self.GRIPPER_OPEN)
    def close_gripper(self): return self.gripper(self.GRIPPER_CLOSE)

    def level(self,l,r):
        l=list(l); r=list(r); l[5]+=self.TOOL_LEVEL_CORRECTION; r[5]+=self.TOOL_LEVEL_CORRECTION
        return self.clamp_arm(l),self.clamp_arm(r)
    def widen(self,l,r):
        l=list(l); r=list(r); l[2]-=self.GRASP_WIDEN_J3; r[2]+=self.GRASP_WIDEN_J3
        return self.clamp_arm(l),self.clamp_arm(r)
    def tighten(self,l,r):
        l=list(l); r=list(r); l[2]+=self.POST_GRIP_TIGHTEN_J3; r[2]-=self.POST_GRIP_TIGHTEN_J3
        return self.clamp_arm(l),self.clamp_arm(r)

    def home(self): return [1.571,-1.571,-1.571,-1.571,0,0,0],[-1.571,-1.571,1.571,-1.571,0,0,0]
    def pre(self):
        l=[1.0384,-.4499,-.8836,-1.3103,-1.2244,-.0975,-1.5350]; r=[-1.0384,-.4499,.8836,-1.3103,1.2244,-.0975,1.5350]
        return self.widen(*self.level(l,r))
    def grasp(self):
        l=[1.0775,-.6531,-.7560,-1.2266,-1.1235,-.0809,-1.4215]; r=[-1.0775,-.6531,.7560,-1.2266,1.1235,-.0809,1.4215]
        return self.widen(*self.level(l,r))
    def grasp_tight(self): return self.tighten(*self.grasp())
    def lift_tight(self):
        l=[1.0203,-.4381,-.9223,-1.5762,-1.1871,-.0952,-1.3054]; r=[-1.0203,-.4381,.9223,-1.5762,1.1871,-.0952,1.3054]
        return self.tighten(*self.widen(*self.level(l,r)))

    @staticmethod
    def task_text(st):
        vals=[]
        for n in ("state","status","task_state","result"):
            v=getattr(st,n,None)
            if v is not None: vals.append(str(v))
        vals.append(str(st)); return " | ".join(vals).upper()

    def wait_task_terminal(self):
        """等待任务进入IDLE/FAIL/SUCCESS；若状态对象不可解析则采用短等待兜底。"""
        t0=time.time(); readable=False; last=""
        while time.time()-t0<self.TASK_TIMEOUT:
            try:
                st=self.pnc.get_task_state(); text=self.task_text(st); last=text; readable=True
                terminal=any(x in text for x in ("IDLE","SUCCESS","FAIL"))
                running=any(x in text for x in ("RUNNING","EXECUTING","ACTIVE","PENDING","PROCESSING"))
                if terminal and not running:
                    print(f"PNC任务终态:{text}"); return True
            except Exception as e:
                print(f"读取PNC任务状态提示:{e}"); break
            time.sleep(self.TASK_POLL)
        if not readable:
            print(f"任务状态不可读，兜底等待{self.TASK_FALLBACK_WAIT:.1f}s")
            time.sleep(self.TASK_FALLBACK_WAIT); return True
        print(f"PNC任务未进入终态，最后状态:{last}"); return False

    def relative_move(self,distance):
        direction="前进" if distance>0 else "后退"
        req=agibot_gdk.NaviReq(); req.target.position.x=float(distance); req.target.position.y=0.; req.target.position.z=0.
        req.target.orientation.x=0.; req.target.orientation.y=0.; req.target.orientation.z=0.; req.target.orientation.w=1.
        # 先等上一个任务终态，解决偶发Task is not in IDLE
        if not self.wait_task_terminal(): return False
        for attempt in range(1,4):
            try:
                print(f"底盘{direction}{abs(distance):.2f}m，发送{attempt}/3")
                self.pnc.relative_move(req)
                return self.wait_task_terminal()
            except Exception as e:
                text=str(e); print(f"底盘{direction}异常:{text}")
                if "NOT IN IDLE" in text.upper() and attempt<3:
                    time.sleep(0.4)
                    if not self.wait_task_terminal(): return False
                    continue
                return False
        return False

    def stop(self):
        try:
            tw=agibot_gdk.Twist(); tw.linear.x=0.;tw.linear.y=0.;tw.angular.z=0.;self.pnc.move_chassis(tw)
        except Exception: pass

    def run(self):
        if not self.load_waist(): return False
        self.stop()
        if not self.open_gripper(): return False
        if not self.move_waist_exact(self.safe_waist): return False
        hL,hR=self.home(); pL,pR=self.pre(); gL,gR=self.grasp(); tL,tR=self.grasp_tight(); lL,lR=self.lift_tight()
        print("Step1 预抓取");
        if not self.move_arms(pL,pR,.20): return False
        print("Step2 前进0.50m");
        if not self.relative_move(self.FORWARD): return False
        print("Step3 到盆沿");
        if not self.move_arms(gL,gR,.15): return False
        print("Step4 闭合夹爪并额外内收约1cm");
        if not self.close_gripper(): return False
        if not self.move_arms(tL,tR,.10): return False
        print("Step5 抬升");
        if not self.move_arms(lL,lR,.18): return False
        print("Step6 后退0.50m");
        if not self.relative_move(self.BACKWARD): return False
        print("Step7 腰部立即旋转90度");
        turn=list(self.safe_waist); turn[4]=max(self.WAIST_MIN[4],min(self.WAIST_MAX[4],turn[4]+self.TURN_DELTA))
        if not self.move_waist_smooth(turn,2.0): return False
        print("Step8 放置");
        if not self.move_arms(tL,tR,.15): return False
        print("Step9 松开撤离复位");
        if not self.open_gripper(): return False
        if not self.move_arms(pL,pR,.18): return False
        if not self.move_waist_exact(self.safe_waist): return False
        return self.move_arms(hL,hR,.18)

    def shutdown(self):
        self.stop(); agibot_gdk.gdk_release()

def main():
    c=None
    try:
        c=G2BinGraspController(); c.run()
    except KeyboardInterrupt: print("用户中断")
    except Exception as e:
        print(f"程序异常:{e}"); import traceback; traceback.print_exc()
    finally:
        if c: c.shutdown()

if __name__=="__main__": main()
