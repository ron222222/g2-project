#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""102_g2_o10_dual_arm_tf_probe_v1.py
G2 + OmniHand O10 双臂 TF 只读 Probe v1
运行前: source ~/.cache/agibot/app/env.sh
运行: /usr/bin/python3.10 102_g2_o10_dual_arm_tf_probe_v1.py
只读：不包含任何运动、控制模式、TCP设置、O10控制或底盘命令。
"""
from __future__ import annotations
import json, math, os, sys, time, traceback
from datetime import datetime
from pathlib import Path
from typing import Any, Dict
import numpy as np

READ_ONLY_PROBE=True
ENABLE_REAL_MOTION=False
PROGRAM_VERSION="102_v1"
WAIT_SEC=2.0
RETRIES=10
RETRY_SEC=0.5
REPORT_DIR=Path(__file__).resolve().parent/'reports'
FRAMES=['arm_l_end_link','arm_r_end_link','hand_l_base_link','hand_r_base_link']

def serial(v):
    if v is None or isinstance(v,(str,int,float,bool)): return v
    if isinstance(v,dict): return {str(k):serial(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)): return [serial(x) for x in v]
    if hasattr(v,'__dict__'):
        try:return {k:serial(x) for k,x in vars(v).items() if not k.startswith('_')}
        except Exception:pass
    return repr(v)

def xyzq(tf):
    tr=getattr(tf,'translation',None); ro=getattr(tf,'rotation',None)
    if tr is None and hasattr(tf,'transform'):
        tr=tf.transform.translation; ro=tf.transform.rotation
    return np.array([float(tr.x),float(tr.y),float(tr.z)]), np.array([float(ro.x),float(ro.y),float(ro.z),float(ro.w)])

def qrot(q):
    x,y,z,w=q; n=np.linalg.norm(q)
    if n==0: raise ValueError('zero quaternion')
    x,y,z,w=q/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def mat(p,q):
    T=np.eye(4); T[:3,:3]=qrot(q); T[:3,3]=p; return T

def euler_zyx(R):
    pitch=math.asin(max(-1,min(1,-R[2,0])))
    if abs(math.cos(pitch))>1e-9:
        roll=math.atan2(R[2,1],R[2,2]); yaw=math.atan2(R[1,0],R[0,0])
    else:
        roll=0.; yaw=math.atan2(-R[0,1],R[1,1])
    return np.degrees([yaw,pitch,roll])

def tf_record(T):
    R=T[:3,:3]; p=T[:3,3]
    return {'translation_m':p.tolist(),'distance_m':float(np.linalg.norm(p)),
            'rotation_matrix':R.tolist(),'euler_zyx_deg':euler_zyx(R).tolist(),'matrix_4x4':T.tolist()}

def get_base_tf(tf_api, frame):
    last=None
    for i in range(1,RETRIES+1):
        try:
            x=tf_api.get_tf_from_base_link(frame)
            if x is not None: return x,i
        except Exception as e:last=e
        time.sleep(RETRY_SEC)
    raise RuntimeError(f'无法读取 base_link -> {frame}; last={last}')

def cycle_error(A,B):
    C=A@B
    terr=float(np.linalg.norm(C[:3,3])); R=C[:3,:3]
    c=max(-1.,min(1.,(np.trace(R)-1)/2)); rang=math.degrees(math.acos(c))
    return {'translation_error_m':terr,'rotation_error_deg':rang,'matrix':C.tolist()}

def main():
    print('='*92); print('G2 O10 Dual Arm TF Read-Only Probe'); print('Version             :',PROGRAM_VERSION)
    print('Python executable   :',sys.executable); print('ROS_DISTRO          :',os.getenv('ROS_DISTRO','<not set>'))
    print('READ_ONLY_PROBE     :',READ_ONLY_PROBE); print('ENABLE_REAL_MOTION  :',ENABLE_REAL_MOTION); print('Motion API          : NONE'); print('='*92)
    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION: print('[FATAL] Safety lock invalid.'); return 90
    if '/.cache/agibot/app/gdk/lib' not in os.getenv('PYTHONPATH',''):
        print('[FATAL] 请先执行: source ~/.cache/agibot/app/env.sh'); return 91
    import agibot_gdk
    mp=getattr(agibot_gdk,'__file__','') or ''; print('[GDK] module path       :',mp)
    if '/.cache/agibot/app/gdk/lib/agibot_gdk/' not in mp:
        print('[FATAL] 非官方 env.sh 指向的 GDK runtime，停止。'); return 92
    initialized=False
    try:
        r=agibot_gdk.gdk_init(); print('[GDK] gdk_init result   :',r)
        if r!=agibot_gdk.GDKRes.kSuccess: raise RuntimeError(f'gdk_init failed: {r}')
        initialized=True
        print('[GDK] Creating TF() [read only] ...')
        tf_api=agibot_gdk.TF()
        print(f'[DDS] Waiting {WAIT_SEC:.1f}s ...'); time.sleep(WAIT_SEC)
        Ts={}; raw={}
        for frame in FRAMES:
            obj,attempt=get_base_tf(tf_api,frame); p,q=xyzq(obj); T=mat(p,q); Ts[frame]=T
            raw[frame]={'attempt':attempt,'translation_m':p.tolist(),'quaternion_xyzw':q.tolist(),'derived':tf_record(T),'raw':serial(obj)}
            print(f'
[TF] base_link -> {frame}  attempt={attempt}/{RETRIES}')
            print('     XYZ [m]        :',np.array2string(p,precision=6))
            print('     Quaternion xyzw:',np.array2string(q,precision=7))
            print('     Euler ZYX [deg]:',np.array2string(euler_zyx(T[:3,:3]),precision=4))
        pairs=[('arm_l_end_link','arm_r_end_link','LEFT_END_TO_RIGHT_END'),('hand_l_base_link','hand_r_base_link','LEFT_HAND_TO_RIGHT_HAND')]
        rel={}
        for a,b,label in pairs:
            AB=np.linalg.inv(Ts[a])@Ts[b]; BA=np.linalg.inv(Ts[b])@Ts[a]
            rel[label]={'forward':tf_record(AB),'reverse':tf_record(BA),'cycle_forward_reverse':cycle_error(AB,BA),'cycle_reverse_forward':cycle_error(BA,AB)}
            print('
'+'-'*92); print('[RELATIVE]',label)
            print(f'  {a} -> {b} XYZ [m] :',np.array2string(AB[:3,3],precision=6))
            print('  distance [m]             :',f'{np.linalg.norm(AB[:3,3]):.6f}')
            print('  Euler ZYX [deg]          :',np.array2string(euler_zyx(AB[:3,:3]),precision=4))
            ce=rel[label]['cycle_forward_reverse']
            print('  closure translation [m]  :',f"{ce['translation_error_m']:.3e}")
            print('  closure rotation [deg]    :',f"{ce['rotation_error_deg']:.3e}")
        max_t=max(x['cycle_forward_reverse']['translation_error_m'] for x in rel.values())
        max_r=max(x['cycle_forward_reverse']['rotation_error_deg'] for x in rel.values())
        ok=max_t<1e-6 and max_r<1e-6
        report={'program':'G2 O10 Dual Arm TF Read-Only Probe','version':PROGRAM_VERSION,'timestamp_local':datetime.now().astimezone().isoformat(),
                'safety':{'read_only_probe':True,'enable_real_motion':False,'motion_executed':False},
                'environment':{'python':sys.executable,'ros_distro':os.getenv('ROS_DISTRO'),'agibot_gdk_module':mp},
                'base_transforms':raw,'relative_transforms':rel,'max_cycle_translation_error_m':max_t,'max_cycle_rotation_error_deg':max_r,'overall_ok':ok}
        REPORT_DIR.mkdir(exist_ok=True); stamp=datetime.now().strftime('%Y%m%d_%H%M%S'); rp=REPORT_DIR/f'102_dual_arm_tf_probe_v1_{stamp}.json'
        rp.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print('
'+'='*92); print('PROBE SUMMARY'); print('='*92)
        print('Base TF left end      : PASS'); print('Base TF right end     : PASS'); print('Base TF left hand     : PASS'); print('Base TF right hand    : PASS')
        print('Relative transforms   : PASS'); print('Transform closure     :','PASS' if ok else 'CHECK REQUIRED')
        print('Overall               :','PASS' if ok else 'CHECK REQUIRED'); print('JSON report           :',rp); print('Motion executed       : NO'); print('='*92)
        return 0 if ok else 2
    finally:
        if initialized:
            rr=agibot_gdk.gdk_release(); print('
[GDK] gdk_release result :',rr)

if __name__=='__main__':
    try: raise SystemExit(main())
    except KeyboardInterrupt:
        print('
[STOP] User interrupted. No motion command was issued.'); raise SystemExit(130)
    except Exception as e:
        print(f'
[FATAL] {type(e).__name__}: {e}'); traceback.print_exc(); print('[SAFETY] Probe stopped. No motion command was issued.'); raise SystemExit(1)
