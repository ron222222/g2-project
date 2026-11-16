#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""104_g2_o10_hand_geometry_fingertip_tf_probe_v1.py
G2 + OmniHand O10 双手几何和指尖 TF 只读 Probe。
"""
from __future__ import annotations
import json, math, os, shlex, sys, time, traceback
from datetime import datetime
from pathlib import Path
import numpy as np

READ_ONLY_PROBE=True
ENABLE_REAL_MOTION=False
VERSION='104_v1_hand_geometry_fingertip_tf'
ENV_SH=Path.home()/'.cache/agibot/app/env.sh'
REPORT_DIR=Path(__file__).resolve().parent/'reports'
WAIT_SEC=2.0

HAND_FRAMES={
 'left': {
   'base':'hand_l_base_link','arm_end':'arm_l_end_link','tool':'left_tool0_body',
   'tips':['hand_l_thumb_tip','hand_l_index_tip','hand_l_middle_tip','hand_l_ring_tip','hand_l_pinky_tip']},
 'right': {
   'base':'hand_r_base_link','arm_end':'arm_r_end_link','tool':'right_tool0_body',
   'tips':['hand_r_thumb_tip','hand_r_index_tip','hand_r_middle_tip','hand_r_ring_tip','hand_r_pinky_tip']}
}

def ensure_env():
    if os.environ.get('G2_GDK_ENV_READY')=='1': return
    if not ENV_SH.is_file(): raise FileNotFoundError(str(ENV_SH))
    args=' '.join(shlex.quote(x) for x in [str(Path(__file__).resolve())]+sys.argv[1:])
    cmd='source %s && export G2_GDK_ENV_READY=1 && exec /usr/bin/python3.10 %s'%(shlex.quote(str(ENV_SH)),args)
    print('[ENV] Restarting with official GDK environment ...',flush=True)
    os.execv('/bin/bash',['bash','-lc',cmd])

def unwrap(result):
    if isinstance(result,(tuple,list)):
        if not result: raise ValueError('empty transform tuple/list')
        obj=result[0]
        if obj is None: raise ValueError('transform item[0] is None')
        return obj
    return result

def xyzq(obj):
    obj=unwrap(obj); t=getattr(obj,'transform',obj)
    tr=getattr(t,'translation'); ro=getattr(t,'rotation')
    return np.array([float(tr.x),float(tr.y),float(tr.z)]),np.array([float(ro.x),float(ro.y),float(ro.z),float(ro.w)])

def qrot(q):
    x,y,z,w=q; n=np.linalg.norm(q)
    if n<1e-12: raise ValueError('zero quaternion')
    x,y,z,w=q/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def euler_zyx(R):
    pitch=math.asin(max(-1.,min(1.,-float(R[2,0]))))
    if abs(math.cos(pitch))>1e-9:
        roll=math.atan2(R[2,1],R[2,2]); yaw=math.atan2(R[1,0],R[0,0])
    else:
        roll=0.; yaw=math.atan2(-R[0,1],R[1,1])
    return np.degrees([yaw,pitch,roll])

def parse(result):
    p,q=xyzq(result); R=qrot(q)
    return {'translation_m':p.tolist(),'distance_m':float(np.linalg.norm(p)),
            'quaternion_xyzw':q.tolist(),'euler_zyx_deg':euler_zyx(R).tolist(),
            'rotation_matrix':R.tolist()}

def lookup(tf,parent,child):
    if not tf.can_transform(parent,child): raise RuntimeError('can_transform=False: %s -> %s'%(parent,child))
    return parse(tf.lookup_transform_latest(parent,child))

def pairwise_distances(points):
    out={}; names=list(points)
    for i in range(len(names)):
        for j in range(i+1,len(names)):
            a,b=names[i],names[j]
            pa=np.array(points[a]['translation_m']); pb=np.array(points[b]['translation_m'])
            out[a+' <-> '+b]=float(np.linalg.norm(pa-pb))
    return out

def main():
    ensure_env()
    print('='*96); print('G2 O10 Dual-Hand Geometry & Fingertip TF Read-Only Probe'); print('Version             :',VERSION); print('Python executable   :',sys.executable); print('READ_ONLY_PROBE     :',READ_ONLY_PROBE); print('ENABLE_REAL_MOTION  :',ENABLE_REAL_MOTION); print('Motion API          : NONE'); print('='*96)
    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION:return 90
    import agibot_gdk
    initialized=False
    try:
        r=agibot_gdk.gdk_init(); print('[GDK] gdk_init result   :',r)
        if r!=agibot_gdk.GDKRes.kSuccess: raise RuntimeError('gdk_init failed: %s'%r)
        initialized=True; tf=agibot_gdk.TF(); time.sleep(WAIT_SEC)
        names=[str(x) for x in tf.get_all_frame_names()]; print('[TF] total frames       :',len(names))
        report={'version':VERSION,'timestamp':datetime.now().astimezone().isoformat(),'hands':{},'cross_hand':{}}
        overall=True
        for side,cfg in HAND_FRAMES.items():
            print(); print('#'*96); print('[%s HAND]'%side.upper()); print('#'*96)
            required=[cfg['base'],cfg['arm_end'],cfg['tool']]+cfg['tips']; missing=[n for n in required if n not in names]
            print('Required frames      :',len(required)); print('Missing frames       :',missing if missing else 'NONE')
            if missing: overall=False
            hand={'missing_frames':missing,'arm_end_to_hand_base':None,'arm_end_to_tool':None,'base_to_tips':{},'tip_pair_distances_m':{}}
            if cfg['arm_end'] in names and cfg['base'] in names:
                hand['arm_end_to_hand_base']=lookup(tf,cfg['arm_end'],cfg['base'])
                print('arm_end -> hand_base :',hand['arm_end_to_hand_base']['translation_m'],'m')
            if cfg['arm_end'] in names and cfg['tool'] in names:
                hand['arm_end_to_tool']=lookup(tf,cfg['arm_end'],cfg['tool'])
                print('arm_end -> tool0     :',hand['arm_end_to_tool']['translation_m'],'m')
            for tip in cfg['tips']:
                if cfg['base'] not in names or tip not in names: continue
                rec=lookup(tf,cfg['base'],tip); hand['base_to_tips'][tip]=rec
                print(); print('  [TIP]',tip); print('    XYZ from hand_base [m]:',rec['translation_m']); print('    distance [m]          : %.6f'%rec['distance_m']); print('    Euler ZYX [deg]       :',[round(v,5) for v in rec['euler_zyx_deg']])
            hand['tip_pair_distances_m']=pairwise_distances(hand['base_to_tips'])
            print(); print('  Fingertip pair distances [m]:')
            for k,v in hand['tip_pair_distances_m'].items(): print('    %-62s %.6f'%(k,v))
            report['hands'][side]=hand
        # Current left/right fingertip separation in base_link coordinates.
        for finger in ('thumb','index','middle','ring','pinky'):
            lf='hand_l_%s_tip'%finger; rf='hand_r_%s_tip'%finger
            if lf in names and rf in names:
                rec=lookup(tf,lf,rf); report['cross_hand'][finger+'_tip_left_to_right']=rec
                print(); print('[CROSS HAND] %-6s tip L -> R distance [m]: %.6f'%(finger,rec['distance_m']))
        REPORT_DIR.mkdir(parents=True,exist_ok=True); rp=REPORT_DIR/('104_hand_geometry_fingertip_tf_v1_%s.json'%datetime.now().strftime('%Y%m%d_%H%M%S')); rp.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(); print('='*96); print('PROBE SUMMARY'); print('='*96); print('Left hand geometry    :','PASS' if not report['hands']['left']['missing_frames'] else 'CHECK REQUIRED'); print('Right hand geometry   :','PASS' if not report['hands']['right']['missing_frames'] else 'CHECK REQUIRED'); print('Five fingertips/hand  :',len(report['hands']['left']['base_to_tips']),'+',len(report['hands']['right']['base_to_tips'])); print('Tool/TCP modified     : NO'); print('O10 commanded         : NO'); print('Motion executed       : NO'); print('Overall               :','PASS' if overall else 'CHECK REQUIRED'); print('JSON report           :',rp); print('='*96)
        return 0 if overall else 2
    finally:
        if initialized:
            rr=agibot_gdk.gdk_release(); print(); print('[GDK] gdk_release result :',rr)
if __name__=='__main__':
    try:raise SystemExit(main())
    except Exception as e:
        print(); print('[FATAL] %s: %s'%(type(e).__name__,e)); traceback.print_exc(); print('[SAFETY] No motion/O10/TCP-setting command was issued.'); raise SystemExit(1)
