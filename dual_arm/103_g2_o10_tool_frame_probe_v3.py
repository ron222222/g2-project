#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""103_g2_o10_tool_frame_probe_v3.py - G2 + O10 Tool/Frame read-only probe."""
from __future__ import annotations
import json, math, os, shlex, sys, time, traceback
from datetime import datetime
from pathlib import Path
from typing import Any
import numpy as np

READ_ONLY_PROBE=True
ENABLE_REAL_MOTION=False
VERSION='103_v3_tool_frame'
ENV_SH=Path.home()/'.cache/agibot/app/env.sh'
REPORT_DIR=Path(__file__).resolve().parent/'reports'
WAIT_SEC=2.0
KEYWORDS=('tool','tcp','hand','end','palm','wrist')
TARGET_PAIRS=(('arm_l_end_link','left_tool0_body'),('arm_r_end_link','right_tool0_body'))

def ensure_env():
    if os.environ.get('G2_GDK_ENV_READY')=='1': return
    if not ENV_SH.is_file(): raise FileNotFoundError(str(ENV_SH))
    args=' '.join(shlex.quote(x) for x in [str(Path(__file__).resolve())]+sys.argv[1:])
    cmd='source %s && export G2_GDK_ENV_READY=1 && exec /usr/bin/python3.10 %s'%(shlex.quote(str(ENV_SH)),args)
    print('[ENV] Restarting with official GDK environment ...',flush=True)
    os.execv('/bin/bash',['bash','-lc',cmd])

def serial(v,depth=0):
    if depth>5:return repr(v)
    if v is None or isinstance(v,(str,int,float,bool)):return v
    if isinstance(v,dict):return {str(k):serial(x,depth+1) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [serial(x,depth+1) for x in v]
    if hasattr(v,'__dict__'):
        try:return {k:serial(x,depth+1) for k,x in vars(v).items() if not k.startswith('_')}
        except Exception:pass
    return repr(v)

def xyzq(obj):
    t=getattr(obj,'transform',obj); tr=getattr(t,'translation'); ro=getattr(t,'rotation')
    p=np.array([float(tr.x),float(tr.y),float(tr.z)]); q=np.array([float(ro.x),float(ro.y),float(ro.z),float(ro.w)])
    return p,q

def qrot(q):
    x,y,z,w=q; n=np.linalg.norm(q)
    if n<1e-12:raise ValueError('zero quaternion')
    x,y,z,w=q/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def euler(R):
    pitch=math.asin(max(-1,min(1,-float(R[2,0]))))
    if abs(math.cos(pitch))>1e-9: roll=math.atan2(R[2,1],R[2,2]); yaw=math.atan2(R[1,0],R[0,0])
    else: roll=0.; yaw=math.atan2(-R[0,1],R[1,1])
    return np.degrees([yaw,pitch,roll])

def parse_tf(obj):
    p,q=xyzq(obj); R=qrot(q)
    return {'translation_m':p.tolist(),'distance_m':float(np.linalg.norm(p)),'quaternion_xyzw':q.tolist(),'euler_zyx_deg':euler(R).tolist(),'rotation_matrix':R.tolist(),'raw':serial(obj)}

def main():
    ensure_env()
    print('='*92); print('G2 O10 Tool Frame Read-Only Probe'); print('Version             :',VERSION); print('Python executable   :',sys.executable); print('READ_ONLY_PROBE     :',READ_ONLY_PROBE); print('ENABLE_REAL_MOTION  :',ENABLE_REAL_MOTION); print('Motion API          : NONE'); print('='*92)
    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION:return 90
    import agibot_gdk
    mp=getattr(agibot_gdk,'__file__','') or ''; print('[GDK] module path       :',mp)
    initialized=False
    try:
        r=agibot_gdk.gdk_init(); print('[GDK] gdk_init result   :',r)
        if r!=agibot_gdk.GDKRes.kSuccess:raise RuntimeError('gdk_init failed: %s'%r)
        initialized=True; tf=agibot_gdk.TF(); time.sleep(WAIT_SEC)
        frames=list(tf.get_all_frame_names())
        names=[str(x) for x in frames]
        print('[TF] total frame names  :',len(names))
        filtered=[n for n in names if any(k in n.lower() for k in KEYWORDS)]
        print(); print('-'*92); print('[FILTERED FRAME NAMES: tool/tcp/hand/end/palm/wrist]')
        for n in filtered:print(' ',n)
        if not filtered:print('  NONE')
        report={'version':VERSION,'timestamp':datetime.now().astimezone().isoformat(),'gdk_module':mp,'total_frames':len(names),'all_frames':names,'filtered_frames':filtered,'target_pairs':{}}
        for parent,child in TARGET_PAIRS:
            key=parent+' -> '+child; entry={'parent':parent,'child':child,'parent_exists':parent in names,'child_exists':child in names}
            print(); print('-'*92); print('[TARGET]',key); print('  parent exists:',entry['parent_exists']); print('  child exists :',entry['child_exists'])
            if entry['parent_exists'] and entry['child_exists']:
                can=None
                try:can=bool(tf.can_transform(parent,child))
                except Exception as ex:entry['can_transform_error']='%s: %s'%(type(ex).__name__,ex)
                entry['can_transform']=can; print('  can_transform:',can)
                try:
                    obj=tf.lookup_transform_latest(parent,child); parsed=parse_tf(obj); entry['lookup']='PASS'; entry['transform']=parsed
                    print('  lookup       : PASS'); print('  XYZ [m]      :',parsed['translation_m']); print('  Norm [m]     : %.6f'%parsed['distance_m']); print('  Quaternion   :',parsed['quaternion_xyzw']); print('  Euler ZYX deg:',[round(x,6) for x in parsed['euler_zyx_deg']])
                except Exception as ex:
                    entry['lookup']='FAILED'; entry['lookup_error']='%s: %s'%(type(ex).__name__,ex); print('  lookup       : FAILED',entry['lookup_error'])
            else:
                entry['lookup']='SKIPPED_FRAME_MISSING'; print('  lookup       : SKIPPED_FRAME_MISSING')
            report['target_pairs'][key]=entry
        REPORT_DIR.mkdir(parents=True,exist_ok=True); rp=REPORT_DIR/('103_tool_frame_probe_v3_%s.json'%datetime.now().strftime('%Y%m%d_%H%M%S')); rp.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        left=report['target_pairs']['arm_l_end_link -> left_tool0_body']; right=report['target_pairs']['arm_r_end_link -> right_tool0_body']
        both=left.get('lookup')=='PASS' and right.get('lookup')=='PASS'
        print(); print('='*92); print('PROBE SUMMARY'); print('='*92); print('Frame enumeration     : PASS'); print('left_tool0_body exists:',left['child_exists']); print('right_tool0_body exists:',right['child_exists']); print('Left tool TF lookup   :',left.get('lookup')); print('Right tool TF lookup  :',right.get('lookup')); print('Tool/TCP modified     : NO'); print('Motion executed       : NO'); print('Overall               :','PASS' if both else 'FRAME DISCOVERY COMPLETE / TOOL FRAME CHECK REQUIRED'); print('JSON report           :',rp); print('='*92)
        return 0 if both else 2
    finally:
        if initialized:
            rr=agibot_gdk.gdk_release(); print(); print('[GDK] gdk_release result :',rr)

if __name__=='__main__':
    try:raise SystemExit(main())
    except Exception as ex:
        print(); print('[FATAL] %s: %s'%(type(ex).__name__,ex)); traceback.print_exc(); print('[SAFETY] No motion/TCP-setting command was issued.'); raise SystemExit(1)
