#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""105_g2_o10_dual_hand_joint_probe_v1.py
G2 + OmniHand O10 双手 10+10 DOF Joint 只读 Probe。
"""
from __future__ import annotations
import json, math, os, shlex, sys, time, traceback
from datetime import datetime
from pathlib import Path
from typing import Any

READ_ONLY_PROBE=True
ENABLE_REAL_MOTION=False
VERSION='105_v1_o10_dual_hand_joint'
ENV_SH=Path.home()/'.cache/agibot/app/env.sh'
REPORT_DIR=Path(__file__).resolve().parent/'reports'
WAIT_SEC=2.0
RETRIES=10
RETRY_SEC=0.5

LEFT_O10=[
'idx31_hand_l_thumb_roll_joint','idx32_hand_l_thumb_abad_joint','idx33_hand_l_thumb_mcp_joint',
'idx36_hand_l_index_abad_joint','idx37_hand_l_index_pip_joint','idx39_hand_l_middle_pip_joint',
'idx41_hand_l_ring_abad_joint','idx42_hand_l_ring_pip_joint','idx44_hand_l_pinky_abad_joint','idx45_hand_l_pinky_pip_joint']
RIGHT_O10=[
'idx71_hand_r_thumb_roll_joint','idx72_hand_r_thumb_abad_joint','idx73_hand_r_thumb_mcp_joint',
'idx76_hand_r_index_abad_joint','idx77_hand_r_index_pip_joint','idx79_hand_r_middle_pip_joint',
'idx81_hand_r_ring_abad_joint','idx82_hand_r_ring_pip_joint','idx84_hand_r_pinky_abad_joint','idx85_hand_r_pinky_pip_joint']


def ensure_env():
    if os.environ.get('G2_GDK_ENV_READY')=='1': return
    if not ENV_SH.is_file(): raise FileNotFoundError(str(ENV_SH))
    args=' '.join(shlex.quote(x) for x in [str(Path(__file__).resolve())]+sys.argv[1:])
    cmd='source %s && export G2_GDK_ENV_READY=1 && exec /usr/bin/python3.10 %s'%(shlex.quote(str(ENV_SH)),args)
    print('[ENV] Restarting with official GDK environment ...',flush=True)
    os.execv('/bin/bash',['bash','-lc',cmd])

def field(o,n,d=None): return o.get(n,d) if isinstance(o,dict) else getattr(o,n,d)
def fnum(v):
    try:return None if v is None else float(v)
    except:return None

def serial(v,depth=0):
    if depth>4:return repr(v)
    if v is None or isinstance(v,(str,int,float,bool)):return v
    if isinstance(v,dict):return {str(k):serial(x,depth+1) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [serial(x,depth+1) for x in v]
    if hasattr(v,'__dict__'):
        try:return {k:serial(x,depth+1) for k,x in vars(v).items() if not k.startswith('_')}
        except:pass
    return repr(v)

def get_states(robot):
    last=None
    for i in range(1,RETRIES+1):
        try:
            raw=robot.get_joint_states()
            states=raw.get('states') if isinstance(raw,dict) else raw
            if isinstance(states,(list,tuple)):return raw,list(states),i
        except Exception as e:last=e
        time.sleep(RETRY_SEC)
    raise RuntimeError('get_joint_states failed: %r'%last)

def rec(s):
    return {'name':field(s,'name'),'mode':serial(field(s,'mode')),
            'motor_position':fnum(field(s,'motor_position')),'motor_velocity':fnum(field(s,'motor_velocity')),
            'motor_current':fnum(field(s,'motor_current')),'effort':fnum(field(s,'effort')),
            'error_code':serial(field(s,'error_code')),'raw':serial(s)}

def inspect_hand(label,names,index):
    arr=[]; missing=[]; ok=True
    for i,n in enumerate(names,1):
        s=index.get(n)
        if s is None: missing.append(n); arr.append({'sequence':i,'name':n,'found':False}); ok=False; continue
        r=rec(s); r.update({'sequence':i,'found':True})
        p=r['motor_position']; v=r['motor_velocity']
        r['position_finite']=p is not None and math.isfinite(p); r['velocity_finite']=None if v is None else math.isfinite(v)
        if not r['position_finite']:ok=False
        arr.append(r)
    return {'hand':label,'expected':10,'found':10-len(missing),'missing':missing,'ok':ok and not missing,'joints':arr}

def show(h):
    print(); print('-'*96); print('[%s] FOUND %d/10'%(h['hand'],h['found'])); print('-'*96)
    for j in h['joints']:
        if not j.get('found'): print('[MISSING]',j['name']); continue
        print('J%02d %-38s pos=% .6f vel=%s current=%s effort=%s error=%s'%(j['sequence'],j['name'],j['motor_position'],str(j['motor_velocity']),str(j['motor_current']),str(j['effort']),str(j['error_code'])))

def main():
    ensure_env(); print('='*96); print('G2 O10 Dual-Hand 10+10 DOF Joint Read-Only Probe'); print('Version             :',VERSION); print('Python executable   :',sys.executable); print('READ_ONLY_PROBE     :',READ_ONLY_PROBE); print('ENABLE_REAL_MOTION  :',ENABLE_REAL_MOTION); print('Motion API          : NONE'); print('='*96)
    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION:return 90
    import agibot_gdk
    initialized=False
    try:
        r=agibot_gdk.gdk_init(); print('[GDK] gdk_init result   :',r)
        if r!=agibot_gdk.GDKRes.kSuccess:raise RuntimeError('gdk_init failed: %s'%r)
        initialized=True; robot=agibot_gdk.Robot(); time.sleep(WAIT_SEC)
        raw,states,attempt=get_states(robot); print('[JOINT] read attempt     : %d/%d'%(attempt,RETRIES)); print('[JOINT] total states     :',len(states)); print('[JOINT] reported nums    :',raw.get('nums') if isinstance(raw,dict) else 'N/A')
        index={}; duplicates=[]
        for s in states:
            n=field(s,'name')
            if not n:continue
            if n in index:duplicates.append(n)
            else:index[n]=s
        left=inspect_hand('LEFT_O10',LEFT_O10,index); right=inspect_hand('RIGHT_O10',RIGHT_O10,index); show(left); show(right)
        print(); print('-'*96); print('[LEFT/RIGHT POSITION COMPARISON]')
        comparisons=[]
        for i,(ln,rn) in enumerate(zip(LEFT_O10,RIGHT_O10),1):
            lp=next((x['motor_position'] for x in left['joints'] if x.get('name')==ln and x.get('found')),None); rp=next((x['motor_position'] for x in right['joints'] if x.get('name')==rn and x.get('found')),None)
            d=None if lp is None or rp is None else lp-rp; comparisons.append({'sequence':i,'left':ln,'right':rn,'left_position':lp,'right_position':rp,'left_minus_right':d})
            print('%02d left=% .6f right=% .6f delta=% .6f'%(i,lp if lp is not None else float('nan'),rp if rp is not None else float('nan'),d if d is not None else float('nan')))
        overall=left['ok'] and right['ok'] and not duplicates
        report={'version':VERSION,'timestamp':datetime.now().astimezone().isoformat(),'total_states':len(states),'duplicates':duplicates,'left':left,'right':right,'position_comparisons':comparisons,'overall_ok':overall}
        REPORT_DIR.mkdir(parents=True,exist_ok=True); rp=REPORT_DIR/('105_o10_dual_hand_joint_v1_%s.json'%datetime.now().strftime('%Y%m%d_%H%M%S')); rp.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(); print('='*96); print('PROBE SUMMARY'); print('='*96); print('Left O10 joints       : %d/10'%left['found']); print('Right O10 joints      : %d/10'%right['found']); print('Duplicate names       :',duplicates if duplicates else 'NONE'); print('O10 commanded         : NO'); print('Motion executed       : NO'); print('Overall               :','PASS' if overall else 'CHECK REQUIRED'); print('JSON report           :',rp); print('='*96)
        return 0 if overall else 2
    finally:
        if initialized:
            rr=agibot_gdk.gdk_release(); print(); print('[GDK] gdk_release result :',rr)
if __name__=='__main__':
    try:raise SystemExit(main())
    except Exception as e:
        print(); print('[FATAL] %s: %s'%(type(e).__name__,e)); traceback.print_exc(); print('[SAFETY] No O10/motion command was issued.'); raise SystemExit(1)
