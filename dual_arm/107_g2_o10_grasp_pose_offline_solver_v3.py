#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""107 v3: O10 joint-limit-aware incremental offline grasp solver. READ ONLY."""
from __future__ import annotations
import json, math, os, shlex, sys, time, traceback, xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

READ_ONLY_PROBE=True
ENABLE_REAL_MOTION=False
VERSION='107_v3_joint_limit_aware_incremental_solver'
ENV_SH=Path.home()/'.cache/agibot/app/env.sh'
BASE_DIR=Path(__file__).resolve().parent
REPORT_DIR=BASE_DIR/'reports'
URDF_NAMES=('G2_t2_crs_o10_t2.urdf','G2_t2_crs_o10_t2.urdf.txt')
WAIT_SEC=2.0
LEFT=['idx31_hand_l_thumb_roll_joint','idx32_hand_l_thumb_abad_joint','idx33_hand_l_thumb_mcp_joint','idx36_hand_l_index_abad_joint','idx37_hand_l_index_pip_joint','idx39_hand_l_middle_pip_joint','idx41_hand_l_ring_abad_joint','idx42_hand_l_ring_pip_joint','idx44_hand_l_pinky_abad_joint','idx45_hand_l_pinky_pip_joint']
RIGHT=['idx71_hand_r_thumb_roll_joint','idx72_hand_r_thumb_abad_joint','idx73_hand_r_thumb_mcp_joint','idx76_hand_r_index_abad_joint','idx77_hand_r_index_pip_joint','idx79_hand_r_middle_pip_joint','idx81_hand_r_ring_abad_joint','idx82_hand_r_ring_pip_joint','idx84_hand_r_pinky_abad_joint','idx85_hand_r_pinky_pip_joint']
BASE={'left':'hand_l_base_link','right':'hand_r_base_link'}
TIPS={'left':{'thumb':'hand_l_thumb_tip','index':'hand_l_index_tip','middle':'hand_l_middle_tip','ring':'hand_l_ring_tip','pinky':'hand_l_pinky_tip'},'right':{'thumb':'hand_r_thumb_tip','index':'hand_r_index_tip','middle':'hand_r_middle_tip','ring':'hand_r_ring_tip','pinky':'hand_r_pinky_tip'}}
MIRROR=np.array([-1.,-1.,-1.,-1.,1.,1.,-1.,1.,-1.,1.])
STAGES={
 'OPEN':{'mode':'hold','target':None,'margin':0.0,'max_delta':0.0},
 'PREGRASP':{'mode':'solve','target':0.055,'margin':0.10,'max_delta':0.35},
 'CONTACT':{'mode':'solve','target':0.030,'margin':0.10,'max_delta':0.30},
 'GRASP':{'mode':'solve','target':0.024,'margin':0.08,'max_delta':0.10}}
AP_TOL=0.002
MIRROR_TOL=0.03

def ensure_env():
    if os.environ.get('G2_GDK_ENV_READY')=='1':return
    if not ENV_SH.is_file():raise FileNotFoundError(str(ENV_SH))
    args=' '.join(shlex.quote(x) for x in [str(Path(__file__).resolve())]+sys.argv[1:])
    cmd='source %s && export G2_GDK_ENV_READY=1 && exec /usr/bin/python3.10 %s'%(shlex.quote(str(ENV_SH)),args)
    print('[ENV] Restarting with official GDK environment ...',flush=True); os.execv('/bin/bash',['bash','-lc',cmd])

def vec(text,default=(0,0,0)):return np.array([float(x) for x in text.split()] if text else default,dtype=float)
def rpy(r):
    a,b,c=r; ca,sa=math.cos(a),math.sin(a); cb,sb=math.cos(b),math.sin(b); cc,sc=math.cos(c),math.sin(c)
    return np.array([[cc,-sc,0],[sc,cc,0],[0,0,1.]])@np.array([[cb,0,sb],[0,1,0],[-sb,0,cb]])@np.array([[1,0,0],[0,ca,-sa],[0,sa,ca]])
def aa(axis,q):
    axis=axis/np.linalg.norm(axis); x,y,z=axis; c=math.cos(q); s=math.sin(q); C=1-c
    return np.array([[x*x*C+c,x*y*C-z*s,x*z*C+y*s],[y*x*C+z*s,y*y*C+c,y*z*C-x*s],[z*x*C-y*s,z*y*C+x*s,z*z*C+c]])
def T(xyz,R):
    out=np.eye(4); out[:3,:3]=R; out[:3,3]=xyz; return out

@dataclass
class Joint:
    name:str; kind:str; parent:str; child:str; xyz:np.ndarray; rr:np.ndarray; axis:np.ndarray; lo:Optional[float]; hi:Optional[float]; mimic:Optional[str]; mult:float; offset:float
class Model:
    def __init__(self,path):
        root=ET.fromstring(path.read_text(encoding='utf-8')); self.j={}; self.by_child={}
        for n in root.findall('joint'):
            o=n.find('origin'); a=n.find('axis'); lim=n.find('limit'); m=n.find('mimic')
            q=Joint(n.attrib['name'],n.attrib.get('type','fixed'),n.find('parent').attrib['link'],n.find('child').attrib['link'],vec(o.attrib.get('xyz') if o is not None else None),vec(o.attrib.get('rpy') if o is not None else None),vec(a.attrib.get('xyz') if a is not None else None,(1,0,0)),float(lim.attrib['lower']) if lim is not None and 'lower'in lim.attrib else None,float(lim.attrib['upper']) if lim is not None and 'upper'in lim.attrib else None,m.attrib.get('joint') if m is not None else None,float(m.attrib.get('multiplier','1')) if m is not None else 1.,float(m.attrib.get('offset','0')) if m is not None else 0.)
            self.j[q.name]=q; self.by_child[q.child]=q
    def chain(self,base,tip):
        out=[]; link=tip
        while link!=base:
            q=self.by_child.get(link)
            if q is None:raise KeyError('No parent joint for '+link)
            out.append(q); link=q.parent
        return list(reversed(out))
    def fk(self,base,tip,values):
        out=np.eye(4)
        for j in self.chain(base,tip):
            out=out@T(j.xyz,rpy(j.rr)); q=(j.mult*values.get(j.mimic,0.)+j.offset) if j.mimic else values.get(j.name,0.)
            if j.kind in ('revolute','continuous'):out=out@T(np.zeros(3),aa(j.axis,q))
            elif j.kind=='prismatic':out=out@T(j.axis*q,np.eye(3))
        return out

def find_urdf():
    for n in URDF_NAMES:
        p=BASE_DIR/n
        if p.is_file():return p
    raise FileNotFoundError('URDF missing beside program')
def state_vec(end,names):
    d={str(n).strip():float(s['position']) for n,s in zip(end['names'],end['end_states'])}; missing=[n for n in names if n not in d]
    if missing:raise KeyError(str(missing))
    return np.array([d[n] for n in names])
def bounds(model,names):
    return np.array([model.j[n].lo if model.j[n].lo is not None else -math.pi for n in names]),np.array([model.j[n].hi if model.j[n].hi is not None else math.pi for n in names])
def geom(model,side,names,q):
    values=dict(zip(names,q)); pts={f:model.fk(BASE[side],link,values)[:3,3] for f,link in TIPS[side].items()}; four=np.mean(np.vstack([pts[n] for n in ('index','middle','ring','pinky')]),axis=0); primary=np.mean(np.vstack([pts[n] for n in ('index','middle')]),axis=0); thumb=pts['thumb']
    return {'points':pts,'thumb':thumb,'four':four,'primary':primary,'virtual':.5*(thumb+four),'aperture':float(np.linalg.norm(thumb-four))}
def jac(fun,q,r,eps=1e-5):
    J=np.zeros((len(r),len(q)))
    for k in range(len(q)):
        q2=q.copy();q2[k]+=eps;J[:,k]=(fun(q2)-r)/eps
    return J
def evaluate(model,side,names,q,lower,upper,previous,target):
    g=geom(model,side,names,q); margins=np.minimum(q-lower,upper-q)
    return {'target_aperture_m':target,'achieved_aperture_m':g['aperture'],'aperture_error_m':None if target is None else g['aperture']-target,'joint_limit_margin_rad':float(np.min(margins)),'joint_limit_margin_each_rad':margins.tolist(),'max_joint_delta_from_previous_rad':float(np.max(np.abs(q-previous))),'joint_delta_from_previous_rad':(q-previous).tolist(),'geometry':{'tip_positions_m':{k:v.tolist() for k,v in g['points'].items()},'thumb_m':g['thumb'].tolist(),'four_center_m':g['four'].tolist(),'primary_center_m':g['primary'].tolist(),'virtual_center_m':g['virtual'].tolist()}}
def solve(model,side,names,prev,target,required_margin,max_delta,lower,upper):
    lo=np.maximum(lower+required_margin,prev-max_delta); hi=np.minimum(upper-required_margin,prev+max_delta)
    if np.any(lo>=hi):raise ValueError('No feasible safe incremental range')
    q=np.clip(prev,lo+1e-5,hi-1e-5); ref=q.copy(); damping=.03; best=q.copy(); accepted=0
    def residual(x):
        g=geom(model,side,names,x); zs=np.array([g['points'][n][2] for n in ('index','middle','ring','pinky')]); return np.concatenate(([120*(g['aperture']-target)],30*(zs-zs.mean()),[35*(g['points']['index'][2]-g['points']['middle'][2])],.08*(x-ref)))
    rr=residual(q);best_cost=.5*float(rr@rr); iterations=0
    for iterations in range(1,451):
        rr=residual(q);J=jac(residual,q,rr);A=J.T@J+damping*np.eye(len(q));b=-(J.T@rr)
        try:step=np.linalg.solve(A,b)
        except np.linalg.LinAlgError:step=np.linalg.pinv(A)@b
        m=float(np.max(np.abs(step)))
        if m>.08:step*=.08/m
        old=.5*float(rr@rr);alpha=1.;ok=False
        for _ in range(14):
            candidate=np.clip(q+alpha*step,lo+1e-5,hi-1e-5);r2=residual(candidate);cost=.5*float(r2@r2)
            if cost<old:
                q=candidate;ok=True;accepted+=1
                if cost<best_cost:best_cost=cost;best=q.copy()
                damping=max(1e-5,damping*.75);break
            alpha*=.5
        if not ok:
            damping=min(1e3,damping*4)
            if np.linalg.norm(step)<1e-7:break
        if abs(geom(model,side,names,q)['aperture']-target)<=AP_TOL:best=q.copy();break
    q=best;meta=evaluate(model,side,names,q,lower,upper,prev,target);meta.update({'solver':'numpy_incremental_joint_limit_aware_dls','iterations':iterations,'accepted_steps':accepted,'cost':best_cost,'required_limit_margin_rad':required_margin,'allowed_max_delta_rad':max_delta});meta['aperture_pass']=abs(meta['aperture_error_m'])<=AP_TOL;meta['limit_margin_pass']=meta['joint_limit_margin_rad']>=required_margin-1e-5;meta['delta_pass']=meta['max_joint_delta_from_previous_rad']<=max_delta+1e-6;meta['success']=bool(meta['aperture_pass']and meta['limit_margin_pass']and meta['delta_pass']);return q,meta

def main():
    ensure_env();print('='*104);print('G2 O10 Joint-Limit-Aware Incremental Offline Grasp Solver');print('Version             :',VERSION);print('READ_ONLY_PROBE     :',READ_ONLY_PROBE);print('ENABLE_REAL_MOTION  :',ENABLE_REAL_MOTION);print('Motion API          : NONE');print('Candidate status    : NOT APPROVED FOR MOTION');print('='*104)
    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION:return 90
    urdf=find_urdf();model=Model(urdf);print('[URDF]',urdf)
    import agibot_gdk
    initialized=False
    try:
        r=agibot_gdk.gdk_init();print('[GDK] gdk_init result   :',r)
        if r!=agibot_gdk.GDKRes.kSuccess:raise RuntimeError('gdk_init failed')
        initialized=True;robot=agibot_gdk.Robot();time.sleep(WAIT_SEC);raw=robot.get_end_state();current_l=state_vec(raw['left_end_state'],LEFT);current_r=state_vec(raw['right_end_state'],RIGHT);lb=bounds(model,LEFT);rb=bounds(model,RIGHT)
        report={'version':VERSION,'timestamp':datetime.now().astimezone().isoformat(),'safety':{'approved_for_motion':False,'o10_commanded':False,'motion_executed':False},'urdf':str(urdf),'stages':{}}
        prev_l=current_l.copy();prev_r=current_r.copy();all_ok=True
        for name,cfg in STAGES.items():
            if cfg['mode']=='hold':
                ql=current_l.copy();qr=current_r.copy();ml=evaluate(model,'left',LEFT,ql,*lb,prev_l,None);mr=evaluate(model,'right',RIGHT,qr,*rb,prev_r,None);ml.update({'solver':'hold_current_verified_pose','success':True});mr.update({'solver':'hold_current_verified_pose','success':True})
            else:
                ql,ml=solve(model,'left',LEFT,prev_l,cfg['target'],cfg['margin'],cfg['max_delta'],*lb);qr,mr=solve(model,'right',RIGHT,prev_r,cfg['target'],cfg['margin'],cfg['max_delta'],*rb)
            normalized=MIRROR*qr;delta=ql-normalized;mirror=float(np.max(np.abs(delta)));stage_ok=bool(ml['success']and mr['success']and mirror<=MIRROR_TOL);all_ok=all_ok and stage_ok
            report['stages'][name]={'status':'CANDIDATE_ONLY_NOT_APPROVED','stage_ok':stage_ok,'left_joint_names':LEFT,'right_joint_names':RIGHT,'left_positions_rad':ql.tolist(),'right_positions_rad':qr.tolist(),'right_positions_in_left_convention_rad':normalized.tolist(),'mirror_delta_rad':delta.tolist(),'max_mirror_delta_rad':mirror,'left_solver':ml,'right_solver':mr}
            print();print('[%s CANDIDATE]'%name);print('  Target aperture [mm] :','CURRENT' if cfg['target']is None else '%.1f'%(1000*cfg['target']));print('  Left achieved [mm]   : %.3f'%(1000*ml['achieved_aperture_m']));print('  Right achieved [mm]  : %.3f'%(1000*mr['achieved_aperture_m']));print('  Left limit margin    : %.4f rad'%ml['joint_limit_margin_rad']);print('  Right limit margin   : %.4f rad'%mr['joint_limit_margin_rad']);print('  Left max stage delta : %.4f rad'%ml['max_joint_delta_from_previous_rad']);print('  Right max stage delta: %.4f rad'%mr['max_joint_delta_from_previous_rad']);print('  Max mirror delta     : %.4f rad'%mirror);print('  Stage status         :','PASS_CANDIDATE' if stage_ok else 'CHECK_REQUIRED');prev_l,prev_r=ql,qr
        REPORT_DIR.mkdir(parents=True,exist_ok=True);rp=REPORT_DIR/('107_o10_offline_grasp_solver_v3_%s.json'%datetime.now().strftime('%Y%m%d_%H%M%S'));rp.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print();print('='*104);print('OFFLINE SOLVER SUMMARY');print('='*104);print('URDF parsed                   : PASS');print('Current O10 state read        : PASS');print('Incremental joint-limit solver:', 'CANDIDATES GENERATED' if all_ok else 'CHECK REQUIRED');print('Absolute tray contact Z       : DEFERRED TO 108');print('Approved for real motion      : NO');print('O10 commanded                 : NO');print('Motion executed               : NO');print('JSON report                   :',rp);print('='*104);return 0 if all_ok else 2
    finally:
        if initialized:print();print('[GDK] gdk_release result :',agibot_gdk.gdk_release())
if __name__=='__main__':
    try:raise SystemExit(main())
    except Exception as e:print();print('[FATAL] %s: %s'%(type(e).__name__,e));traceback.print_exc();print('[SAFETY] No O10 or robot motion command was issued.');raise SystemExit(1)
