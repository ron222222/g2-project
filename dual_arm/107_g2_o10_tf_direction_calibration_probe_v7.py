#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
107_g2_o10_tf_direction_calibration_probe_v7.py

G2 + OmniHand O10 TF lookup direction calibration probe v7.
Read-only. No motion commands.

Purpose:
1. Establish the exact semantic direction of TF.lookup_transform_latest(A, B).
2. Compare both query orders against URDF A->B and its inverse.
3. Use multiple known frame pairs, including the verified ~18 mm arm_end/hand_base pair.
4. Produce an evidence-based convention for later O10 FK-vs-live-TF comparison.
"""
from __future__ import annotations
import json, math, os, shlex, sys, time, traceback, xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional
import numpy as np

READ_ONLY_PROBE=True
ENABLE_REAL_MOTION=False
VERSION='107_v7_tf_direction_calibration'
ENV_SH=Path.home()/'.cache/agibot/app/env.sh'
BASE_DIR=Path(__file__).resolve().parent
REPORT_DIR=BASE_DIR/'reports'
URDF_NAMES=('G2_t2_crs_o10_t2.urdf','G2_t2_crs_o10_t2.urdf.txt')
WAIT_SEC=2.0
POS_TOL_M=0.002
ROT_TOL_DEG=1.0

PAIRS=[
 ('arm_l_end_link','hand_l_base_link'),
 ('arm_r_end_link','hand_r_base_link'),
 ('hand_l_base_link','hand_l_thumb_roll_link'),
 ('hand_r_base_link','hand_r_thumb_roll_link'),
 ('hand_l_base_link','hand_l_index_abad_link'),
 ('hand_r_base_link','hand_r_index_abad_link'),
]

def ensure_env():
    if os.environ.get('G2_GDK_ENV_READY')=='1':return
    if not ENV_SH.is_file():raise FileNotFoundError(str(ENV_SH))
    args=' '.join(shlex.quote(x) for x in [str(Path(__file__).resolve())]+sys.argv[1:])
    cmd='source %s && export G2_GDK_ENV_READY=1 && exec /usr/bin/python3.10 %s'%(shlex.quote(str(ENV_SH)),args)
    print('[ENV] Restarting with official GDK environment ...',flush=True)
    os.execv('/bin/bash',['bash','-lc',cmd])
def vec(text,default=(0,0,0)):return np.array([float(x) for x in text.split()] if text else default,dtype=float)
def rpy(v):
    r,p,y=v;cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy,-sy,0],[sy,cy,0],[0,0,1.]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
def aa(axis,q):
    axis=axis/np.linalg.norm(axis);x,y,z=axis;c,s,C=math.cos(q),math.sin(q),1-math.cos(q)
    return np.array([[x*x*C+c,x*y*C-z*s,x*z*C+y*s],[y*x*C+z*s,y*y*C+c,y*z*C-x*s],[z*x*C-y*s,z*y*C+x*s,z*z*C+c]])
def T(xyz,R):
    o=np.eye(4);o[:3,:3]=R;o[:3,3]=xyz;return o
@dataclass
class Joint:
    name:str;kind:str;parent:str;child:str;xyz:np.ndarray;rr:np.ndarray;axis:np.ndarray;mimic:Optional[str];mult:float;offset:float
class Model:
    def __init__(self,path):
        root=ET.fromstring(path.read_text(encoding='utf-8'));self.j={};self.by_child={}
        for n in root.findall('joint'):
            o=n.find('origin');a=n.find('axis');m=n.find('mimic')
            q=Joint(n.attrib['name'],n.attrib.get('type','fixed'),n.find('parent').attrib['link'],n.find('child').attrib['link'],vec(o.attrib.get('xyz') if o is not None else None),vec(o.attrib.get('rpy') if o is not None else None),vec(a.attrib.get('xyz') if a is not None else None,(1,0,0)),m.attrib.get('joint') if m is not None else None,float(m.attrib.get('multiplier','1')) if m is not None else 1.,float(m.attrib.get('offset','0')) if m is not None else 0.)
            self.j[q.name]=q;self.by_child[q.child]=q
    def chain(self,base,tip):
        out=[];link=tip
        while link!=base:
            q=self.by_child.get(link)
            if q is None:raise KeyError('No URDF chain %s -> %s at %s'%(base,tip,link))
            out.append(q);link=q.parent
        return list(reversed(out))
    def resolve(self,name,values,stack=None):
        if name in values:return float(values[name])
        q=self.j.get(name)
        if q is None or not q.mimic:return 0.
        stack=set() if stack is None else set(stack)
        if name in stack:raise RuntimeError('mimic cycle')
        stack.add(name);return q.mult*self.resolve(q.mimic,values,stack)+q.offset
    def fk(self,base,tip,values):
        out=np.eye(4)
        for q in self.chain(base,tip):
            out=out@T(q.xyz,rpy(q.rr));v=self.resolve(q.name,values)
            if q.kind in ('revolute','continuous'):out=out@T(np.zeros(3),aa(q.axis,v))
            elif q.kind=='prismatic':out=out@T(q.axis*v,np.eye(3))
        return out

def find_urdf():
    for n in URDF_NAMES:
        p=BASE_DIR/n
        if p.is_file():return p
    raise FileNotFoundError('URDF missing')
def unwrap(v):return v[0] if isinstance(v,(tuple,list)) else v
def live_matrix(tf_api,a,b):
    v=unwrap(tf_api.lookup_transform_latest(a,b));o=getattr(v,'transform',v);tr=o.translation;ro=o.rotation;q=np.array([ro.x,ro.y,ro.z,ro.w],dtype=float);q/=np.linalg.norm(q);x,y,z,w=q
    R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    return T(np.array([tr.x,tr.y,tr.z],dtype=float),R)
def rot_err_deg(A,B):
    R=A[:3,:3]@B[:3,:3].T;c=max(-1.,min(1.,(np.trace(R)-1)/2));return math.degrees(math.acos(c))
def compare(candidate,reference):return float(np.linalg.norm(candidate[:3,3]-reference[:3,3])),rot_err_deg(candidate,reference)
def states(raw):
    out={}
    for key in ('left_end_state','right_end_state'):
        e=raw[key]
        for n,s in zip(e['names'],e['end_states']):out[str(n).strip()]=float(s['position'])
    return out

def main():
    ensure_env();print('='*108);print('G2 O10 TF Direction Calibration Probe');print('Version             :',VERSION);print('READ_ONLY_PROBE     :',READ_ONLY_PROBE);print('ENABLE_REAL_MOTION  :',ENABLE_REAL_MOTION);print('Motion API          : NONE');print('='*108)
    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION:return 90
    urdf=find_urdf();model=Model(urdf)
    import agibot_gdk
    initialized=False
    try:
        r=agibot_gdk.gdk_init();print('[GDK] gdk_init result   :',r)
        if r!=agibot_gdk.GDKRes.kSuccess:raise RuntimeError('gdk_init failed')
        initialized=True;robot=agibot_gdk.Robot();tf=agibot_gdk.TF();time.sleep(WAIT_SEC);values=states(robot.get_end_state())
        report={'version':VERSION,'timestamp':datetime.now().astimezone().isoformat(),'urdf':str(urdf),'pairs':{}}
        convention_votes=[]
        for a,b in PAIRS:
            print();print('-'*108);print('[PAIR] %s <-> %s'%(a,b))
            urdf_ab=model.fk(a,b,values);urdf_ba=np.linalg.inv(urdf_ab);live_ab=live_matrix(tf,a,b);live_ba=live_matrix(tf,b,a)
            direct=compare(live_ab,urdf_ab);inverse=compare(live_ab,urdf_ba);reverse_direct=compare(live_ba,urdf_ba);reverse_inverse=compare(live_ba,urdf_ab)
            direct_score=direct[0]+direct[1]/180.;inverse_score=inverse[0]+inverse[1]/180.
            inferred='ARG1_TO_ARG2' if direct_score<inverse_score else 'ARG2_TO_ARG1';convention_votes.append(inferred)
            print(' live(A,B) XYZ [m]          :',live_ab[:3,3]);print(' URDF A->B XYZ [m]          :',urdf_ab[:3,3]);print(' inverse(URDF A->B) XYZ [m] :',urdf_ba[:3,3])
            print(' live(A,B) vs URDF A->B     : pos=%7.3f mm rot=%6.3f deg'%(1000*direct[0],direct[1]))
            print(' live(A,B) vs URDF B->A     : pos=%7.3f mm rot=%6.3f deg'%(1000*inverse[0],inverse[1]))
            print(' live(B,A) vs URDF B->A     : pos=%7.3f mm rot=%6.3f deg'%(1000*reverse_direct[0],reverse_direct[1]))
            print(' inferred convention         :',inferred)
            report['pairs'][a+' <-> '+b]={'live_ab':live_ab.tolist(),'live_ba':live_ba.tolist(),'urdf_ab':urdf_ab.tolist(),'urdf_ba':urdf_ba.tolist(),'direct_pos_error_m':direct[0],'direct_rot_error_deg':direct[1],'inverse_pos_error_m':inverse[0],'inverse_rot_error_deg':inverse[1],'reverse_direct_pos_error_m':reverse_direct[0],'reverse_direct_rot_error_deg':reverse_direct[1],'reverse_inverse_pos_error_m':reverse_inverse[0],'reverse_inverse_rot_error_deg':reverse_inverse[1],'inferred_convention':inferred}
        a_votes=convention_votes.count('ARG1_TO_ARG2');b_votes=convention_votes.count('ARG2_TO_ARG1');overall='ARG1_TO_ARG2' if a_votes>b_votes else ('ARG2_TO_ARG1' if b_votes>a_votes else 'AMBIGUOUS')
        report['vote_counts']={'ARG1_TO_ARG2':a_votes,'ARG2_TO_ARG1':b_votes};report['overall_convention']=overall
        REPORT_DIR.mkdir(parents=True,exist_ok=True);rp=REPORT_DIR/('107_tf_direction_calibration_v7_%s.json'%datetime.now().strftime('%Y%m%d_%H%M%S'));rp.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print();print('='*108);print('TF DIRECTION SUMMARY');print('='*108);print('ARG1_TO_ARG2 votes :',a_votes);print('ARG2_TO_ARG1 votes :',b_votes);print('Overall convention :',overall);print('Robot commanded    : NO');print('Motion executed    : NO');print('JSON report        :',rp);print('='*108);return 0 if overall!='AMBIGUOUS' else 2
    finally:
        if initialized:print();print('[GDK] gdk_release result :',agibot_gdk.gdk_release())
if __name__=='__main__':
    try:raise SystemExit(main())
    except Exception as e:print();print('[FATAL] %s: %s'%(type(e).__name__,e));traceback.print_exc();print('[SAFETY] No robot/O10 command was issued.');raise SystemExit(1)
