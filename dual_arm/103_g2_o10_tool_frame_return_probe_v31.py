#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""103 v3.1 - inspect lookup_transform_latest tuple structure. READ ONLY."""
from __future__ import annotations
import json, os, shlex, sys, time, traceback
from datetime import datetime
from pathlib import Path

READ_ONLY_PROBE=True
ENABLE_REAL_MOTION=False
VERSION='103_v3.1_return_structure'
ENV_SH=Path.home()/'.cache/agibot/app/env.sh'
REPORT_DIR=Path(__file__).resolve().parent/'reports'
PAIRS=(('arm_l_end_link','left_tool0_body'),('arm_r_end_link','right_tool0_body'))

def ensure_env():
    if os.environ.get('G2_GDK_ENV_READY')=='1': return
    if not ENV_SH.is_file(): raise FileNotFoundError(str(ENV_SH))
    args=' '.join(shlex.quote(x) for x in [str(Path(__file__).resolve())]+sys.argv[1:])
    cmd='source %s && export G2_GDK_ENV_READY=1 && exec /usr/bin/python3.10 %s'%(shlex.quote(str(ENV_SH)),args)
    print('[ENV] Restarting with official GDK environment ...',flush=True)
    os.execv('/bin/bash',['bash','-lc',cmd])

def safe_repr(v):
    try:return repr(v)
    except Exception as e:return '<repr failed: %s>'%e

def inspect_value(v,depth=0):
    out={'type':type(v).__name__,'repr':safe_repr(v)}
    if depth>=2:return out
    if isinstance(v,(tuple,list)):
        out['length']=len(v); out['items']=[inspect_value(x,depth+1) for x in v]
    else:
        attrs={}
        for n in ('translation','rotation','transform','timestamp_ns','frame_id','child_frame_id','x','y','z','w'):
            try:
                if hasattr(v,n): attrs[n]=inspect_value(getattr(v,n),depth+1)
            except Exception as e: attrs[n]={'error':'%s: %s'%(type(e).__name__,e)}
        if attrs:out['selected_attributes']=attrs
    return out

def main():
    ensure_env()
    print('='*92); print('G2 O10 Tool Frame Return-Structure Probe'); print('Version             :',VERSION); print('Python executable   :',sys.executable); print('READ_ONLY_PROBE     :',READ_ONLY_PROBE); print('ENABLE_REAL_MOTION  :',ENABLE_REAL_MOTION); print('Motion API          : NONE'); print('='*92)
    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION:return 90
    import agibot_gdk
    initialized=False
    try:
        r=agibot_gdk.gdk_init(); print('[GDK] gdk_init result   :',r)
        if r!=agibot_gdk.GDKRes.kSuccess:raise RuntimeError('gdk_init failed: %s'%r)
        initialized=True; tf=agibot_gdk.TF(); time.sleep(2.0)
        report={'version':VERSION,'timestamp':datetime.now().astimezone().isoformat(),'results':{}}
        for parent,child in PAIRS:
            print(); print('-'*92); print('[LOOKUP]',parent,'->',child)
            result=tf.lookup_transform_latest(parent,child)
            info=inspect_value(result); report['results'][parent+' -> '+child]=info
            print('  return type :',type(result).__name__)
            if isinstance(result,(tuple,list)):
                print('  length      :',len(result))
                for i,item in enumerate(result):
                    print('  item[%d] type: %s'%(i,type(item).__name__))
                    print('  item[%d] repr: %s'%(i,safe_repr(item)))
                    for attr in ('translation','rotation','transform','timestamp_ns','frame_id','child_frame_id'):
                        if hasattr(item,attr):
                            try: print('    .%s: %s'%(attr,safe_repr(getattr(item,attr))))
                            except Exception as e: print('    .%s: ERROR %s'%(attr,e))
            else:
                print('  repr        :',safe_repr(result))
        REPORT_DIR.mkdir(parents=True,exist_ok=True); rp=REPORT_DIR/('103_tool_frame_return_probe_v31_%s.json'%datetime.now().strftime('%Y%m%d_%H%M%S')); rp.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(); print('='*92); print('PROBE SUMMARY'); print('='*92); print('Return structure read : PASS'); print('Tool/TCP modified     : NO'); print('Motion executed       : NO'); print('JSON report           :',rp); print('='*92)
        return 0
    finally:
        if initialized:
            rr=agibot_gdk.gdk_release(); print(); print('[GDK] gdk_release result :',rr)
if __name__=='__main__':
    try:raise SystemExit(main())
    except Exception as e:
        print(); print('[FATAL] %s: %s'%(type(e).__name__,e)); traceback.print_exc(); print('[SAFETY] No motion/TCP-setting command was issued.'); raise SystemExit(1)
