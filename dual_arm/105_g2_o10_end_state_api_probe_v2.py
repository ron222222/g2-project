#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""105_g2_o10_end_state_api_probe_v2.py
G2 + OmniHand O10 end-state API read-only capability/state probe.
"""
from __future__ import annotations
import inspect, json, os, shlex, sys, time, traceback
from datetime import datetime
from pathlib import Path
from typing import Any

READ_ONLY_PROBE=True
ENABLE_REAL_MOTION=False
VERSION='105_v2_end_state_api_probe'
ENV_SH=Path.home()/'.cache/agibot/app/env.sh'
REPORT_DIR=Path(__file__).resolve().parent/'reports'
WAIT_SEC=2.0


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


def serial(v,depth=0):
    if depth>6:return safe_repr(v)
    if v is None or isinstance(v,(str,int,float,bool)):return v
    if isinstance(v,dict):return {str(k):serial(x,depth+1) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [serial(x,depth+1) for x in v]
    if hasattr(v,'__dict__'):
        try:return {str(k):serial(x,depth+1) for k,x in vars(v).items() if not str(k).startswith('_')}
        except Exception:pass
    # pybind objects may not have __dict__: expose selected public attrs without invoking methods.
    attrs={}
    for name in dir(v):
        if name.startswith('_'):continue
        try:
            value=getattr(v,name)
            if callable(value):continue
            attrs[name]=serial(value,depth+1)
        except Exception:pass
    return {'type':type(v).__name__,'repr':safe_repr(v),'attributes':attrs} if attrs else safe_repr(v)


def api_metadata(method):
    info={'type':type(method).__name__,'repr':safe_repr(method),'doc':getattr(method,'__doc__',None)}
    try:info['signature']=str(inspect.signature(method))
    except Exception as e:info['signature_error']='%s: %s'%(type(e).__name__,e)
    return info


def print_structure(v,label='result',depth=0,max_depth=3):
    indent='  '*depth
    print('%s%s: type=%s repr=%s'%(indent,label,type(v).__name__,safe_repr(v)))
    if depth>=max_depth:return
    if isinstance(v,dict):
        for k,x in v.items():print_structure(x,'[%r]'%k,depth+1,max_depth)
    elif isinstance(v,(list,tuple)):
        print('%s  length=%d'%(indent,len(v)))
        for i,x in enumerate(v):print_structure(x,'[%d]'%i,depth+1,max_depth)
    else:
        for n in ('left','right','left_arm','right_arm','left_hand','right_hand','states','state','position','positions','joint_states','motor_position','motor_current','effort','error_code','name','group','target_type'):
            try:
                if hasattr(v,n):
                    x=getattr(v,n)
                    if not callable(x):print_structure(x,'.'+n,depth+1,max_depth)
            except Exception:pass


def main():
    ensure_env()
    print('='*96); print('G2 O10 End-State API Read-Only Probe'); print('Version             :',VERSION); print('Python executable   :',sys.executable); print('READ_ONLY_PROBE     :',READ_ONLY_PROBE); print('ENABLE_REAL_MOTION  :',ENABLE_REAL_MOTION); print('Motion API          : NONE'); print('='*96)
    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION:return 90
    import agibot_gdk
    initialized=False
    try:
        r=agibot_gdk.gdk_init(); print('[GDK] gdk_init result   :',r)
        if r!=agibot_gdk.GDKRes.kSuccess:raise RuntimeError('gdk_init failed: %s'%r)
        initialized=True; robot=agibot_gdk.Robot(); time.sleep(WAIT_SEC)
        if not hasattr(robot,'get_end_state'):raise AttributeError('Robot.get_end_state not available')
        method=robot.get_end_state; meta=api_metadata(method)
        print(); print('-'*96); print('[get_end_state API METADATA]'); print('type      :',meta.get('type')); print('repr      :',meta.get('repr')); print('signature :',meta.get('signature',meta.get('signature_error'))); print('doc       :',meta.get('doc'))

        # Only call without arguments when metadata/docs or pybind behavior allows it.
        print(); print('-'*96); print('[get_end_state READ]')
        call_status='NOT_CALLED'; result=None; call_error=None
        try:
            result=method(); call_status='PASS'
            print('call status:',call_status); print_structure(result)
        except TypeError as e:
            call_status='ARGUMENT_REQUIRED'; call_error='%s: %s'%(type(e).__name__,e); print('call status:',call_status); print('error      :',call_error)
        except Exception as e:
            call_status='FAILED'; call_error='%s: %s'%(type(e).__name__,e); print('call status:',call_status); print('error      :',call_error)

        report={'version':VERSION,'timestamp':datetime.now().astimezone().isoformat(),'gdk_module':getattr(agibot_gdk,'__file__',None),'api_metadata':meta,'call_status':call_status,'call_error':call_error,'result':serial(result)}
        REPORT_DIR.mkdir(parents=True,exist_ok=True); rp=REPORT_DIR/('105_o10_end_state_api_probe_v2_%s.json'%datetime.now().strftime('%Y%m%d_%H%M%S')); rp.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(); print('='*96); print('PROBE SUMMARY'); print('='*96); print('get_end_state exists  : YES'); print('No-arg read status    :',call_status); print('O10 commanded         : NO'); print('Motion executed       : NO'); print('JSON report           :',rp); print('='*96)
        return 0 if call_status=='PASS' else 2
    finally:
        if initialized:
            rr=agibot_gdk.gdk_release(); print(); print('[GDK] gdk_release result :',rr)

if __name__=='__main__':
    try:raise SystemExit(main())
    except Exception as e:
        print(); print('[FATAL] %s: %s'%(type(e).__name__,e)); traceback.print_exc(); print('[SAFETY] No O10/motion command was issued.'); raise SystemExit(1)
