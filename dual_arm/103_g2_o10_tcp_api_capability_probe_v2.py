#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""103 v2: G2 GDK 4.1.5 TCP/Tool/Frame API capability probe. READ ONLY."""
from __future__ import annotations
import json, os, shlex, sys, traceback
from datetime import datetime
from pathlib import Path

READ_ONLY_PROBE = True
ENABLE_REAL_MOTION = False
VERSION = "103_v2_api_capability"
ENV_SH = Path.home() / ".cache/agibot/app/env.sh"
REPORT_DIR = Path(__file__).resolve().parent / "reports"
KEYWORDS = ("tcp", "tool", "frame", "transform", "pose", "end")


def ensure_env():
    if os.environ.get("G2_GDK_ENV_READY") == "1": return
    if not ENV_SH.is_file(): raise FileNotFoundError(str(ENV_SH))
    args = " ".join(shlex.quote(x) for x in [str(Path(__file__).resolve())] + sys.argv[1:])
    cmd = "source %s && export G2_GDK_ENV_READY=1 && exec /usr/bin/python3.10 %s" % (shlex.quote(str(ENV_SH)), args)
    print("[ENV] Restarting with official GDK environment ...", flush=True)
    os.execv("/bin/bash", ["bash", "-lc", cmd])


def public_names(obj):
    return sorted(n for n in dir(obj) if not n.startswith("_"))


def related(names):
    return [n for n in names if any(k in n.lower() for k in KEYWORDS)]


def callable_info(obj, names):
    rows=[]
    for n in names:
        try:
            value=getattr(obj,n)
            rows.append({"name":n,"callable":callable(value),"type":type(value).__name__})
        except Exception as e:
            rows.append({"name":n,"error":"%s: %s"%(type(e).__name__,e)})
    return rows


def main():
    ensure_env()
    print("="*92)
    print("G2 O10 TCP/Tool/Frame API Capability Probe")
    print("Version             :", VERSION)
    print("Python executable   :", sys.executable)
    print("READ_ONLY_PROBE     :", READ_ONLY_PROBE)
    print("ENABLE_REAL_MOTION  :", ENABLE_REAL_MOTION)
    print("Motion API          : NONE")
    print("="*92)
    if not READ_ONLY_PROBE or ENABLE_REAL_MOTION: return 90

    import agibot_gdk
    module_path=getattr(agibot_gdk,"__file__","") or ""
    print("[GDK] module path       :", module_path)
    initialized=False
    try:
        r=agibot_gdk.gdk_init(); print("[GDK] gdk_init result   :",r)
        if r != agibot_gdk.GDKRes.kSuccess: raise RuntimeError("gdk_init failed: %s"%r)
        initialized=True
        robot=agibot_gdk.Robot(); tf=agibot_gdk.TF()
        targets={"Robot":robot,"TF":tf,"agibot_gdk":agibot_gdk}
        report={"version":VERSION,"timestamp":datetime.now().astimezone().isoformat(),"gdk_module":module_path,"objects":{}}
        for label,obj in targets.items():
            names=public_names(obj); hits=related(names)
            report["objects"][label]={"related_names":hits,"related_detail":callable_info(obj,hits),"all_public_names":names}
            print(); print("-"*92); print("[%s RELATED API]"%label)
            if hits:
                for n in hits:
                    try: print("  %-42s callable=%s type=%s"%(n,callable(getattr(obj,n)),type(getattr(obj,n)).__name__))
                    except Exception as e: print("  %-42s ERROR=%s"%(n,e))
            else: print("  NONE")

        expected=["get_tcp_transform","set_tcp_transform","get_end_pose_control_frame","switch_end_pose_control_frame"]
        print(); print("-"*92); print("[EXPECTED ROBOT API EXISTENCE]")
        existence={}
        for n in expected:
            exists=hasattr(robot,n); existence[n]=exists; print("  %-40s : %s"%(n,exists))
        report["expected_robot_api_existence"]=existence

        REPORT_DIR.mkdir(parents=True,exist_ok=True)
        rp=REPORT_DIR/("103_tcp_api_capability_v2_%s.json"%datetime.now().strftime("%Y%m%d_%H%M%S"))
        rp.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
        print(); print("="*92); print("PROBE SUMMARY"); print("="*92)
        print("Runtime introspection : PASS")
        print("API calls executed    : introspection only")
        print("TCP modified          : NO")
        print("Motion executed       : NO")
        print("JSON report           :",rp)
        print("="*92)
        return 0
    finally:
        if initialized:
            rr=agibot_gdk.gdk_release(); print(); print("[GDK] gdk_release result :",rr)

if __name__ == "__main__":
    try: raise SystemExit(main())
    except Exception as e:
        print(); print("[FATAL] %s: %s"%(type(e).__name__,e)); traceback.print_exc(); print("[SAFETY] No motion/TCP-setting command was issued."); raise SystemExit(1)
