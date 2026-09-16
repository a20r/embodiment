import time,sys,os,subprocess
sys.path.insert(0,"/bot/src")
from robot import *
E=open("/bot/src/EVENTS.log","a",buffering=1)
def ev(s): E.write("%.1f %s\n"%(time.time(),s))
last_rx_size=os.path.getsize("/bot/src/rx.log")
last_alive=0
d11hi=0
while True:
    try:
        # goal trigger
        if os.path.exists("/bot/src/GOALALERT.txt") and not getattr(watch_events,"seen_alert",False):
            watch_events.seen_alert=True
            ev("GOALALERT: "+open("/bot/src/GOALALERT.txt").read()[-200:])
        # A radio returns
        sz=os.path.getsize("/bot/src/rx.log")
        if sz>last_rx_size:
            new=open("/bot/src/rx.log").read()[last_rx_size:]
            last_rx_size=sz
            for ln in new.splitlines():
                if "A PING" in ln or "A HERE" in ln:
                    ev("RX: "+ln[:120])
        # explorer alive
        r=subprocess.run(["pgrep","-f","python3 /bot/src/explore6.py"],capture_output=True,text=True)
        if r.stdout.strip():
            last_alive=time.time()
        elif time.time()-last_alive>90:
            ev("EXPLORE6 DEAD >90s (keepalive should fix; check keepalive.log)")
            last_alive=time.time()
        # d11 drift (A leaving)
        v=rd("d11",timeout=0.2)
        try:
            f=float(v)
            if f>0.8:
                d11hi+=1
                if d11hi==5: ev("d11>0.8 x5: A FALLING BEHIND d11=%.2f"%f)
            else: d11hi=0
        except: pass
        # here/goal direct check (belt & braces with goalseek)
        s=rd("d3",timeout=0.1) or ""
        if "here=1" in s or "goal=1" in s:
            ev("D3 FLAG! "+s)
        time.sleep(2)
    except Exception as e:
        ev("ERR %s"%e); time.sleep(2)
