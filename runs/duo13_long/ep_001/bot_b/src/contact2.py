import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
motors(0,0)
t0=time.time(); msg_n=0; mn=9
while time.time()-t0<110:
    msg_n+=1
    wr("d8","HELLO %d"%msg_n)
    r=rd("d10",timeout=0.3)
    if r:
        print("RX: %s"%r, flush=True)
        with open("/bot/src/rx.log","a") as f: f.write("%.1f %s\n"%(time.time(),r))
    v=rd("d11")
    try: mn=min(mn,float(v))
    except: pass
    if int(time.time()-t0)%8==0:
        print("t=%.0f d11=%s min=%.3f d3=%s d5=%s"%(time.time()-t0,v,mn,rd("d3"),rd("d5")), flush=True)
        time.sleep(0.9)
