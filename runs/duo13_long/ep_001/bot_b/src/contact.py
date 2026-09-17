import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
motors(0,0)
t0=time.time()
msg_n=0
while time.time()-t0<75:
    msg_n+=1
    wr("d8","HELLO %d"%msg_n)
    r=rd("d10",timeout=0.35)
    if r:
        print("RX: %s"%r, flush=True)
        with open("/bot/src/rx.log","a") as f: f.write("%.1f %s\n"%(time.time(),r))
    if int(time.time()-t0)%5==0:
        print("t=%.0f d11=%s d5=%s d3=%s"%(time.time()-t0,rd("d11"),rd("d5"),rd("d3")), flush=True)
        time.sleep(0.8)
