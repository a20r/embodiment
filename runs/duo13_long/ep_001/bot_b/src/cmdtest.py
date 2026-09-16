import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
def d11(n=5):
    vals=[]
    for _ in range(n):
        v=rd("d11")
        if v:
            try: vals.append(float(v))
            except: pass
        time.sleep(0.03)
    return sum(vals)/len(vals) if vals else 9
t0=time.time()
print("phase1 baseline (no cmd):", flush=True)
while time.time()-t0<15:
    print("  d11=%.3f d3=%s"%(d11(4), rd("d3")), flush=True); time.sleep(2)
print("phase2 sending A CMD STOP:", flush=True)
t1=time.time()
while time.time()-t1<25:
    wr("d8","A CMD STOP")
    r=rd("d10",timeout=0.15)
    if r: print("  RX: %s"%r, flush=True)
    print("  d11=%.3f"%(d11(4)), flush=True); time.sleep(1.5)
print("phase3 sending A CMD RESUME:", flush=True)
t2=time.time()
while time.time()-t2<12:
    wr("d8","A CMD RESUME")
    r=rd("d10",timeout=0.15)
    if r: print("  RX: %s"%r, flush=True)
    print("  d11=%.3f"%(d11(4)), flush=True); time.sleep(1.5)
