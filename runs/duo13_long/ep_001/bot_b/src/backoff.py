import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
def enc():
    a=rd("d6"); b=rd("d9")
    return (float(a) if a is not None else 0.0, float(b) if b is not None else 0.0)
e0=enc()
motors(-3,-3)
t0=time.time()
while time.time()-t0<12:
    time.sleep(1)
    print("t=%.0f d0=%s d5=%s b8=%s ticks=%d h=%s" % (time.time()-t0, rd("d0"), rd("d5"), (lidar() or [0]*16)[8], int((enc()[0]-e0[0]+enc()[1]-e0[1])/2), heading()), flush=True)
    if rd("d0")=="0": 
        print("bump cleared", flush=True); break
motors(0,0)
print("h", heading(), lidar(), flush=True)
