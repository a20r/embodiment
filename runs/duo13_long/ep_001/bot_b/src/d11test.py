import time, sys, math
sys.path.insert(0,"/bot/src")
from robot import *
def d11(n=7):
    vals=[]
    for _ in range(n):
        v=rd("d11")
        if v: vals.append(float(v))
        time.sleep(0.06)
    return sum(vals)/len(vals) if vals else None
motors(0,0); time.sleep(1)
print("baseline d11=%.3f d5=%s" % (d11(), rd("d5")), flush=True)
# rotate in place full circle, d11 every ~45deg
motors(3,-3)
t0=time.time()
while time.time()-t0<40:
    time.sleep(2.5)
    print("rot h=%.0f d11=%.3f d5=%s" % (heading(), d11(4), rd("d5")), flush=True)
motors(0,0)
