import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
def enc():
    a=rd("d6"); b=rd("d9")
    return (float(a) if a is not None else 0.0, float(b) if b is not None else 0.0)
# rotate to heading ~145 (CCW)
h=heading()
dh=(145-h+540)%360-180
rate=6
t=abs(dh)/(2.0*rate)
if dh>0: motors(rate,-rate)
else: motors(-rate,rate)
time.sleep(t); motors(0,0); time.sleep(0.5)
print("now h=%.1f" % heading(), flush=True)
e0=enc()
motors(2,2)
t0=time.time()
while time.time()-t0<25:
    time.sleep(0.5)
    l=lidar(); e=enc()
    if l: print("t=%.1f b8=%.3f ticks=%d" % (time.time()-t0, l[8], int((e[0]-e0[0]+e[1]-e0[1])/2)), flush=True)
    if l and l[8]<0.28: break
motors(0,0)
print("final", lidar(), flush=True)
