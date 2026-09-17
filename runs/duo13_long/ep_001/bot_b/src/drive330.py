import time, sys, math
sys.path.insert(0,"/bot/src")
from robot import *
def d11(n=5):
    vals=[]
    for _ in range(n):
        v=rd("d11")
        if v: vals.append(float(v))
        time.sleep(0.05)
    return sum(vals)/len(vals) if vals else 9
def havg(n=4):
    vals=[heading() for _ in range(n)]
    s=sum(math.sin(math.radians(v)) for v in vals)/n
    c=sum(math.cos(math.radians(v)) for v in vals)/n
    return math.degrees(math.atan2(s,c))%360
def lid():
    for _ in range(3):
        l=lidar()
        if l: return l
        time.sleep(0.05)
def seth(target):
    for _ in range(22):
        h=havg(); dh=(target-h+540)%360-180
        if abs(dh)<8: return True
        if rd("d0")=="1": 
            motors(-3,-3); time.sleep(0.6); motors(0,0); time.sleep(0.3); continue
        rate=4; t=min(abs(dh)/(2.2*rate),1.2)
        if dh>0: motors(rate,-rate)
        else: motors(-rate,rate)
        time.sleep(t); motors(0,0); time.sleep(0.15)
    return False
t0=time.time()
h=330
while time.time()-t0<50:
    if rd("d0")=="1":
        print("BUMP d11=%.3f" % d11(4), flush=True)
        motors(-4,-4); time.sleep(0.8); motors(0,0); time.sleep(0.3)
    else:
        seth(h)
        l=lid(); b8=l[8] if l else 0
        if b8>0.4: 
            motors(7,7); time.sleep(1.0)
        else:
            motors(4,4); time.sleep(0.6)
    print("t=%.0f h=%.0f d11=%.3f d5=%s d0=%s d3=%s b8=%s" % (time.time()-t0, havg(), d11(4), rd("d5"), rd("d0"), rd("d3"), (lid() or [0]*16)[8]), flush=True)
motors(0,0)
