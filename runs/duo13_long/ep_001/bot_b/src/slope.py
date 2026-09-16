import time, sys, math
sys.path.insert(0,"/bot/src")
from robot import *
def d11(n=6):
    vals=[]
    for _ in range(n):
        v=rd("d11")
        if v: vals.append(float(v))
        time.sleep(0.05)
    return sum(vals)/len(vals) if vals else None
def seth(target):
    for _ in range(25):
        h=heading()
        dh=(target-h+540)%360-180
        if abs(dh)<7: break
        rate=4; t=min(abs(dh)/(2.2*rate),1.4)
        if dh>0: motors(rate,-rate)
        else: motors(-rate,rate)
        time.sleep(t); motors(0,0); time.sleep(0.2)
def drive(h, dur):
    seth(h); time.sleep(0.5)
    a=d11(8)
    motors(6,6); t0=time.time(); mn=9; mx=0
    while time.time()-t0<dur:
        time.sleep(1.5); v=d11(4); mn=min(mn,v); mx=max(mx,v)
    motors(0,0); time.sleep(0.5)
    b=d11(8)
    print("h=%d: d11 %.3f -> %.3f (slope %+.4f/s) min=%.3f max=%.3f d5=%s d3=%s" % (h,a,b,(b-a)/dur,mn,mx,rd("d5"),rd("d3")), flush=True)
drive(330,22)
drive(150,22)
drive(225,22)
drive(45,22)
