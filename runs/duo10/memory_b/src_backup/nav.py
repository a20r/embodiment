import time, math
from rob import *

def d5val(n=6):
    v=[]
    for _ in range(n):
        try: v.append(float(rd('d5')))
        except: pass
        time.sleep(0.08)
    return sum(v)/len(v)

def havg(n=3):
    s=0.0; c=0
    import math
    x=y=0.0
    for _ in range(n):
        h=math.radians(heading()); x+=math.cos(h); y+=math.sin(h)
        time.sleep(0.06)
    return math.degrees(math.atan2(y,x))%360

def rot_to(tgt, tol=4, tmax=14):
    t0=time.time()
    for _ in range(10):
        if time.time()-t0>tmax: stop(); return False
        h=havg()
        err=(tgt-h+180)%360-180
        if abs(err)<tol: stop(); return True
        s=10 if abs(err)>8 else 7
        rate=2.0*s  # deg/s approx
        dur=min(2.5, abs(err)/rate)
        if err>0: motors(s,-s)
        else: motors(-s,s)
        time.sleep(dur); stop(); time.sleep(0.15)
    return abs((tgt-havg()+180)%360-180)<tol*1.5

def front(): return ranges()[2]

def drive_hold(tgt_h, dist_stop=0.34, tmax=10, base=9):
    t0=time.time(); f0=front(); lastprog=t0; fmin=f0
    while time.time()-t0<tmax:
        h=heading(); err=(tgt_h-h+180)%360-180
        corr=max(-4,min(4,0.4*err))
        motors(round(base+corr,1), round(base-corr,1))
        time.sleep(0.12)
        f=front()
        if f<0: continue
        if f<dist_stop: stop(); return f0-f,'wall'
        if f<fmin-0.04: fmin=f; lastprog=time.time()
        if f>fmin+0.5: fmin=f; lastprog=time.time()  # opened
        if time.time()-lastprog>2.2: stop(); return f0-f,'stuck'
    stop(); return f0-front(),'time'

def unstick(tgt_h):
    motors(-10,-10); time.sleep(0.9); stop(); time.sleep(0.2)
    rot_to(tgt_h)
