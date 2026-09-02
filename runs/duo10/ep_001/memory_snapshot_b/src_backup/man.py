import time
from rob import *
from nav import rot_to, havg, d5val
def fsweep(center, span=25, dur=8):
    """fine sweep front beam around center heading, return argmax"""
    rot_to(center-span, tol=4)
    data=[]
    motors(5,-5)  # slow CW ~10deg/s
    t0=time.time()
    while time.time()-t0<dur+span/5:
        h=heading(); f=ranges()[2]
        if f>0: data.append((h,f))
        time.sleep(0.08)
        if (h-(center+span))%360<30 and time.time()-t0>2: break
    stop()
    if not data: return center,0
    # max range, tie-break near center
    m=max(v for h,v in data)
    cands=[h for h,v in data if v>m-0.1]
    import math
    x=sum(math.cos(math.radians(c)) for c in cands); y=sum(math.sin(math.radians(c)) for c in cands)
    best=math.degrees(math.atan2(y,x))%360
    return best,m
def fb():
    r=ranges(); return r[2],r[10]
def push(nom, tmax=30, dist_stop=0.30):
    t0=time.time(); moved=0.0
    variants=[(7,1.2),(-7,0.35),(7,1.2),(4,1.0),(7,1.2),(10,1.0),(4,1.0)]
    vi=0; f0,b0=fb()
    while time.time()-t0<tmax:
        v,dur=variants[vi%len(variants)]; vi+=1
        motors(v,v); time.sleep(dur)
        st=rd('d6')
        if 'here=1' in st or 'goal=1' in st: stop(); return 'HERE',moved
        f1,b1=fb()
        if 0<f1<dist_stop: stop(); return 'wall',moved
        d=0
        if f0>0 and f1>0 and f0<2.9: d=f0-f1
        if b0>0 and b1>0 and b1<2.9: d=max(d,b1-b0)
        if v>0 and d>0.15:
            moved+=d; fmin=f1 if f1>0 else 3.0; last=time.time()
            while time.time()-t0<tmax:
                motors(v,v); time.sleep(0.13)
                st=rd('d6')
                if 'here=1' in st: stop(); return 'HERE',moved
                fc,bc=fb()
                if fc<0: continue
                if fc<dist_stop: stop(); return 'wall',moved+max(0,fmin-fc)
                if fc<fmin-0.05: moved+=fmin-fc; fmin=fc; last=time.time()
                if fc>fmin+0.4: fmin=fc; last=time.time()
                if time.time()-last>1.6: break
            stop()
            f0,b0=fb(); vi=0; continue
        f0,b0=f1,b1
    stop(); return 'end',moved
