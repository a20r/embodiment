import time, math
from rob import *
from nav import rot_to, d5val

AXES=[35,125,215,305]
def near_axis(h):
    return min(AXES, key=lambda a: abs((h-a+180)%360-180))
def front(): return ranges()[2]

def align(nominal, span=14, step=3):
    """sweep heading around nominal, maximize front beam range"""
    best=(-1,nominal)
    a=nominal-span
    rot_to(a, tol=3)
    while a<=nominal+span:
        rot_to(a, tol=2.5)
        time.sleep(0.15)
        fs=[]
        for _ in range(3):
            f=front()
            if f>0: fs.append(f)
            time.sleep(0.08)
        if fs:
            m=sum(fs)/len(fs)
            if m>best[0]: best=(m,a)
        a+=step
    rot_to(best[1], tol=2.5)
    return best

def drive(tmax=8, dist_stop=0.32):
    t0=time.time(); f0=front(); fmin=f0; lastprog=t0
    while time.time()-t0<tmax:
        motors(7,7); time.sleep(0.15)
        st=rd('d6')
        if 'here=1' in st or 'goal=1' in st: stop(); return 0,'HERE'
        f=front()
        if f<0: continue
        if f<dist_stop: stop(); return f0-f,'wall'
        if f<fmin-0.05: fmin=f; lastprog=time.time()
        if f>fmin+0.5: fmin=f; lastprog=time.time()
        if time.time()-lastprog>1.3:
            return f0-f,'stuck'
    stop(); return f0-front(),'time'
