import sys, time, math
sys.path.insert(0,'/bot/src')
from robot import R
TARGET=347.0
def wrap(a):
    while a>180: a-=360
    while a<-180: a+=360
    return a
r=R()
def turnto(target,tol=4.0):
    for _ in range(60):
        h=r.heading()
        if h is None: time.sleep(0.05); continue
        e=wrap(target-h)
        if abs(e)<tol: r.motors(0,0); time.sleep(0.06); return
        v=int(max(-80,min(80,2.2*e)))
        r.motors(v,-v); time.sleep(0.07)
    r.motors(0,0)
def drive(dur):
    t0=time.time()
    while time.time()-t0<dur:
        st=r.status() or (0,0,0)
        if st[2]==1: return 'HERE'
        rg=r.ranges()
        f0=rg[0] if (rg and rg[0] is not None and rg[0]>=0) else 9
        if f0<0.27:
            r.motors(0,0); return 'wall'
        err=wrap(TARGET-(r.heading() or TARGET))
        a=int(max(-40,min(95, 60+1.6*err)))
        r.motors(a,a); time.sleep(0.1)
    r.motors(0,0); return 'ok'
t0=time.time()
sidesteps=0
while time.time()-t0<420:
    st=r.status() or (0,0,0)
    if st[2]==1:
        print("*** HERE=1 ON GOAL ***",flush=True)
        r.motors(0,0)
        while True:
            r.write(8,f"B2 ATGOAL ON GOAL NOW t={time.time():.0f}")
            time.sleep(1.0)
    turnto(TARGET)
    res=drive(1.6)
    if res=='HERE': continue
    if res=='wall':
        print("wall - sidestep",flush=True)
        turnto(TARGET+60); drive(0.8)
        turnto(TARGET-60); drive(0.4)
        sidesteps+=1
        if sidesteps>14:
            TARGET=(TARGET+90)%360
            print("too many walls, new target heading",TARGET,flush=True)
            sidesteps=0
    print(f"t={time.time()-t0:.0f} res={res}",flush=True)
