import sys, time, math
sys.path.insert(0,'/bot/src')
from robot import R
def wrap(a):
    while a>180: a-=360
    while a<-180: a+=360
    return a
r=R(); r.motors(0,0); time.sleep(0.3)
def turnto(target,tol=3.0):
    for _ in range(70):
        h=r.heading()
        if h is None: time.sleep(0.05); continue
        e=wrap(target-h)
        if abs(e)<tol: r.motors(0,0); time.sleep(0.08); return True
        v=int(max(-80,min(80,2.2*e)))
        r.motors(v,-v); time.sleep(0.07)
    r.motors(0,0); return False
def beacon():
    c=0
    for _ in range(3):
        if r.read(5,0.04)=='1': c+=1
        if r.read(0,0.04)=='1': c+=1
    return c
t0=time.time()
best_h=None; best_c=0
hunt_ang=None
while time.time()-t0<600:
    st=r.status() or (0,0,0)
    if st[2]==1:
        print("*** HERE=1 ***",flush=True)
        r.motors(0,0)
        while True:
            r.write(8,f"B2 ATGOAL ON GOAL t={time.time():.0f}")
            time.sleep(1.0)
    # 1) find bearing with strongest beacon
    h0=r.heading()
    if h0 is None: time.sleep(0.1); continue
    base=h0 if hunt_ang is None else hunt_ang
    best_h=None; best_c=0
    for k in range(12):
        ang=base+k*30
        turnto(ang)
        c=beacon()
        if c>best_c: best_c=c; best_h=ang
        if time.time()-t0>600: break
    print(f"scan done: best_h={best_h} count={best_c}",flush=True)
    hunt_ang=best_h
    if best_c==0:
        # no beacon: creep forward 0.3 in last dir anyway
        best_h=(r.heading() or 0)
    # 2) creep toward beacon 0.3m
    turnto(best_h,tol=3)
    e0=r.enc()
    drive_t=0; moved=0
    while drive_t<3.5:
        st=r.status() or (0,0,0)
        if st[2]==1: break
        rg=r.ranges()
        f0=rg[0] if (rg and rg[0] is not None and rg[0]>=0) else 9
        if f0<0.26:
            print("wall ahead",flush=True); break
        err=wrap(best_h-(r.heading() or best_h))
        a=int(max(-40,min(90, 55+1.5*err)))
        r.motors(a,a); time.sleep(0.1); drive_t+=0.1
        c=beacon()
        if c>=2:
            moved+=1
            if moved>8: break  # strong beacon - stop and rescan
        else: moved=0
    r.motors(0,0)
    print(f"crept toward {best_h}, t={time.time()-t0:.0f}",flush=True)
    r.write(8,f"B2 HUNTING GOAL t={time.time():.0f}")
    time.sleep(0.3)
