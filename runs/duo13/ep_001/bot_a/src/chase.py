import sys, time, math, statistics
sys.path.insert(0,'/bot/src')
from robot import R
def wrap(a):
    while a>180: a-=360
    while a<-180: a+=360
    return a
r=R(); r.motors(0,0); time.sleep(0.3)
def sig():
    vals=[]
    for _ in range(14):
        v=r.read(11,0.04)
        try: vals.append(float(v))
        except: pass
        if len(vals)>=6: break
    return statistics.median(vals) if vals else None
def turnto(target,tol=4.0):
    for _ in range(60):
        h=r.heading()
        if h is None: time.sleep(0.05); continue
        e=wrap(target-h)
        if abs(e)<tol: r.motors(0,0); time.sleep(0.06); return
        v=int(max(-80,min(80,2.2*e)))
        r.motors(v,-v); time.sleep(0.07)
    r.motors(0,0)
def step(ang,dur=0.85):
    turnto(ang)
    t0=time.time(); e0=r.enc()
    while time.time()-t0<dur:
        st=r.status() or (0,0,0)
        if st[2]==1: return 'HERE'
        rg=r.ranges()
        f0=rg[0] if (rg and rg[0] is not None and rg[0]>=0) else 9
        if f0<0.27:
            r.motors(0,0); return 'wall'
        err=wrap(ang-(r.heading() or ang))
        a=int(max(-40,min(90, 55+1.5*err)))
        r.motors(a,a); time.sleep(0.1)
    r.motors(0,0)
    return 'ok'
t0=time.time()
base_sig=sig()
print(f"chase start base d11={base_sig}",flush=True)
cur_ang=0.0
while time.time()-t0<480:
    st=r.status() or (0,0,0)
    if st[2]==1:
        print("*** HERE=1 ***",flush=True)
        r.motors(0,0)
        while True:
            r.write(8,f"B2 ATGOAL ON GOAL t={time.time():.0f}")
            time.sleep(1.0)
    # try 8 directions with a test step, keep the best d11
    best=None; bestv=-1; base=sig()
    for k in range(8):
        ang=cur_ang+k*45
        res=step(ang,0.6)
        if res=='HERE': break
        v=sig()
        adv = 1 if v is not None else 0
        score = v if v is not None else -1
        if score>bestv: bestv=score; best=ang
        # undo: step back
        if res=='ok':
            step(ang+180,0.55)
        elif res=='wall':
            pass
        time.sleep(0.1)
    if best is None:
        cur_ang+=45; continue
    # commit: 2 steps toward best
    res=step(best,0.9)
    cur_ang=best
    print(f"committed ang={best%360:.0f} d11 {base}->{bestv}",flush=True)
    r.write(8,f"B2 CHASING d11={bestv:.3f} t={time.time():.0f}")
    if res=='HERE': continue
    time.sleep(0.2)
