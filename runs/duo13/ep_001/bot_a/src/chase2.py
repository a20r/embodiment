import sys, time, math, statistics, random
sys.path.insert(0,'/bot/src')
from robot import R
def wrap(a):
    while a>180: a-=360
    while a<-180: a+=360
    return a
r=R(); r.motors(0,0); time.sleep(0.3)
def sig():
    vals=[]
    for _ in range(16):
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
def drive(dur):
    t0=time.time()
    while time.time()-t0<dur:
        st=r.status() or (0,0,0)
        if st[2]==1: return 'HERE'
        rg=r.ranges()
        f0=rg[0] if (rg and rg[0] is not None and rg[0]>=0) else 9
        if f0<0.27:
            r.motors(0,0); return 'wall'
        h=r.heading() or 0
        r.motors(60,60); time.sleep(0.1)
    r.motors(0,0); return 'ok'
t0=time.time(); cur=0.0; stall=0; bestever=0
lastreq=0
print("CHASE2 start d11=",sig(),flush=True)
while time.time()-t0<900:
    now=time.time()
    if now-lastreq>25:
        lastreq=now
        r.write(8,f"B2 meeting you! d11={sig()} t={now:.0f}. B1: reply GOAL bearing+dist from me!")
    st=r.status() or (0,0,0)
    if st[2]==1:
        print("*** HERE=1 ON GOAL ***",flush=True)
        r.motors(0,0)
        while True:
            r.write(8,f"B2 ATGOAL ON GOAL t={time.time():.0f}")
            time.sleep(1.0)
    base=sig()
    if base is None: time.sleep(0.2); continue
    if base>bestever: bestever=base; stall=0
    else: stall+=1
    if base>0.93:
        print("B1 adjacent - hold",flush=True)
        r.motors(0,0); time.sleep(1.5)
        continue
    # test 4 orthogonal dirs
    results=[]
    for k in range(4):
        ang=(cur+k*90)%360
        turnto(ang)
        res=drive(0.55)
        v=sig()
        results.append((v if v is not None else -1, ang, res))
        if res=='HERE': break
        if res=='ok':
            turnto((ang+180)%360); drive(0.5)  # undo
        time.sleep(0.05)
    results.sort(reverse=True)
    bv,bang,bres=results[0]
    if bres=='HERE': continue
    if bv<=base+0.005:
        stall+=1
    # commit to best (or random if stalling)
    if stall>3:
        bang=(cur+random.choice([45,135,225,315]))%360
        print("stall - random jump",flush=True)
        stall=0
    turnto(bang)
    res=drive(1.2)
    cur=bang
    print(f"t={now-t0:.0f} base={base:.3f} best={bv:.3f} ang={bang:.0f} res={res} stall={stall}",flush=True)
    if res=='wall':
        turnto((bang+60)%360); drive(0.6)
