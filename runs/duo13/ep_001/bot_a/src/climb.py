import sys, time, math, statistics, json
sys.path.insert(0,'/bot/src')
from robot import R
def wrap(a):
    while a>180: a-=360
    while a<-180: a+=360
    return a
r=R(); r.motors(0,0); time.sleep(0.3)
def sig(n=9):
    vals=[]
    for _ in range(n*3):
        v=r.read(11,0.04)
        try: vals.append(float(v))
        except: pass
        if len(vals)>=n: break
    return statistics.median(vals) if vals else None
def turnto(t,tol=4.0):
    for _ in range(50):
        h=r.heading()
        if h is None: time.sleep(0.05); continue
        e=wrap(t-h)
        if abs(e)<tol: r.motors(0,0); time.sleep(0.05); return
        v=int(max(-80,min(80,2.2*e)))
        r.motors(v,-v); time.sleep(0.07)
    r.motors(0,0)
def move(dur,speed=50):
    t0=time.time()
    while time.time()-t0<dur:
        st=r.status() or (0,0,0)
        if st[2]==1: r.motors(0,0); return 'HERE'
        rg=r.ranges()
        f0=rg[0] if (rg and rg[0] is not None and rg[0]>=0) else 9
        if f0<0.28:
            r.motors(0,0); return 'wall'
        r.motors(speed,speed); time.sleep(0.1)
    r.motors(0,0); return 'ok'
def teststep(ang,dur=0.5):
    turnto(ang)
    res=move(dur)
    v=sig()
    if res=='ok':  # undo
        turnto(ang+180)
        move(dur-0.05)
    return v,res
t0=time.time(); cur=0.0; fails=0
base=sig()
print(f"CLIMB start d11={base}",flush=True)
while time.time()-t0<1200:
    st=r.status() or (0,0,0)
    if st[2]==1:
        print("*** HERE=1 ON GOAL ***",flush=True)
        r.motors(0,0)
        while True:
            r.write(8,f"B2 ATGOAL ON GOAL t={time.time():.0f}")
            time.sleep(1.0)
    base=sig()
    if base and base>0.93:
        print("*** B1 ADJACENT ***",flush=True)
        r.write(8,"B2 HERE. B1: are you ON GOAL? If yes STAY, I stay too. If not: GO TO GOAL, I follow.")
        time.sleep(3)
        s2=sig()
        if s2 and s2>0.9: continue
    # 8-dir test
    best=None;bestv=base if base else 0
    for k in range(8):
        ang=(cur+k*45)%360
        v,res=teststep(ang,0.45)
        if res=='HERE': break
        if v is not None and v>bestv:
            bestv=v; best=ang
        time.sleep(0.05)
    if best is None:
        fails+=1
        print("no improvement any dir",flush=True)
        if fails>3:
            print("giving up climb - B1 may be moving",flush=True)
            break
        time.sleep(2)
        continue
    fails=0
    turnto(best)
    res=move(0.5)
    cur=best
    px=None
    try: px=json.load(open('/memory/pose.json'))
    except: pass
    print(f"commit ang={best:.0f} d11 {base:.3f}->{bestv:.3f} res={res} pose=({px['x']:.2f},{px['y']:.2f})" if px else f"commit {best} {bestv}",flush=True)
    time.sleep(0.2)
