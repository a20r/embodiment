import sys, time, math, json
sys.path.insert(0,'/bot/src')
from robot import R
GOALP=(2.00,-0.62)   # goal in posetrack frame (from B1 coords via frame chain)
def wrap(a):
    while a>180: a-=360
    while a<-180: a+=360
    return a
r=R(); r.motors(0,0); time.sleep(0.3)
def pose():
    try:
        j=json.load(open('/memory/pose.json'))
        return j['x'],j['y']
    except: return None,None
def turnto(t,tol=4.0):
    for _ in range(60):
        h=r.heading()
        if h is None: time.sleep(0.05); continue
        e=wrap(t-h)
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
        # live bearing correction
        px,py=pose()
        if px is not None:
            b=math.degrees(math.atan2(GOALP[1]-py,GOALP[0]-px))
            err=wrap(b-(r.heading() or b))
        else:
            err=0
        a=int(max(-40,min(95, 60+1.6*err)))
        r.motors(a,a); time.sleep(0.1)
    r.motors(0,0); return 'ok'
t0=time.time(); lastdist=None; lastprog=time.time(); side=1
print("RUSH2 to",GOALP,flush=True)
while time.time()-t0<1500:
    st=r.status() or (0,0,0)
    if st[2]==1:
        print("*** HERE=1 ON GOAL ***",flush=True)
        r.motors(0,0)
        while True:
            r.write(8,f"B2 ATGOAL ON GOAL t={time.time():.0f}")
            time.sleep(1.0)
    px,py=pose()
    if px is None: time.sleep(0.2); continue
    dist=math.hypot(GOALP[0]-px,GOALP[1]-py)
    bear=math.degrees(math.atan2(GOALP[1]-py,GOALP[0]-px))
    print(f"t={time.time()-t0:.0f} dist={dist:.2f} bear={bear%360:.0f} pose=({px:.2f},{py:.2f})",flush=True)
    if dist<0.30:
        # close: precise local search
        print("CLOSE - local search",flush=True)
        r.write(8,"B2 AT GOAL AREA - holding")
        ang=0; found=False
        for i in range(40):
            st=r.status() or (0,0,0)
            if st[2]==1: found=True; break
            turnto(bear+ang); 
            rg=r.ranges()
            f0=rg[0] if (rg and rg[0] is not None and rg[0]>=0) else 9
            if f0>0.3:
                te=time.time()
                while time.time()-te<0.5:
                    s2=r.status() or (0,0,0)
                    if s2[2]==1: found=True; break
                    r.motors(50,50); time.sleep(0.1)
                r.motors(0,0)
            ang+=60
            if found: break
        if found:
            print("*** HERE=1 ON GOAL (local) ***",flush=True)
            r.motors(0,0)
            while True:
                r.write(8,f"B2 ATGOAL ON GOAL t={time.time():.0f}")
                time.sleep(1.0)
    # progress tracking
    if lastdist is not None and lastdist-dist>0.15:
        lastprog=time.time()
    lastdist=dist
    if time.time()-lastprog>14:
        print("no progress - breakout",flush=True)
        turnto(bear+130*side); drive(1.4)
        side*=-1
        lastprog=time.time()
        continue
    turnto(bear)
    res=drive(1.1)
    if res=='HERE': continue
    if res=='wall':
        rg=r.ranges()
        turnto(bear+75*side); 
        r2=drive(0.7)
        if r2=='wall':
            turnto(bear-75*side); drive(0.9)
        side*=-1
        r.motors(-55,-55); time.sleep(0.35); r.motors(0,0)
