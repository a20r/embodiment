import sys,time,math,statistics,json,re
sys.path.insert(0,'/bot/src')
from robot import R
r=R(); r.motors(0,0)
LOG=open('/memory/home2.out','a',buffering=1)
def log(s):
    LOG.write(f"[{time.time()-T0:.0f}] {s}\n")
def med(n=9):
    vals=[]
    for _ in range(n*3):
        v=r.read(11,0.06)
        try: vals.append(float(v))
        except: pass
        if len(vals)>=n: break
        time.sleep(0.04)
    return statistics.median(vals) if vals else None
def wrap(a):
    while a>180:a-=360
    while a<-180:a+=360
    return a
def turnto(t,tol=5):
    for _ in range(50):
        h=r.heading()
        if h is None: time.sleep(0.05); continue
        e=wrap(t-h)
        if abs(e)<tol: r.motors(0,0); time.sleep(0.05); return
        v=int(max(-70,min(70,2.0*e)))
        r.motors(v,-v); time.sleep(0.06)
    r.motors(0,0)
def fwd(dist,speed=70,maxt=5.0):
    t0=time.time(); e0=r.enc()
    while time.time()-t0<maxt:
        st=r.status() or (0,0,0)
        if st[2]==1: r.motors(0,0); return 'HERE'
        rg=r.ranges()
        f=[rg[i] for i in (0,15,1) if rg and rg[i] is not None and rg[i]>=0]
        if f and min(f)<0.22: break
        r.motors(speed,speed); time.sleep(0.08)
    r.motors(0,0); time.sleep(0.15)
    e1=r.enc()
    if e1[0] is None or e0[0] is None: return 0
    return (e1[0]-e0[0]+e1[1]-e0[1])/2.0*0.000503
def back(dist,speed=60,maxt=5.0):
    t0=time.time(); e0=r.enc()
    while time.time()-t0<maxt:
        rg=r.ranges()
        b=[rg[i] for i in (8,7,9) if rg and rg[i] is not None and rg[i]>=0]
        if b and min(b)<0.15: break
        r.motors(-speed,-speed); time.sleep(0.08)
    r.motors(0,0); time.sleep(0.15)
    e1=r.enc()
    if e1[0] is None or e0[0] is None: return 0
    return -(e1[0]-e0[0]+e1[1]-e0[1])/2.0*0.000503
def pose():
    try:
        j=json.load(open('/memory/pose.json')); return j['x'],j['y']
    except: return None,None
def checkvec():
    try: txt=open('/memory/rx_all.log').read()
    except: return None
    hits=re.findall(r'B2VEC\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)',txt)
    if hits:
        dx,dy=float(hits[-1][0]),float(hits[-1][1])
        return ('VEC',dx,dy)
    hits=re.findall(r'B2BRG\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)',txt)
    if hits:
        return ('BRG',float(hits[-1][0]),float(hits[-1][1]))
    return None
T0=time.time()
best=med() or 0
log(f"HOME2 start d11={best:.3f}")
step=0.4; lastvecn=0
while time.time()-T0<3300:
    st=r.status() or (0,0,0)
    if st[2]==1:
        log("*** HERE=1 ON GOAL ***")
        r.motors(0,0)
        while True:
            r.write(8,f"B2 ATGOAL ON GOAL t={time.time():.0f}"); time.sleep(1)
    v=checkvec()
    if v:
        log(f"VECTOR MODE {v}")
        break
    cur=med()
    if cur is None: time.sleep(0.3); continue
    log(f"cur={cur:.3f} step={step:.2f}")
    if cur>=0.93:
        log("ADJACENT - hold+ping")
        r.write(8,f"B2 HERE NEXT TO YOU d11={cur:.3f} SPEAK")
        time.sleep(2.0); continue
    if time.time()-T0>45 and int(time.time()-T0)%45<2:
        r.write(8,f"B2 HOMING-TO-YOU d11={cur:.3f}")
    improved=False
    h0=r.heading() or 0
    for d in (0,45,-45,90,-90,135,-135,180):
        turnto(h0+d)
        m=fwd(step)
        if m=='HERE': break
        if isinstance(m,float) and m<0.12:  # blocked
            continue
        after=med()
        if after is not None and after>cur+0.006:
            log(f"  commit d={d} {cur:.3f}->{after:.3f}")
            cur=after; improved=True; step=max(0.4,step*0.9)
            break
        else:
            b=back(step)
            turnto(h0)
    if not improved:
        log("  no dir improved")
        step=min(1.0,step*1.4)
        if step>0.95:
            turnto((r.heading() or 0)+90+int(time.time())%180)
            step=0.4
    time.sleep(0.2)
log("home2 exit")
r.motors(0,0)
