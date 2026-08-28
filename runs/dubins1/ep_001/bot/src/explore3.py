import sys, time, math
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
CELL=0.35
visits={}; blocked={}
def cell(x,y): return (round(x/CELL), round(y/CELL))
def mark():
    c=cell(n.x,n.y); visits[c]=visits.get(c,0)+1
def ray_pen(wa,dist):
    p=0.0
    for f in (0.4,0.8,1.2):
        if f>dist-0.15: break
        px=n.x+f*math.cos(math.radians(wa)); py=n.y+f*math.sin(math.radians(wa))
        p+=visits.get(cell(px,py),0)
    return p
log=open('/bot/src/explore3.out','a')
def P(*a):
    log.write(f"{time.strftime('%H:%M:%S')} {' '.join(str(x) for x in a)}\n"); log.flush()
P("=== explorer3 start ===")

def scanvec():
    s=r.scan()
    return s

def drive(target,dur,thr=18):
    t0=time.time(); lastscan=scanvec(); lastt=t0; moved_ok=True
    while time.time()-t0<dur:
        if r.goal(): n.stop(); return 'goal'
        h=n.upd(); mark()
        front=n.clearance(0)
        if r.bump():
            n.cmd(0,-12); time.sleep(1.0); n.stop(); return 'stall'
        if front<0.16: n.stop(); return 'blocked'
        steer=0
        if h is not None: steer=max(-90,min(90,3*angdiff(target,h)))
        n.cmd(steer, thr if front>0.4 else 9)
        time.sleep(0.15)
        if time.time()-lastt>1.6:
            s=scanvec()
            if s and lastscan:
                diffs=[abs(a-b) for a,b in zip(s,lastscan) if a>0 and b>0]
                diffs.sort()
                med=diffs[len(diffs)//2] if diffs else 0
                if med<0.012:
                    n.cmd(0,-14); time.sleep(1.4); n.stop(); return 'stall'
            lastscan=s; lastt=time.time()
    n.stop(); return 'time'

stall_dirs=[]  # (x,y,wa,expiry)
while True:
    if r.goal(): P("GOAL!", r.get(0)); n.stop(); break
    h=n.upd(); s=r.scan()
    if h is None or not s: time.sleep(0.4); continue
    mark()
    now=time.time()
    stall_dirs=[e for e in stall_dirs if e[3]>now]
    best=None
    for i in range(16):
        d=s[i]
        if d<0.4: continue
        wa=(h+22.5*i)%360
        pen=ray_pen(wa,d)
        tc=abs(angdiff(wa,h))/180.0
        sp=0
        for (bx,by,bwa,_) in stall_dirs:
            if (n.x-bx)**2+(n.y-by)**2<0.36 and abs(angdiff(wa,bwa))<35: sp+=1
        score=min(d,2.2)-1.0*pen-1.8*tc-4.0*sp
        if best is None or score>best[0]: best=(score,wa,d,i,pen,sp)
    if best is None:
        P("no cand: reversing"); n.cmd(0,-12); time.sleep(2.5); n.stop(); continue
    score,wa,d,i,pen,sp=best
    P(f"pos=({n.x:.2f},{n.y:.2f}) h={h:.0f} beam{i} wa={wa:.0f} d={d:.2f} pen={pen:.1f} sp={sp} sc={score:.2f} cells={len(visits)}")
    if i!=0:
        res=n.turn_to(wa, tol=18, timeout=75)
        if res=='goal': P("GOAL during turn"); break
    res=drive(wa, dur=10)
    P("  drive:",res,f"pos=({n.x:.2f},{n.y:.2f})")
    if res=='goal': P("GOAL during drive"); break
    if res=='stall':
        stall_dirs.append((n.x,n.y,wa,time.time()+240))
        P(f"  marked stall dir wa={wa:.0f} at ({n.x:.2f},{n.y:.2f})")
