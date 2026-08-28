import sys, time, math
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
CELL=0.35
visits={}
def cell(x,y): return (round(x/CELL), round(y/CELL))
def mark():
    c=cell(n.x,n.y); visits[c]=visits.get(c,0)+1
def ray_pen(h,wa,dist):
    p=0.0
    for f in (0.4,0.8,1.2):
        if f>dist-0.15: break
        px=n.x+f*math.cos(math.radians(wa)); py=n.y+f*math.sin(math.radians(wa))
        p+=visits.get(cell(px,py),0)
    return p
log=open('/bot/src/explore2.out','a')
def P(*a):
    log.write(f"{time.strftime('%H:%M:%S')} {' '.join(str(x) for x in a)}\n"); log.flush()
P("=== explorer2 start ===")

# patched drive that marks trail
def drive(target,dur,thr=18):
    t0=time.time()
    while time.time()-t0<dur:
        if r.goal(): n.stop(); return 'goal'
        h=n.upd(); mark()
        front=n.clearance(0)
        if r.bump():
            n.cmd(0,-12); time.sleep(1.0); n.stop(); return 'bump'
        if front<0.16: n.stop(); return 'blocked'
        steer=0
        if h is not None: steer=max(-90,min(90,3*angdiff(target,h)))
        n.cmd(steer, thr if front>0.4 else 9)
        time.sleep(0.15)
    n.stop(); return 'time'

stall=0; last_cells=len(visits)
while True:
    if r.goal(): P("GOAL!", r.get(0)); n.stop(); break
    h=n.upd(); s=r.scan()
    if h is None or not s: time.sleep(0.4); continue
    mark()
    best=None
    for i in range(16):
        d=s[i]
        if d<0.4: continue
        wa=(h+22.5*i)%360
        pen=ray_pen(h,wa,d)
        tc=abs(angdiff(wa,h))/180.0
        score=min(d,2.2)-1.0*pen-1.8*tc
        if best is None or score>best[0]: best=(score,wa,d,i,pen)
    if best is None:
        P("stuck: reversing"); n.cmd(0,-12); time.sleep(2.5); n.stop(); continue
    score,wa,d,i,pen=best
    P(f"pos=({n.x:.2f},{n.y:.2f}) h={h:.0f} beam{i} wa={wa:.0f} d={d:.2f} pen={pen:.1f} sc={score:.2f} cells={len(visits)}")
    if i!=0:
        res=n.turn_to(wa, tol=18, timeout=75)
        if res=='goal': P("GOAL during turn"); break
    res=drive(wa, dur=10)
    P("  drive:",res,f"pos=({n.x:.2f},{n.y:.2f})")
    if res=='goal': P("GOAL during drive"); break
    # stagnation check
    if len(visits)==last_cells: stall+=1
    else: stall=0; last_cells=len(visits)
    if stall>=6:
        P("stagnant: random reversal")
        n.cmd(0,-14); time.sleep(3); n.stop(); stall=0
