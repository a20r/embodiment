import sys, time, math, json
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
visited=[]  # (x,y)
def visit_penalty(px,py):
    p=0
    for (vx,vy) in visited[-400:]:
        d2=(px-vx)**2+(py-vy)**2
        if d2<0.09: p+=1
    return p
log=open('/bot/src/explore.out','a')
def P(*a):
    s=' '.join(str(x) for x in a)
    log.write(f"{time.strftime('%H:%M:%S')} {s}\n"); log.flush()
P("=== explorer start ===")
last_h=None
while True:
    if r.goal():
        P("GOAL REACHED!!!", r.get(0)); n.stop(); break
    h=n.upd()
    s=r.scan()
    if h is None or not s:
        time.sleep(0.5); continue
    visited.append((n.x,n.y))
    # candidate directions: beams with clearance, scored
    best=None
    for i in range(16):
        d=s[i]
        if d<0: continue
        wa=(h+22.5*i)%360
        reach=min(0.9, max(0.0,d-0.25))
        if d<0.35: continue
        px=n.x+reach*math.cos(math.radians(wa))
        py=n.y+reach*math.sin(math.radians(wa))
        pen=visit_penalty(px,py)
        turncost=abs(angdiff(wa,h))/180*1.5
        score=d + 0.8*reach - 0.25*pen - turncost + 0.3*(1 if d>1.5 else 0)
        if best is None or score>best[0]: best=(score,wa,d,i,pen)
    if best is None:
        P("no candidates, backing"); n.cmd(0,-10); time.sleep(2); n.stop(); continue
    score,wa,d,i,pen=best
    P(f"pos=({n.x:.2f},{n.y:.2f}) h={h:.0f} pick beam{i} wa={wa:.0f} d={d:.2f} pen={pen} score={score:.2f}")
    res=n.turn_to(wa, tol=18, timeout=60)
    if res=='goal': P("GOAL during turn"); break
    res=n.drive(target_hdg=wa, dur=8, thr=18)
    P("  drive:",res,f"pos=({n.x:.2f},{n.y:.2f})", "d0=",r.get(0))
    if res=='goal': P("GOAL during drive"); break
