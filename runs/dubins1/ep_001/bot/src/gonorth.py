import sys, time, math
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
from nav2 import turn_to2
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
P=lambda *a: print(*a, flush=True)
# phase 1: reverse until beam12 opens (>1.5) & d6=1, max 15s
t0=time.time(); ok=False
while time.time()-t0<15:
    if r.goal(): P("GOAL"); n.stop(); sys.exit()
    s=r.scan(); h=r.heading()
    if s: P(f"rev b12={s[12]:.2f} b0={s[0]:.2f} d6={r.get(6)} h={h}")
    if s and s[12]>1.5:
        ok=True; break
    steer = max(-90,min(90,3*angdiff(91,h))) if h is not None else 0
    n.cmd(-steer,-10)   # reverse along corridor, keep heading ~91
    time.sleep(0.3)
n.stop()
P("phase1 done ok=",ok,"d6=",r.get(6))
# extra half-step back to center junction
n.cmd(0,-8); time.sleep(1.2); n.stop()
# phase 2: turn to heading 3
res=turn_to2(n,r,3,tol=14,timeout=150)
P("turn:",res,"h=",r.heading(),"scan=",r.scan(),"d6=",r.get(6))
if res=='goal': sys.exit()
# phase 3: drive north, centering with side beams 4 & 12
t0=time.time()
while time.time()-t0<40:
    if r.goal(): P("GOAL!!!", r.get(0)); break
    h=r.heading(); s=r.scan()
    if h is None or not s: time.sleep(0.2); continue
    f=min(x for x in [s[0], s[1]+0.1, s[15]+0.1] if x>0)
    if r.bump() or f<0.15:
        P("blocked f=",f); n.cmd(0,-10); time.sleep(0.8); n.stop(); break
    steer=max(-90,min(90,2.5*angdiff(3,h)))
    # centering: left=beam4, right=beam12
    l,rr=s[4],s[12]
    if l>0 and rr>0 and l<0.6 and rr<0.6:
        steer+=max(-30,min(30,120*(l-rr)))
    n.cmd(steer, 13 if f>0.4 else 8)
    P(f"n h={h:.0f} f={f:.2f} l={l:.2f} r={rr:.2f} d6={r.get(6)}")
    time.sleep(0.3)
n.stop()
P("end d0=",r.get(0),"scan=",r.scan())
