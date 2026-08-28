import sys, time
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
from nav2 import turn_to2
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
P=lambda *a: print(*a, flush=True)
res=turn_to2(n,r,3,tol=12,timeout=120)
P("turn north:",res,"h=",r.heading(),"d6=",r.get(6))
if res=='goal': P("GOAL"); sys.exit()
# creep north slowly watching d6
t0=time.time()
while time.time()-t0<35:
    if r.goal(): P("GOAL!!!",r.get(0)); break
    h=r.heading(); s=r.scan()
    if h is None or not s: time.sleep(0.1); continue
    d6=r.get(6)
    P(f"b0={s[0]:.2f} d6={d6} h={h:.0f} b4={s[4]:.2f} b12={s[12]:.2f}")
    if d6=='1': P("d6=1! stop"); break
    if 0<s[0]<0.15 or r.bump(): P("blocked"); break
    steer=max(-90,min(90,2.5*angdiff(3,h)))
    n.cmd(steer,8)
    time.sleep(0.25)
n.stop()
P("final",r.scan(),r.heading(),"d6=",r.get(6))
