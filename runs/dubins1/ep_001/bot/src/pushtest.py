import sys, time
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
from nav2 import turn_to2
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
P=lambda *a: print(*a, flush=True)
res=turn_to2(n,r,95,tol=14,timeout=90)
P("turn:",res,"h=",r.heading(),"scan",r.scan())
if res=='goal': P("GOAL"); sys.exit()
# approach & push
t0=time.time()
while time.time()-t0<25:
    if r.goal(): P("GOAL!!!",r.get(0)); n.stop(); sys.exit()
    h=r.heading(); s=r.scan()
    if h is None or not s: time.sleep(0.1); continue
    P(f"b0={s[0]:.2f} d5={r.get(5)} d6={r.get(6)} h={h:.0f}")
    steer=max(-90,min(90,2.5*angdiff(95,h)))
    n.cmd(steer, 18)
    time.sleep(0.3)
n.stop()
P("final scan",r.scan(),"d0=",r.get(0),"d6=",r.get(6))
