import sys, time
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
P=lambda *a: print(*a, flush=True)
t0=time.time()
while time.time()-t0<30:
    if r.goal(): P("GOAL"); n.stop(); sys.exit()
    h=r.heading(); s=r.scan()
    if h is None or not s: time.sleep(0.1); continue
    P(f"b0={s[0]:.2f} b8={s[8]:.2f} b4={s[4]:.2f} b12={s[12]:.2f} d6={r.get(6)} h={h:.0f}")
    if s[8]>0 and s[8]<0.75: P("at junction-ish"); break
    steer=max(-90,min(90,2.5*angdiff(3,h)))
    n.cmd(-steer,-11)
    time.sleep(0.25)
n.stop()
P("final",r.scan(),r.heading())
