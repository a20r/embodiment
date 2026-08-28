import sys, time
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
P=lambda *a: print(*a, flush=True)
t0=time.time()
while time.time()-t0<60:
    if r.goal(): P("GOAL"); n.stop(); sys.exit()
    h=r.heading(); s=r.scan()
    if h is None or not s: time.sleep(0.1); continue
    d6=r.get(6)
    P(f"b0={s[0]:.2f} b8={s[8]:.2f} d6={d6} h={h:.0f}")
    if d6=='1':
        P("d6=1 !! stopping"); n.stop(); break
    if s[0]>2.25:
        P("reached b0>2.25"); n.stop(); break
    if 0<s[8]<0.15:
        P("rear blocked"); n.stop(); break
    steer=max(-90,min(90,2.5*angdiff(91,h)))
    n.cmd(-steer,-9)
    time.sleep(0.25)
n.stop()
P("final",r.scan(),r.heading(),r.get(6))
time.sleep(2)
P("settle d6=",r.get(6),"d0=",r.get(0))
