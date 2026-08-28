import sys, time, math
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
from nav2 import turn_to2
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
P=lambda *a: print(*a, flush=True)
# reverse south along corridor (facing ~3), until east side (b4, world~93) opens >1.5
t0=time.time()
while time.time()-t0<40:
    if r.goal(): P("GOAL"); n.stop(); sys.exit()
    h=r.heading(); s=r.scan()
    if h is None or not s: time.sleep(0.2); continue
    back=min(x for x in (s[8],s[7]+0.1,s[9]+0.1) if x>0)
    e=s[4]
    P(f"h={h:.0f} back={back:.2f} east(b4)={e:.2f} d6={r.get(6)}")
    if e>1.5 and s[4]>0: P("junction reached"); break
    if back<0.14 or r.bump():
        P("rear blocked"); break
    steer=max(-90,min(90,2.5*angdiff(3,h)))
    n.cmd(-steer,-12)
    time.sleep(0.3)
n.stop()
P("scan:",r.scan(),"h=",r.heading(),"d6=",r.get(6))
