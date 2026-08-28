import sys, time, math
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
from nav2 import turn_to2
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
P=lambda *a: print(*a, flush=True)
res=turn_to2(n,r,91,tol=12,timeout=120)
P("turn east:",res,"h=",r.heading())
if res=='goal': sys.exit()
def creep(thr, dur, tag):
    t0=time.time()
    while time.time()-t0<dur:
        if r.goal(): P("GOAL!!!",r.get(0)); n.stop(); return 'goal'
        h=r.heading(); s=r.scan()
        if h is None or not s: time.sleep(0.1); continue
        f = s[0] if thr>0 else s[8]
        if f<0.14 and f>0 or r.bump():
            P(tag,"blocked/bump f=",f,"d6=",r.get(6)); n.stop(); return 'blocked'
        steer=max(-90,min(90,2.5*angdiff(91,h)))
        n.cmd(steer if thr>0 else -steer, thr)
        P(f"{tag} b0={s[0]:.2f} b8={s[8]:.2f} d6={r.get(6)} d5={r.get(5)} h={h:.0f}")
        time.sleep(0.25)
    n.stop(); return 'time'
# creep east until b0~1.5 then west until blocked
if creep(9, 25, 'E')!='goal':
    creep(-9, 45, 'W')
P("final scan",r.scan())
