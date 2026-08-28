import sys, time, math
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
from nav2 import turn_to2
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
P=lambda *a: print(time.strftime('%H:%M:%S'),*a, flush=True)
P("hunter start")
prev=None; prevh=None
while True:
    if r.goal(): P("GOAL!!!", r.get(0)); n.stop(); break
    h=r.heading(); s=r.scan()
    if h is None or not s: time.sleep(0.2); continue
    d6=r.get(6)
    blob=None
    if prev is not None and abs(angdiff(h,prevh))<4:
        for i in range(16):
            a,b=prev[i],s[i]
            if a>0 and b>0 and abs(b-a)>0.18 and min(a,b)<1.4:
                blob=(i,(h+22.5*i)%360,min(a,b),b-a)
    if blob: P(f"blob beam{blob[0]} world={blob[1]:.0f} dist={blob[2]:.2f} dd={blob[3]:+.2f} d6={d6}")
    if d6=='1':
        P("d6=1! pursuing", "blob=",blob)
        target = blob[1] if blob else h
        # quick turn toward, then lunge
        res=turn_to2(n,r,target,tol=20,timeout=40)
        if res=='goal': P("GOAL during turn"); break
        t0=time.time()
        while time.time()-t0<12:
            if r.goal(): P("GOAL!!!"); n.stop(); break
            hh=r.heading(); ss=r.scan()
            if hh is None or not ss: time.sleep(0.1); continue
            if r.get(6)!='1': P("lost d6"); break
            f=ss[0]
            if 0<f<0.12 or r.bump():
                n.cmd(0,-8); time.sleep(0.5); n.stop()
            else:
                n.cmd(max(-90,min(90,3*angdiff(target,hh))), 16)
            time.sleep(0.15)
        n.stop()
        if r.goal(): P("GOAL!!!", r.get(0)); break
        prev=None; prevh=None; continue
    prev=s; prevh=h
    time.sleep(0.4)
