import time, math
from robot import Robot
r=Robot(); time.sleep(0.5)
# collect scans while rotating slowly
pts={}
r.motors(12,-12)
t0=time.time()
while time.time()-t0<10:
    h=r.hdg; l=r.lidar
    if h is None or l is None: continue
    for i,v in enumerate(l):
        if v<0: continue
        a=int(round((h+22.5*i)%360/5)*5)%360
        pts.setdefault(a,[]).append(v)
    time.sleep(0.05)
r.motors(0,0)
for a in sorted(pts):
    vs=sorted(pts[a]); m=vs[len(vs)//2]
    bar="#"*int(m*20)
    print(f"{a:3d} {m:5.2f} {bar}")
