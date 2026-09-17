import sys,time
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
while r.h is None or r.rays is None: r.update(); time.sleep(0.05)
# pick most open axis
AXES=[5,95,185,275]
best=None
for a in AXES:
    # ray index closest to a-h
    r.update()
    idx=round(((a-r.h)%360)/22.5)%16
    v=r.ray(idx)
    print('axis',a,'idx',idx,'clear',v)
    if v>0 and (best is None or v>best[1]): best=(a,v)
a=best[0]
print('calibrating along',a,'clear',best[1])
d.turn_to(a)
r.update()
samples=[]
t0=time.time()
r.wheels(60,60)
while time.time()-t0<3.0:
    r.update()
    f=r.ray(0)
    if f and f>0.25 and f<2.9: samples.append((time.time()-t0,f))
    if f and f<0.3: break
    time.sleep(0.05)
r.wheels(0,0)
if len(samples)>5:
    n=len(samples); sx=sum(s[0] for s in samples); sy=sum(s[1] for s in samples)
    sxx=sum(s[0]**2 for s in samples); sxy=sum(s[0]*s[1] for s in samples)
    slope=(n*sxy-sx*sy)/(n*sxx-sx*sx)
    print('samples',n,'v=%.4f m/s at cmd60 => %.6f per unit'%(-slope,-slope/60))
else:
    print('too few samples',samples)
