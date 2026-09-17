import time, math, json, sys
sys.path.insert(0,'/bot/src')
from lib import *
def status():
    d={}
    for kv in r('d3').split():
        k,v=kv.split('='); d[k]=int(v)
    return d
log=open('map.log','a')
TICKS_PER_M=1600.0
x,y=-3.37,4.18
l0=ri('d9'); r0=ri('d6')
def upd():
    global l0,r0,x,y
    l=ri('d9'); rr=ri('d6')
    d=((l-l0)+(rr-r0))/2/TICKS_PER_M
    l0,r0=l,rr
    h=math.radians(heading())
    x+=d*math.cos(h); y+=d*math.sin(h)
res=[]
t0=time.time()
while time.time()-t0<100:
    upd(); st=status()
    res.append((round(x,2),round(y,2),st['here']))
    log.write(json.dumps({'t':round(time.time(),1),'x':round(x,3),'y':round(y,3),'h':heading(),'L':lidar(),'st':st,'d11':r('d11'),'tag':'probeE'})+'\n'); log.flush()
    h=heading()
    Lc=[v if v>0 else 2.0 for v in lidar()]
    best=None
    for i in range(16):
        rng=min(Lc[i], Lc[(i+1)%16]*1.3, Lc[(i-1)%16]*1.3)
        if rng<0.4: continue
        ang=(h+22.5*i)%360
        dd=abs(((ang-0)+180)%360-180)  # east
        if best is None or dd<best[0]: best=(dd,ang,rng)
    if best is None:
        w('d1',-25); w('d7',25); time.sleep(0.7); stop(); continue
    _,ang,rng=best
    turn_to(ang,tol=8)
    w('d1',55); w('d7',55); ts=time.time()
    while time.time()-ts<3:
        time.sleep(0.15); upd()
        if bump(): w('d1',-35); w('d7',-35); time.sleep(0.5); break
        Lc=[v if v>0 else 2.0 for v in lidar()]
        if min(Lc[0],Lc[1]*1.3,Lc[15]*1.3)<0.24: break
    stop()
# summarize transitions
prev=None
for p in res:
    if prev is None or p[2]!=prev:
        print("HERE",p,flush=True)
        prev=p[2]
print("end",res[-1],flush=True)
