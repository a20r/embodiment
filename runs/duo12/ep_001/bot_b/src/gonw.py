import time, math, json, sys
sys.path.insert(0,'/bot/src')
from lib import *
TICKS_PER_M=1600.0
def status():
    d={}
    for kv in r('d3').split():
        k,v=kv.split('='); d[k]=int(v)
    return d
import subprocess
p=json.loads(subprocess.check_output(['tail','-1','/bot/src/map.log']).decode())
x,y=p['x'],p['y']
l0=ri('d9'); r0=ri('d6')
log=open('map.log','a')
def upd():
    global l0,r0,x,y
    l=ri('d9'); rr=ri('d6')
    d=((l-l0)+(rr-r0))/2/TICKS_PER_M
    l0,r0=l,rr
    h=math.radians(heading())
    x+=d*math.cos(h); y+=d*math.sin(h)
def med11(n=5,dt=0.2):
    vs=[]
    for _ in range(n):
        try: vs.append(float(r('d11')))
        except: pass
        time.sleep(dt)
    vs.sort(); return vs[len(vs)//2]
def logit(tag,d11=None):
    st=status()
    log.write(json.dumps({'t':round(time.time(),1),'x':round(x,3),'y':round(y,3),'h':heading(),
        'L':lidar(),'st':st,'d11':d11 if d11 else r('d11'),'tag':tag})+'\n'); log.flush()
    return st
# waypoints toward NW passage then beyond
wps=[(-2.2,3.9),(-2.9,3.95),(-3.3,4.2),(-3.45,4.4)]
for gx,gy in wps:
    for it in range(25):
        upd(); st=logit('gonw')
        d=math.hypot(gx-x,gy-y)
        if d<0.2: break
        b=math.degrees(math.atan2(gy-y,gx-x))
        h=heading()
        Lc=[v if v>0 else 2.0 for v in lidar()]
        # pick clear beam closest to bearing
        best=None
        for i in range(16):
            rng=min(Lc[i], Lc[(i+1)%16]*1.3, Lc[(i-1)%16]*1.3)
            if rng<0.4: continue
            ang=h+22.5*i
            dd=abs(((ang-b)+180)%360-180)
            if best is None or dd<best[0]: best=(dd,ang,rng)
        if best is None:
            w('d1',-25); w('d7',25); time.sleep(0.7); stop(); continue
        _,ang,rng=best
        turn_to(ang,tol=8)
        w('d1',55); w('d7',55); ts=time.time(); tr=0
        lA=ri('d9')
        while time.time()-ts<4:
            time.sleep(0.12); upd()
            if bump(): w('d1',-35); w('d7',-35); time.sleep(0.5); upd(); break
            Lc=[v if v>0 else 2.0 for v in lidar()]
            if min(Lc[0],Lc[1]*1.3,Lc[15]*1.3)<0.24: break
            if math.hypot(gx-x,gy-y)<0.2: break
        stop()
    m=med11()
    print(f"wp({gx},{gy}) reached~({x:.2f},{y:.2f}) d11={m:.3f}",flush=True)
print("end",x,y,med11(),status(),flush=True)
