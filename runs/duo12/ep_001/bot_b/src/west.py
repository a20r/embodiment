import time, math, json, sys
sys.path.insert(0,'/bot/src')
from lib import *
def status():
    d={}
    for kv in r('d3').split():
        k,v=kv.split('='); d[k]=int(v)
    return d
log=open('map.log','a')
def logit(tag):
    st=status()
    log.write(json.dumps({'t':round(time.time(),1),'x':0,'y':0,'h':heading(),'L':lidar(),'st':st,'d11':r('d11'),'tag':tag})+'\n'); log.flush()
    return st
t0=time.time()
while time.time()-t0<160:
    st=logit('west')
    if st['here']==1:
        stop(); print("HERE=1",flush=True); break
    h=heading()
    Lc=[v if v>0 else 2.0 for v in lidar()]
    # prefer beam closest to 180 that is clear; if none, any clear beam, tie-break closest to 180
    best=None
    for i in range(16):
        rng=min(Lc[i], Lc[(i+1)%16]*1.3, Lc[(i-1)%16]*1.3)
        if rng<0.4: continue
        ang=(h+22.5*i)%360
        dd=abs(((ang-180)+180)%360-180)
        if best is None or dd<best[0]: best=(dd,ang,rng)
    if best is None:
        w('d1',-25); w('d7',25); time.sleep(0.7); stop(); continue
    _,ang,rng=best
    turn_to(ang,tol=8)
    w('d1',55); w('d7',55); ts=time.time()
    while time.time()-ts<4:
        time.sleep(0.12)
        if bump(): w('d1',-35); w('d7',-35); time.sleep(0.5); break
        Lc=[v if v>0 else 2.0 for v in lidar()]
        if min(Lc[0],Lc[1]*1.3,Lc[15]*1.3)<0.24: break
        st=status()
        if st['here']==1: break
    stop()
    if status()['here']==1:
        print("HERE=1 stop",flush=True); break
stop(); print("end",status(),flush=True)
