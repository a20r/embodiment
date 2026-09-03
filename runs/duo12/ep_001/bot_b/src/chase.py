import time, math, json, sys
sys.path.insert(0,'/bot/src')
from lib import *
TICKS_PER_M=1600.0
dur=float(sys.argv[1]) if len(sys.argv)>1 else 300
def status():
    d={}
    for kv in r('d3').split():
        k,v=kv.split('='); d[k]=int(v)
    return d
x,y=-3.131,4.141
l0=ri('d9'); r0=ri('d6')
log=open('map.log','a')
hist=[]
def upd():
    global l0,r0,x,y
    l=ri('d9'); rr=ri('d6')
    d=((l-l0)+(rr-r0))/2/TICKS_PER_M
    l0,r0=l,rr
    h=math.radians(heading())
    x+=d*math.cos(h); y+=d*math.sin(h)
    return d
def logit(L,st,d11):
    log.write(json.dumps({'t':round(time.time(),1),'x':round(x,3),'y':round(y,3),
        'h':heading(),'L':L,'st':st,'d11':d11})+'\n'); log.flush()
def fit_target():
    if len(hist)<15: return None
    data=hist[-120:]
    best=None
    for gxi in range(-16,17):
        for gyi in range(-16,17):
            gx,gy=x+gxi*0.5,y+gyi*0.5
            ds=[math.hypot(a-gx,b-gy) for a,b,_ in data]
            vs=[v for _,_,v in data]
            k=sum(d*v for d,v in zip(ds,vs))/max(1e-9,sum(v*v for v in vs))
            if not (2.0<k<9.0): continue
            e=sum((d-k*v)**2 for d,v in zip(ds,vs))/len(data)
            if best is None or e<best[0]: best=(e,gx,gy,k)
    return best
t0=time.time()
target=None
try:
  while time.time()-t0<dur:
    stop(); time.sleep(0.15)
    L=lidar(); st=status(); upd()
    d11=float(r('d11')); hist.append((x,y,d11)); logit(L,st,d11)
    if d11<0.10:
        print("CLOSE!",d11,x,y); break
    if len(hist)%8==0 or target is None:
        target=fit_target()
        if target: print("fit",[round(v,2) for v in target], "pos",round(x,2),round(y,2),"d11",d11)
    h=heading()
    Lc=[v if v>0 else 2.0 for v in L]
    best=None
    for i in range(16):
        rng=min(Lc[i], Lc[(i+1)%16]*1.4, Lc[(i-1)%16]*1.4)
        if rng<0.35: continue
        a=math.radians(h+22.5*i)
        step=min(rng-0.22,0.5)
        px,py=x+step*math.cos(a), y+step*math.sin(a)
        if target:
            _,gx,gy,k=target
            score=-math.hypot(px-gx,py-gy)
        else:
            score=rng
        if best is None or score>best[0]: best=(score,i,rng)
    if best is None:
        w('d1',-25); w('d7',25); time.sleep(0.8); continue
    _,i,rng=best
    turn_to(h+22.5*i,tol=6)
    dist=min(rng-0.22,0.5); traveled=0
    w('d1',60); w('d7',60); ts=time.time()
    while traveled<dist and time.time()-ts<5:
        time.sleep(0.12)
        traveled+=abs(upd())
        if bump():
            w('d1',-35); w('d7',-35); time.sleep(0.5); break
        L=lidar()
        Lc=[v if v>0 else 2.0 for v in L]
        if min(Lc[0],Lc[1]*1.4,Lc[15]*1.4)<0.24: break
    stop()
finally:
  stop(); print("end",round(x,2),round(y,2),status(),r('d11'))
