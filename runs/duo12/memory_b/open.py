import time, math, json, sys
sys.path.insert(0,'/bot/src')
from lib import *
TICKS_PER_M=1600.0
dur=float(sys.argv[1]) if len(sys.argv)>1 else 240
def status():
    d={}
    for kv in r('d3').split():
        k,v=kv.split('='); d[k]=int(v)
    return d
x=y=0.0
visited={}
try:
    for line in open('map.log'):
        try:
            p=json.loads(line); x=p['x']; y=p['y']
            c=(round(x*5),round(y*5))
            visited[c]=visited.get(c,0)+1
        except: pass
except Exception: pass
l0=ri('d9'); r0=ri('d6')
log=open('map.log','a')
def upd():
    global l0,r0,x,y
    l=ri('d9'); rr=ri('d6')
    d=((l-l0)+(rr-r0))/2/TICKS_PER_M
    l0,r0=l,rr
    h=math.radians(heading())
    x+=d*math.cos(h); y+=d*math.sin(h)
    return d
def logit(L,st):
    log.write(json.dumps({'t':round(time.time(),1),'x':round(x,3),'y':round(y,3),
        'h':heading(),'L':L,'st':st,'d11':r('d11')})+'\n'); log.flush()
t0=time.time()
flag=False
try:
  while time.time()-t0<dur and not flag:
    stop(); time.sleep(0.2)
    L=lidar(); st=status(); upd(); logit(L,st)
    if st.get('here') or st.get('goal'): print("FLAG",st); break
    h=heading()
    Lc=[v if v>0 else 2.0 for v in L]
    best=None
    for i in range(16):
        rng=min(Lc[i], Lc[(i+1)%16]*1.4, Lc[(i-1)%16]*1.4)
        if rng<0.35: continue
        a=math.radians(h+22.5*i)
        # novelty at projected point
        px,py=x+min(rng,1.0)*math.cos(a), y+min(rng,1.0)*math.sin(a)
        nov=visited.get((round(px*5),round(py*5)),0)
        score=min(rng,1.5) - 0.35*nov
        if best is None or score>best[0]: best=(score,i,rng)
    if best is None:
        w('d1',-25); w('d7',25); time.sleep(0.8); continue
    _,i,rng=best
    target=h+22.5*i
    turn_to(target,tol=6)
    # drive forward
    dist=min(rng-0.22, 0.6)
    traveled=0
    w('d1',60); w('d7',60)
    ts=time.time()
    while traveled<dist and time.time()-ts<6:
        time.sleep(0.12)
        traveled+=abs(upd())
        if bump():
            w('d1',-35); w('d7',-35); time.sleep(0.5); break
        L=lidar(); st=status(); logit(L,st)
        if st.get('here') or st.get('goal'): print("FLAG",st); flag=True; break
        Lc=[v if v>0 else 2.0 for v in L]
        if min(Lc[0],Lc[1]*1.4,Lc[15]*1.4)<0.24: break
        c=(round(x*5),round(y*5)); visited[c]=visited.get(c,0)+1
    stop()
finally:
  stop(); print("end",round(x,2),round(y,2),status())
