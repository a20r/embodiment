import time, math, json, sys
sys.path.insert(0,'/bot/src')
from lib import *
TICKS_PER_M=1600.0
dur=float(sys.argv[1]) if len(sys.argv)>1 else 400
def status():
    d={}
    for kv in r('d3').split():
        k,v=kv.split('='); d[k]=int(v)
    return d
x,y=-3.37,4.18
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
def med11(n=7,dt=0.2):
    vs=[]
    for _ in range(n):
        try: vs.append(float(r('d11')))
        except: pass
        time.sleep(dt)
    vs.sort(); return vs[len(vs)//2]
def logit(tag,d11):
    st=status()
    log.write(json.dumps({'t':round(time.time(),1),'x':round(x,3),'y':round(y,3),'h':heading(),
        'L':lidar(),'st':st,'d11':d11,'tag':tag})+'\n'); log.flush()
    if st.get('goal'): open('/bot/src/GOAL_DONE','w').write(str(st))
    return st
def step(direction_h, dist, sp=55):
    turn_to(direction_h,tol=8)
    traveled=0
    w('d1',sp); w('d7',sp); ts=time.time()
    while traveled<dist and time.time()-ts<6:
        time.sleep(0.12)
        traveled+=abs(upd())
        if bump(): w('d1',-35); w('d7',-35); time.sleep(0.5); upd(); break
        Lc=[v if v>0 else 2.0 for v in lidar()]
        if min(Lc[0],Lc[1]*1.4,Lc[15]*1.4)<0.22: break
    stop(); return traveled
t0=time.time()
cur=med11()
last_dir=None
fails=0
while time.time()-t0<dur:
    st=logit('chase',cur)
    if cur<0.09:
        print("ADJACENT",cur,x,y,flush=True); break
    h=heading()
    Lc=[v if v>0 else 2.0 for v in lidar()]
    cands=[]
    for i in range(16):
        rng=min(Lc[i], Lc[(i+1)%16]*1.4, Lc[(i-1)%16]*1.4)
        if rng<0.42: continue
        cands.append((h+22.5*i, min(rng-0.24,0.55)))
    if not cands:
        w('d1',-25); w('d7',25); time.sleep(0.8); stop(); continue
    if last_dir is not None and fails==0:
        cands.sort(key=lambda c: abs(((c[0]-last_dir)+180)%360-180))
    else:
        import random
        random.shuffle(cands)
        cands.sort(key=lambda c:-c[1])
    ang,dist=cands[0]
    tr=step(ang,dist)
    new=med11()
    print(f"({x:.2f},{y:.2f}) d11 {cur:.3f}->{new:.3f} dir{ang%360:.0f} tr{tr:.2f}",flush=True)
    if new<cur-0.005:
        last_dir=ang; fails=0
    else:
        fails+=1
        if fails>=2: last_dir=None; fails=0
    cur=new
stop(); print("end",x,y,cur,status(),flush=True)
