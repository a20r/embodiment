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
# start pose: last chase pose
x,y=-3.288,4.751
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
def d11avg(n=6,dt=0.25):
    vs=[]
    for _ in range(n):
        try: vs.append(float(r('d11')))
        except: pass
        time.sleep(dt)
    vs.sort()
    return vs[len(vs)//2]
def logit(L,st,d11):
    log.write(json.dumps({'t':round(time.time(),1),'x':round(x,3),'y':round(y,3),
        'h':heading(),'L':L,'st':st,'d11':d11})+'\n'); log.flush()
def step(direction_h, dist):
    turn_to(direction_h,tol=8)
    traveled=0
    w('d1',60); w('d7',60); ts=time.time()
    while traveled<dist and time.time()-ts<6:
        time.sleep(0.12)
        traveled+=abs(upd())
        if bump():
            w('d1',-35); w('d7',-35); time.sleep(0.5); upd(); break
        L=lidar()
        Lc=[v if v>0 else 2.0 for v in L]
        if min(Lc[0],Lc[1]*1.4,Lc[15]*1.4)<0.24: break
    stop()
    return traveled
t0=time.time()
cur=d11avg()
print("start d11",cur, flush=True)
last_dir=None
while time.time()-t0<dur:
    st=status(); L=lidar(); upd(); logit(L,st,cur)
    if cur<0.15:
        print("CLOSE",cur,x,y,flush=True); break
    h=heading()
    Lc=[v if v>0 else 2.0 for v in L]
    # candidate directions: prefer last_dir, else all clear beams
    cands=[]
    for i in range(16):
        rng=min(Lc[i], Lc[(i+1)%16]*1.4, Lc[(i-1)%16]*1.4)
        if rng<0.45: continue
        cands.append((h+22.5*i, min(rng-0.25,0.6)))
    if not cands:
        w('d1',-25); w('d7',25); time.sleep(0.8); stop(); continue
    if last_dir is not None:
        cands.sort(key=lambda c: abs(((c[0]-last_dir)+180)%360-180))
    else:
        cands.sort(key=lambda c:-c[1])
    ang,dist=cands[0]
    tr=step(ang,dist)
    new=d11avg()
    print(f"moved {tr:.2f} to ({x:.2f},{y:.2f}) d11 {cur:.3f}->{new:.3f}",flush=True)
    if new<cur-0.01:
        last_dir=ang
    elif new>cur+0.01:
        # go back / try different
        last_dir=None if last_dir is None else (last_dir+180)
    cur=new
stop(); print("end",x,y,cur,status(),flush=True)
