import time, math, threading, json
from collections import deque
def rline(p):
    for _ in range(30):
        try:
            with open(f'/dev/robot/d{p}') as f:
                s=f.readline().strip()
            if s: return s
        except: pass
        time.sleep(0.01)
    return ''
def w(p,v):
    with open(f'/dev/robot/d{p}','w') as f: f.write(str(v)+'\n')
def motors(a,b): w(10,a); w(11,b)
def lidar():
    for _ in range(30):
        s=rline(3)
        try:
            l=[float(x) for x in s.split(',')]
            if len(l)==16: return l
        except: pass
    return None
def heading():
    try: return float(rline(1))
    except: return None
LOG=open('/memory/run7.log','a',buffering=1)
def log(*a): LOG.write(f"{time.time():.1f} "+" ".join(str(x) for x in a)+"\n")
def rx_loop():
    while True:
        try:
            with open('/dev/robot/d4') as f:
                for line in f:
                    line=line.strip()
                    if line: log("RX",line)
        except: time.sleep(0.5)
threading.Thread(target=rx_loop,daemon=True).start()
RES=0.05; OCC_TTL=120
occ={}; free=set()
pose=[0.0,0.0]; enc=[None,None]
def cellof(x,y): return (int(math.floor(x/RES)),int(math.floor(y/RES)))
def upd_pose(h,skip):
    try: L=float(rline(7)); R=float(rline(8))
    except: return
    if enc[0] is not None and not skip and h is not None:
        ds=((L-enc[0])+(R-enc[1]))/2000.0
        if abs(ds)<0.2:
            th=math.radians(-h)
            pose[0]+=ds*math.cos(th); pose[1]+=ds*math.sin(th)
    enc[0],enc[1]=L,R
def update_map(h,l,t):
    x,y=pose
    for k in range(16):
        r=l[k]
        if r<=0: continue
        th=math.radians(-(h+22.5*k))
        c=math.cos(th); s=math.sin(th)
        n=int(min(r,2.5)/RES)
        for i in range(1,n):
            d=i*RES
            fc=cellof(x+c*d,y+s*d)
            free.add(fc); occ.pop(fc,None)
        if r<2.4:
            oc=cellof(x+c*r,y+s*r)
            occ[oc]=t; free.discard(oc)
def neighbors(c):
    x,y=c
    return ((x+1,y),(x-1,y),(x,y+1),(x,y-1))
INFL=2
def plan(t):
    for c in [c for c,ts in occ.items() if t-ts>OCC_TTL]: occ.pop(c)
    b=set()
    for (x,y) in occ:
        for dx in range(-INFL,INFL+1):
            for dy in range(-INFL,INFL+1):
                b.add((x+dx,y+dy))
    start_c=cellof(*pose)
    if start_c in b or start_c not in free:
        q=deque([start_c]); seen={start_c}; found=None
        while q:
            c=q.popleft()
            if c in free and c not in b: found=c; break
            if abs(c[0]-start_c[0])>10 or abs(c[1]-start_c[1])>10: continue
            for n2 in neighbors(c):
                if n2 not in seen: seen.add(n2); q.append(n2)
        if found is None: return None
        start_c=found
    prev={start_c:None}; q=deque([start_c]); target=None
    while q:
        c=q.popleft()
        unk=0
        for n2 in neighbors(c):
            if n2 not in free and n2 not in occ: unk+=1
        if unk>=1:
            dx=c[0]*RES-pose[0]; dy=c[1]*RES-pose[1]
            if dx*dx+dy*dy>0.2: target=c; break
        for n2 in neighbors(c):
            if n2 in prev or n2 not in free or n2 in b: continue
            prev[n2]=c; q.append(n2)
    if not target: return None
    path=[]; c=target
    while c is not None: path.append(c); c=prev[c]
    path.reverse(); return path
start=time.time(); last_tx=0; last_stat=0; last_plan=0; last_save=time.time()
path=None; hist=[]; nofront=0
log("START frontier2")
def escape():
    log("ESCAPE",round(pose[0],2),round(pose[1],2))
    t0=time.time()
    while time.time()-t0<1.3:
        motors(-55,-55); time.sleep(0.05)
    motors(0,0)
try:
  while time.time()-start<4800:
    t=time.time()
    h=heading(); l=lidar()
    if l is None or h is None: continue
    hist.append((t,list(l)))
    while hist and hist[0][0]<t-1.6: hist.pop(0)
    if len(hist)>8 and t-hist[0][0]>1.2:
        old=hist[0][1]
        diff=sum(abs(a-b) for a,b in zip(old,l) if 0<a<2.9 and 0<b<2.9)/16
        if diff<0.02:
            upd_pose(h,True)
            escape(); hist.clear(); path=None; last_plan=0
            continue
    upd_pose(h,False)
    update_map(h,l,t)
    if t-last_plan>4 or not path:
        last_plan=t
        path=plan(t)
        if path is None:
            nofront+=1; log("NOFRONTIER",nofront)
            if nofront>=6:
                occ.clear(); nofront=0; log("OCC-RESET")
    if path:
        pc=cellof(*pose)
        best=min(range(len(path)),key=lambda i:(path[i][0]-pc[0])**2+(path[i][1]-pc[1])**2)
        wi=min(best+7,len(path)-1)
        wx=(path[wi][0]+0.5)*RES; wy=(path[wi][1]+0.5)*RES
        dx=wx-pose[0]; dy=wy-pose[1]; dist=math.hypot(dx,dy)
        if dist<0.1 and wi==len(path)-1:
            path=None; motors(0,0)
        else:
            tgt_h=(-math.degrees(math.atan2(dy,dx)))%360
            err=(tgt_h-h+180)%360-180
            if abs(err)>50:
                v=max(min(err*0.8,30),-30); v=math.copysign(max(abs(v),6),v)
                motors(v,-v)
            else:
                fr=min(l[0],l[1],l[15])
                base=60 if fr>0.6 else 42
                stv=max(min(err*1.5,26),-26)
                motors(base+stv,base-stv)
    else:
        motors(15,-15)
    if t-last_tx>2:
        last_tx=t
        w(0,json.dumps({"id":"A","x":round(pose[0],2),"y":round(pose[1],2)}))
    if t-last_stat>3:
        last_stat=t
        st6=rline(6); d5=rline(5)
        log("POS",round(pose[0],2),round(pose[1],2),"h",round(h,1),"d5",d5,"st",st6,"pl",len(path) if path else 0,"occ",len(occ),"free",len(free))
        if "here=1" in st6 or "goal=1" in st6: log("HIT",st6)
    if t-last_save>60:
        last_save=t
        json.dump({"occ":list(occ),"free":list(free),"pose":pose},open('/memory/map.json','w'))
    time.sleep(0.03)
finally:
    motors(0,0); log("END",pose)
    json.dump({"occ":list(occ),"free":list(free),"pose":pose},open('/memory/map.json','w'))
