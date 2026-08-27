import sys, time, math, json, random, collections
sys.path.insert(0,'/bot/src')
from lib import *
LOG=open('/bot/nav.log','a',buffering=1)
def log(*a): print(time.strftime('%H:%M:%S'),*a,file=LOG)
def angdiff(a,b):
    d=(a-b)%360
    return d-360 if d>180 else d
prev=[0.5]*16
def lid():
    global prev
    v=lidar()
    if v is None: return prev
    v=[prev[i] if v[i]<0 else v[i] for i in range(16)]
    prev=v; return v
def enc():
    l=rdf(6); r=rdf(2)
    return (l,r) if l is not None and r is not None else None

SCALE=760.0   # counts per lidar unit
CELL=120.0    # counts per grid cell
MAXR=1.8      # trust lidar out to this many units
free=set(); wall=collections.Counter(); freec=collections.Counter()
samples=[]    # (x,y,s)
x,y=0.0,0.0
last=enc()
def cell(px,py): return (int(math.floor(px/CELL)), int(math.floor(py/CELL)))
def update_map(v,h):
    for i in range(16):
        d=v[i]
        if d<0: continue
        b=math.radians((h+i*22.5)%360)
        dd=min(d,MAXR)
        # free along ray
        steps=int(dd*SCALE/ (CELL*0.6))
        for t in range(1,steps+1):
            r=t*CELL*0.6
            if r>dd*SCALE: break
            c=cell(x+r*math.cos(b), y+r*math.sin(b))
            freec[c]+=1
        if d<MAXR:
            c=cell(x+d*SCALE*math.cos(b), y+d*SCALE*math.sin(b))
            wall[c]+=1
def is_free(c): return freec[c]>=2 and freec[c]>3*wall[c]
def is_wall(c): return wall[c]>=2 and not (freec[c]>3*wall[c])

def fit_goal():
    if len(samples)<80: return None
    data=samples[::max(1,len(samples)//400)]
    best=None
    xs=[p[0] for p in data]; ys=[p[1] for p in data]
    cx,cy=sum(xs)/len(xs), sum(ys)/len(ys)
    for gx in range(int(cx)-3000,int(cx)+3001,200):
        for gy in range(int(cy)-3000,int(cy)+3001,200):
            n=0;sx=sy=sxx=sxy=syy=0
            bad=False
            for (px,py,s) in data:
                r=math.hypot(gx-px,gy-py)
                if r<40: bad=True;break
                lx=math.log(r); ly=math.log(s)
                n+=1;sx+=lx;sy+=ly;sxx+=lx*lx;sxy+=lx*ly;syy+=ly*ly
            if bad: continue
            cov=sxy-sx*sy/n; vx=sxx-sx*sx/n; vy=syy-sy*sy/n
            if vx<=0 or vy<=0: continue
            bcoef=cov/vx; r2=cov*cov/(vx*vy)
            if bcoef<0 and (best is None or r2>best[0]):
                best=(r2,gx,gy,bcoef)
    return best

def plan(goalc):
    # BFS over free cells from current cell to goalc or nearest reachable
    start=cell(x,y)
    if not is_free(start): freec[start]+=5
    q=collections.deque([start]); came={start:None}
    target=None; bestd=1e18
    while q:
        c=q.popleft()
        d2=(c[0]-goalc[0])**2+(c[1]-goalc[1])**2
        if d2<bestd: bestd=d2; target=c
        if c==goalc: break
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                if dx==0 and dy==0: continue
                n=(c[0]+dx,c[1]+dy)
                if n in came: continue
                if is_free(n) and not is_wall(n):
                    came[n]=c; q.append(n)
    # frontier bonus: if goalc unreachable, target = reachable cell closest to goalc
    path=[]
    c=target
    while c is not None:
        path.append(c); c=came[c]
    path.reverse()
    return path

def drive_to(wx,wy,tmax=25):
    global x,y,last
    t0=time.time()
    while time.time()-t0<tmax:
        if goal():
            drive(0,0); log('GOAL!'); open('/memory/GOAL.txt','a').write('GOAL by nav\n'); sys.exit(0)
        v=lid(); h=hdg()
        e=enc()
        if e and last:
            d=((e[0]-last[0])+(e[1]-last[1]))/2.0
            x+=d*math.cos(math.radians(h)); y+=d*math.sin(math.radians(h))
        last=e
        update_map(v,h)
        s=rdf(4)
        if s: samples.append((x,y,s))
        if math.hypot(wx-x,wy-y)<CELL*0.9: return True
        bias=math.degrees(math.atan2(wy-y,wx-x))%360
        dh=angdiff(bias,h)
        front=min(v[15],v[0],v[1])
        if abs(dh)>55:
            drive(-4,4) if dh>0 else drive(4,-4)
            time.sleep(0.2); continue
        if front<0.20:
            drive(-5,-5); time.sleep(0.5); drive(0,0)
            return False
        steer=max(-2,min(2,dh/25))
        l=min(v[1],v[2]); r=min(v[14],v[15])
        if l<0.13: steer=min(steer,-1)
        if r<0.13: steer=max(steer,1)
        drive(6-steer,6+steer)
        time.sleep(0.2)
    return False

log('nav start')
gest=None; lastfit=0; ebias=random.uniform(0,360)
it=0
while True:
    it+=1
    v=lid(); h=hdg(); update_map(v,h)
    s=rdf(4)
    if s: samples.append((x,y,s))
    if time.time()-lastfit>40 and len(samples)>150:
        f=fit_goal()
        if f:
            log(f'fit r2={f[0]:.3f} at ({f[1]},{f[2]}) p={-f[3]:.2f} pose=({x:.0f},{y:.0f}) s={s} n={len(samples)}')
        else:
            log(f'fit failed n={len(samples)} pose=({x:.0f},{y:.0f}) s={s}')
        if f and f[0]>0.6:
            gest=(f[1],f[2])
        lastfit=time.time()
    if gest is None:
        # explore: run-and-tumble hill climb on s
        def med(n=8):
            a=[]
            for _ in range(n):
                vv=rdf(4)
                if vv is not None: a.append(vv)
                time.sleep(0.05)
            a.sort(); return a[len(a)//2] if a else 0
        s0=med()
        cands=sorted(range(16), key=lambda k: abs(angdiff((h+k*22.5)%360,ebias)))
        kk=None
        for k in cands:
            if min(v[(k-1)%16],v[k],v[(k+1)%16])>0.32: kk=k;break
        if kk is None:
            kk=max(range(16),key=lambda k:min(v[(k-1)%16],v[k],v[(k+1)%16]))
        b=(h+kk*22.5)%360
        sx0,sy0=x,y
        drive_to(x+350*math.cos(math.radians(b)), y+350*math.sin(math.radians(b)),tmax=16)
        s1=med()
        moved=math.hypot(x-sx0,y-sy0)
        db=math.degrees(math.atan2(y-sy0,x-sx0))%360 if moved>60 else ebias
        if s1>s0+0.004 and moved>60: ebias=db
        elif s1<s0-0.004 or moved<=60: ebias=(db+180+random.uniform(-70,70))%360
        log(f'explore s {s0:.3f}->{s1:.3f} moved {moved:.0f} ebias {ebias:.0f}')
        continue
    gc=cell(gest[0],gest[1])
    path=plan(gc)
    if len(path)<2:
        # nowhere known; nudge toward gest blindly
        b=math.atan2(gest[1]-y,gest[0]-x)
        drive_to(x+300*math.cos(b), y+300*math.sin(b))
        continue
    # follow up to 4 waypoints ahead
    wp=path[:6][-1]
    wx,wy=(wp[0]+0.5)*CELL,(wp[1]+0.5)*CELL
    ok=drive_to(wx,wy)
    if it%5==0:
        log(f'pose ({x:.0f},{y:.0f}) s={s} path len {len(path)} wp {wp} ok={ok}')
