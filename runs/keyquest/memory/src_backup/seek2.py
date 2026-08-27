import sys, time, math, random
sys.path.insert(0,'/bot/src')
from lib import *
LOG=open('/bot/seek2.log','a',buffering=1)
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

def turn_to(target, tol=12, tmax=10):
    t0=time.time()
    while time.time()-t0<tmax:
        h=hdg(); d=angdiff(target,h)
        if abs(d)<tol: drive(0,0); return True
        s=4 if abs(d)<40 else 7
        drive(-s,s) if d>0 else drive(s,-s)
        time.sleep(0.15)
    drive(0,0); return False

def grad(pts):
    # fit s=a x + b y + c ; return bearing, magnitude
    n=len(pts)
    if n<20: return None
    sx=sum(p[0] for p in pts); sy=sum(p[1] for p in pts); ss=sum(p[2] for p in pts)
    sxx=sum(p[0]**2 for p in pts); syy=sum(p[1]**2 for p in pts); sxy=sum(p[0]*p[1] for p in pts)
    sxs=sum(p[0]*p[2] for p in pts); sys_=sum(p[1]*p[2] for p in pts)
    A=[[sxx,sxy,sx],[sxy,syy,sy],[sx,sy,n]]; b=[sxs,sys_,ss]
    try:
        M=[row[:] for row in A]; v=b[:]
        for i in range(3):
            p=M[i][i]
            if abs(p)<1e-12: return None
            for j in range(i+1,3):
                f=M[j][i]/p
                for k in range(3): M[j][k]-=f*M[i][k]
                v[j]-=f*v[i]
        sol=[0,0,0]
        for i in range(2,-1,-1):
            sol[i]=(v[i]-sum(M[i][k]*sol[k] for k in range(i+1,3)))/M[i][i]
        a,bb,c=sol
        mag=math.hypot(a,bb)
        return (math.degrees(math.atan2(bb,a))%360, mag)
    except Exception:
        return None

def pick(v,h,bias):
    # choose beam: clearance-weighted, prefer bias bearing
    best,bs=None,-1e9
    for k in range(16):
        w=min(v[(k-1)%16],v[k],v[(k+1)%16])
        if w<0.30: continue
        sc = w*0.6 - abs(angdiff((h+k*22.5)%360,bias))/180.0
        if sc>bs: bs,best=sc,k
    if best is None:
        for k in range(16):
            w=min(v[(k-1)%16],v[k],v[(k+1)%16])
            if w>bs: bs,best=w,k
    return (h+best*22.5)%360

x,y=0.0,0.0
last=enc()
pts=[]  # (x,y,s)
trace=open('/memory/trace3.csv','a',buffering=1)
t0=time.time()
bias=random.uniform(0,360)
best=(0.0,0.0,-1.0)
allpts=open('/memory/points.csv','a',buffering=1)
last_fit=0
step=0
while True:
    step+=1
    if goal():
        drive(0,0); log('GOAL! pose',x,y)
        with open('/memory/GOAL.txt','a') as f: f.write(f'GOAL reached seek2 pose {x:.0f},{y:.0f}\n')
        break
    v=lid(); h=hdg()
    e=enc()
    if e and last:
        d=((e[0]-last[0])+(e[1]-last[1]))/2.0
        x+=d*math.cos(math.radians(h)); y+=d*math.sin(math.radians(h))
    last=e
    s=rdf(4)
    if s is not None:
        pts.append((x,y,s))
        if len(pts)>150: pts.pop(0)
        allpts.write(f'{x:.1f},{y:.1f},{s}\n')
        # running best (smoothed): use median-ish of last 5
        recent=[p[2] for p in pts[-5:]]
        rs=sorted(recent)[len(recent)//2]
        if rs>best[2]:
            best=(x,y,rs)
    if time.time()-last_fit>10 and len(pts)>=40:
        g=grad(pts)
        if g and g[1]>1e-6:
            bias=g[0]
            log(f'gradient fit: bearing {bias:.0f} mag {g[1]:.2e} pose ({x:.0f},{y:.0f}) s={s} best={best[2]:.3f}@({best[0]:.0f},{best[1]:.0f})')
        if s is not None and best[2]>0 and s<best[2]-0.03:
            bias=math.degrees(math.atan2(best[1]-y,best[0]-x))%360
            log(f'returning to best {best[2]:.3f} at ({best[0]:.0f},{best[1]:.0f}) bias {bias:.0f}')
        last_fit=time.time()
        trace.write(f'{time.time()-t0:.0f},{x:.1f},{y:.1f},{s},{bias:.0f}\n')
    front=min(v[15],v[0],v[1])
    dh=angdiff(bias,h)
    if front>0.25 and abs(dh)<50:
        # steer proportionally toward bias, avoid side walls
        steer=max(-2,min(2,dh/25))
        l=min(v[1],v[2]); r=min(v[14],v[15])
        if l<0.15: steer=min(steer,-1)
        if r<0.15: steer=max(steer,1)
        drive(6-steer,6+steer)
        time.sleep(0.35)
    elif front>0.25 and abs(dh)>=50:
        tgt=pick(v,h,bias)
        turn_to(tgt)
    else:
        drive(-5,-5); time.sleep(0.8); drive(0,0)
        v=lid(); h=hdg()
        tgt=pick(v,h,bias)
        if abs(angdiff(tgt,h))<15:
            # forced widest regardless of bias
            bw,bk=-1,0
            for k in range(16):
                w=min(v[(k-1)%16],v[k],v[(k+1)%16])
                if w>bw: bw,bk=w,k
            tgt=(h+bk*22.5)%360
        log(f'blocked f={front:.2f} h={h:.0f} -> {tgt:.0f} (bias {bias:.0f})')
        turn_to(tgt)
