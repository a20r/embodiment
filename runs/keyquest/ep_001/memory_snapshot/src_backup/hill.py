import sys, time, math, random
sys.path.insert(0,'/bot/src')
from lib import *
LOG=open('/bot/hill.log','a',buffering=1)
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
def med(n=10):
    a=[]
    for _ in range(n):
        v=rdf(4)
        if v is not None: a.append(v)
        time.sleep(0.05)
    a.sort(); return a[len(a)//2]
def turn_to(t,tol=12,tmax=9):
    t0=time.time()
    while time.time()-t0<tmax:
        h=hdg(); d=angdiff(t,h)
        if abs(d)<tol: drive(0,0); return True
        s=4 if abs(d)<40 else 7
        drive(-s,s) if d>0 else drive(s,-s)
        time.sleep(0.15)
    drive(0,0); return False
def chkgoal():
    if goal():
        drive(0,0); log('GOAL!')
        open('/memory/GOAL.txt','a').write('GOAL by hill.py\n')
        sys.exit(0)
def openness(v,b,h):
    k=int(((b-h)%360)/22.5+0.5)%16
    return min(v[(k-1)%16],v[k],v[(k+1)%16])
x,y=0.0,0.0
last=enc()
bias=0.0
s0=med()
log('start s',s0)
leg=0
while True:
    chkgoal()
    v=lid(); h=hdg()
    # pick direction: candidates near bias that are open
    cands=sorted(range(16), key=lambda k: abs(angdiff((h+k*22.5)%360,bias)))
    tgt=None
    for k in cands:
        if min(v[(k-1)%16],v[k],v[(k+1)%16])>0.32:
            tgt=(h+k*22.5)%360; break
    if tgt is None:
        k=max(range(16),key=lambda k:min(v[(k-1)%16],v[k],v[(k+1)%16]))
        tgt=(h+k*22.5)%360
    turn_to(tgt)
    # drive a leg ~260 counts
    sx,sy=x,y
    dist=0; t0=time.time()
    while dist<160 and time.time()-t0<14:
        chkgoal()
        v=lid(); h=hdg()
        e=enc()
        if e and last:
            d=((e[0]-last[0])+(e[1]-last[1]))/2.0
            dist+=abs(d)
            x+=d*math.cos(math.radians(h)); y+=d*math.sin(math.radians(h))
        last=e
        fr=min(v[15],v[0],v[1])
        if fr<0.20:
            break
        l=min(v[1],v[2]); r=min(v[14],v[15])
        steer=0
        if l<0.14: steer=-1
        if r<0.14: steer=1
        drive(6-steer,6+steer)
        time.sleep(0.2)
    drive(0,0)
    s1=med()
    db=math.degrees(math.atan2(y-sy,x-sx))%360 if dist>60 else bias
    ds=s1-s0
    leg+=1
    log(f'leg{leg} moved {dist:.0f} brg {db:.0f} s {s0:.3f}->{s1:.3f} ds {ds:+.3f}')
    if ds>0.004 and dist>60:
        bias=db
    elif ds<-0.004 or dist<=60:
        bias=(db+180+random.uniform(-70,70))%360
    # else keep bias
    s0=s1
