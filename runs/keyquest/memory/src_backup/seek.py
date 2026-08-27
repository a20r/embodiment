import sys, time, math, random
sys.path.insert(0,'/bot/src')
from lib import *
LOG=open('/bot/seek.log','a',buffering=1)
def log(*a): print(time.strftime('%H:%M:%S'),*a,file=LOG)

def sig(n=4):
    s=[]
    for _ in range(n):
        v=rdf(4)
        if v is not None: s.append(v)
        time.sleep(0.05)
    return sum(s)/len(s) if s else 0.0

def angdiff(a,b):
    d=(a-b)%360
    return d-360 if d>180 else d

def turn_to(target, tol=12, tmax=10):
    t0=time.time()
    while time.time()-t0<tmax:
        h=hdg(); d=angdiff(target,h)
        if abs(d)<tol: drive(0,0); return True
        s=4 if abs(d)<40 else 7
        drive(-s,s) if d>0 else drive(s,-s)
        time.sleep(0.15)
    drive(0,0); return False

prev=[0.5]*16
def lid():
    global prev
    v=lidar()
    if v is None: return prev
    v=[prev[i] if v[i]<0 else v[i] for i in range(16)]
    prev=v
    return v

def widest(v,h,bias=None):
    # pick beam with best window clearance; optional bias toward bearing
    best,bs=None,-1
    for k in range(16):
        w=min(v[(k-1)%16],v[k],v[(k+1)%16])
        sc=w
        if bias is not None:
            sc-= abs(angdiff(h+k*22.5,bias))/360*0.3
        if sc>bs: bs,best=sc,k
    return (h+best*22.5)%360, min(v[(best-1)%16],v[best],v[(best+1)%16])

trace=open('/memory/trace2.csv','a',buffering=1)
t0=time.time()
s_prev=sig()
log('start sig',s_prev)
last_progress=time.time()
while True:
    if goal():
        drive(0,0); log('GOAL!')
        with open('/memory/GOAL.txt','a') as f: f.write('GOAL reached by seek.py\n')
        break
    v=lid(); h=hdg()
    front=min(v[15],v[0],v[1])
    if front>0.22:
        l=min(v[1],v[2]); r=min(v[14],v[15])
        if l>r+0.1: drive(4,6)
        elif r>l+0.1: drive(6,4)
        else: drive(6,6)
        time.sleep(0.6)
        s=sig()
        trace.write(f'{time.time()-t0:.1f},{h:.1f},{s:.4f},{front:.2f}\n')
        if s<s_prev-0.004:
            # signal dropping: tumble to a new direction, prefer open+random
            drive(0,0)
            tgt,clr=widest(lid(),hdg(),bias=(h+180+random.uniform(-60,60))%360)
            log(f'sig drop {s_prev:.3f}->{s:.3f}, tumble to {tgt:.0f}')
            turn_to(tgt)
            s_prev=sig()
        else:
            if s>s_prev: s_prev=s
    else:
        drive(0,0)
        tgt,clr=widest(v,h)
        log(f'blocked f={front:.2f} turn to {tgt:.0f} clr={clr:.2f}')
        turn_to(tgt)
        s_prev=sig()
