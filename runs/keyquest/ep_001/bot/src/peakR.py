import sys, time, math
sys.path.insert(0,'/bot/src')
from lib import *
LOG=open('/bot/peak.log','a',buffering=1)
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
def med(n=6):
    a=[]
    for _ in range(n):
        vv=rdf(4)
        if vv is not None: a.append(vv)
        time.sleep(0.04)
    a.sort(); return a[len(a)//2] if a else 0
def chk():
    if goal():
        drive(0,0); log('GOAL!!!'); open('/memory/GOAL.txt','a').write('GOAL by peak.py\n'); sys.exit(0)
def turn_to(t,tol=12,tmax=9):
    while True:
        h=hdg(); d=angdiff(t,h)
        if abs(d)<tol: drive(0,0); return
        sp=4 if abs(d)<40 else 7
        drive(-sp,sp) if d>0 else drive(sp,-sp)
        time.sleep(0.15)

# phase 1: find the high-s corridor: climb s to >0.72 using simple biased moves
log('phase1: climb to s>0.70')
smax=0
t0=time.time()
while time.time()-t0<600:
    chk()
    s=med()
    smax=max(smax,s)
    if s>0.70: break
    v=lid(); h=hdg()
    # hill step: choose open dir maximizing quick probe = prefer north/east-ish rotation over time
    # use gradient probe: try best-open dir among all, do leg, keep if improves else reverse
    k=max(range(16),key=lambda k:min(v[(k-1)%16],v[k],v[(k+1)%16]))
    best=None
    # candidates: all k open enough
    cands=[k for k in range(16) if min(v[(k-1)%16],v[k],v[(k+1)%16])>0.3] or [k]
    # pick candidate that historically raises s: probe by short legs
    improved=False
    for k in cands[:6]:
        chk()
        tgt=(hdg()+k*22.5)%360
        # recompute relative each time
        v2=lid(); h2=hdg()
        kk=int(((tgt-h2)%360)/22.5+0.5)%16
        if min(v2[(kk-1)%16],v2[kk],v2[(kk+1)%16])<0.3: continue
        turn_to(tgt)
        s0=med(4)
        te=time.time()
        while time.time()-te<6:
            chk()
            vv=lid()
            if min(vv[15],vv[0],vv[1])<0.18: break
            drive(6,6); time.sleep(0.2)
        drive(0,0)
        s1=med(4)
        log(f'probe brg {tgt:.0f} s {s0:.3f}->{s1:.3f}')
        if s1>s0+0.003: improved=True; break
    if not improved:
        time.sleep(0.1)
log(f'phase1 done s={smax:.3f}')
# phase 2: acquire nearest wall and trace contour tightly (left hand), aggressive openings
log('phase2: tight RIGHT-hand contour trace')
v=lid()
k=min(range(16),key=lambda k:v[k])
turn_to((hdg()+k*22.5)%360)
# approach
while True:
    chk()
    v=lid()
    if min(v[15],v[0],v[1])<0.15: break
    drive(4,4); time.sleep(0.2)
drive(0,0)
# wall now in front: turn right so wall is on left
turn_to((hdg()+90)%360)
smax2=0; step=0
while True:
    step+=1
    chk()
    v=lid()
    front=min(v[15],v[0],v[1])
    left=min(v[12],v[13]); lfront=v[14]
    s=rdf(4) or 0
    if s>smax2:
        smax2=s
        if s>0.85: log(f'smax2={s:.3f}')
    if step%40==0: log(f'trace s={s:.3f} smax={smax2:.3f} f={front:.2f} l={left:.2f}')
    if front<0.13:
        drive(-5,5); time.sleep(0.22); continue
    if left>0.30 and lfront>0.30:
        drive(6,2)     # aggressive arc into any right opening
    elif left<0.09:
        drive(3,6)
    elif left>0.20:
        drive(6,3)
    else:
        drive(5,5)
    time.sleep(0.22)
