import sys, time, math, random
sys.path.insert(0,'/bot/src')
from lib import *
LOG=open('/bot/east.log','a',buffering=1)
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
def med(n=8):
    a=[]
    for _ in range(n):
        vv=rdf(4)
        if vv is not None: a.append(vv)
        time.sleep(0.05)
    a.sort(); return a[len(a)//2] if a else 0
def chk():
    if goal():
        drive(0,0); log('GOAL!'); open('/memory/GOAL.txt','a').write('GOAL by east.py\n'); sys.exit(0)
PREF=20  # preferred bearing east-northeast
t0=time.time(); lastlog=0
while True:
    chk()
    v=lid(); h=hdg()
    s=med(4)
    if time.time()-lastlog>6:
        log(f's={s:.3f} h={h:.0f}')
        lastlog=time.time()
    # choose best beam: open and closest to PREF
    best=None;bs=-1e9
    for k in range(16):
        w=min(v[(k-1)%16],v[k],v[(k+1)%16])
        if w<0.30: continue
        sc=-abs(angdiff((h+k*22.5)%360,PREF))/180.0 + w*0.15
        if sc>bs: bs,sc2=sc,None;best=k
    if best is None:
        best=max(range(16),key=lambda k:min(v[(k-1)%16],v[k],v[(k+1)%16]))
    tgt=(h+best*22.5)%360
    # turn
    tt=time.time()
    while time.time()-tt<8:
        h=hdg(); d=angdiff(tgt,h)
        if abs(d)<14: break
        sp=4 if abs(d)<40 else 7
        drive(-sp,sp) if d>0 else drive(sp,-sp)
        time.sleep(0.15)
    # go while front open, slide along walls
    tt=time.time()
    while time.time()-tt<10:
        chk()
        v=lid(); h=hdg()
        fr=min(v[15],v[0],v[1])
        if fr<0.20: break
        l=min(v[1],v[2]); r=min(v[14],v[15])
        steer=0
        if l<0.14: steer=-1
        if r<0.14: steer=1
        drive(6-steer,6+steer)
        time.sleep(0.2)
    drive(0,0)
