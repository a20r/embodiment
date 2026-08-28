import sys, time, math, json
sys.path.insert(0,'/bot/src')
from lib import *
LOG=open('/bot/goto.log','a',buffering=1)
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
def turn_to(target,tol=14,tmax=10):
    t0=time.time()
    while time.time()-t0<tmax:
        h=hdg(); d=angdiff(target,h)
        if abs(d)<tol: drive(0,0); return True
        s=4 if abs(d)<40 else 7
        drive(-s,s) if d>0 else drive(s,-s)
        time.sleep(0.15)
    drive(0,0); return False
def pick(v,h,bias):
    best,bs=None,-1e9
    for k in range(16):
        w=min(v[(k-1)%16],v[k],v[(k+1)%16])
        if w<0.30: continue
        sc=w*0.6-abs(angdiff((h+k*22.5)%360,bias))/180.0
        if sc>bs: bs,best=sc,k
    if best is None:
        for k in range(16):
            w=min(v[(k-1)%16],v[k],v[(k+1)%16])
            if w>bs: bs,best=w,k
    return (h+best*22.5)%360
POSE='/memory/pose.json'
x,y=0.0,0.0
try:
    with open(POSE) as f: p=json.load(f); x,y=p['x'],p['y']
except Exception: pass
last=enc()
pf=open('/memory/points.csv','a',buffering=1)
step=0; lastsave=0
log('start pose',x,y)
stuck=0; px,py=x,y
while True:
    step+=1
    if goal():
        drive(0,0); log('GOAL! pose',x,y)
        with open('/memory/GOAL.txt','a') as f: f.write(f'GOAL by goto.py pose {x:.0f},{y:.0f}\n')
        break
    try:
        with open('/bot/target.json') as f: tg=json.load(f)
        tx,ty=tg['x'],tg['y']
    except Exception:
        tx,ty=x,y
    v=lid(); h=hdg()
    e=enc()
    if e and last:
        d=((e[0]-last[0])+(e[1]-last[1]))/2.0
        x+=d*math.cos(math.radians(h)); y+=d*math.sin(math.radians(h))
    last=e
    s=rdf(4)
    if s is not None and step%3==0: pf.write(f'{x:.1f},{y:.1f},{s}\n')
    if time.time()-lastsave>5:
        with open(POSE,'w') as f: json.dump({'x':x,'y':y},f)
        lastsave=time.time()
        log(f'pose ({x:.0f},{y:.0f}) s={s} tgt=({tx:.0f},{ty:.0f}) dist={math.hypot(tx-x,ty-y):.0f}')
        # stuck detection
        if math.hypot(x-px,y-py)<15: stuck+=1
        else: stuck=0
        px,py=x,y
        if stuck>=2:
            log('stuck! wall-follow escape 25s')
            drive(-6,-6); time.sleep(1.0)
            tesc=time.time()
            while time.time()-tesc<25:
                if goal():
                    drive(0,0); log('GOAL during escape')
                    open('/memory/GOAL.txt','a').write('GOAL during escape\n')
                    sys.exit(0)
                vv=lid(); hh=hdg()
                ee=enc()
                if ee and last:
                    dd=((ee[0]-last[0])+(ee[1]-last[1]))/2.0
                    x+=dd*math.cos(math.radians(hh)); y+=dd*math.sin(math.radians(hh))
                last=ee
                fr=min(vv[15],vv[0],vv[1]); rt=min(vv[12],vv[13]); rf=vv[14]
                if fr<0.22: drive(-5,5); time.sleep(0.25); continue
                if rt>0.55 and rf>0.4: drive(6,2)
                elif rt<0.16: drive(3,6)
                elif rt>0.32: drive(6,3)
                else: drive(6,6)
                time.sleep(0.25)
            stuck=0
    if math.hypot(tx-x,ty-y)<60:
        drive(0,0); log('ARRIVED at target'); time.sleep(2); continue
    bias=math.degrees(math.atan2(ty-y,tx-x))%360
    front=min(v[15],v[0],v[1])
    dh=angdiff(bias,h)
    if front>0.25 and abs(dh)<50:
        steer=max(-2,min(2,dh/25))
        l=min(v[1],v[2]); r=min(v[14],v[15])
        if l<0.15: steer=min(steer,-1)
        if r<0.15: steer=max(steer,1)
        drive(6-steer,6+steer); time.sleep(0.3)
    elif front>0.25:
        turn_to(pick(v,h,bias))
    else:
        drive(-5,-5); time.sleep(0.6); drive(0,0)
        turn_to(pick(lid(),hdg(),bias))
