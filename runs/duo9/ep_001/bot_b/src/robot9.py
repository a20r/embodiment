import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, math, json, random

log=open("/bot/src/tele9.log","a")
t0=time.time()
X=Y=0.0
E7=read_float("d7"); E8=read_float("d8")
mpts=[]  # stationary measure points (x,y,sm)

def clear(l,i):
    v=l[i%16]
    return 3.0 if v<0 else v

def upd_odo(h=None):
    global X,Y,E7,E8
    if h is None: h=read_float("d1")
    e7=read_float("d7"); e8=read_float("d8")
    d=((e7-E7)+(e8-E8))/2.0
    E7,E8=e7,e8
    th=math.radians(h)
    X+=d*math.sin(th); Y+=d*math.cos(th)
    return h

def check_goal():
    d6=read_port("d6")
    if "here=1" in d6 or "goal=1" in d6:
        print("GOALFLAG:",d6,flush=True)
        motors(0,0)
        write_port("d0", json.dumps(dict(who="A",msg="AT_GOAL")))
        return True
    return False

def avg_d5(n=8):
    vals=[]
    for _ in range(n):
        vals.append(read_float("d5"))
        time.sleep(0.1)
    vals.sort()
    return sum(vals[1:-1])/(len(vals)-2)

def turn_to(target_h):
    for _ in range(100):
        h=read_float("d1")
        err=((target_h-h+180)%360)-180
        if abs(err)<8: break
        sp=max(12,min(70,abs(err)*1.6))
        motors(sp if err>0 else -sp, -sp if err>0 else sp)
        time.sleep(0.07)
        upd_odo()
    motors(0,0)

def drive(dist_mm, speed=80):
    # drive forward up to dist; stop early if blocked. return actual mm
    start=(E7+E8)/2.0
    last_l=None
    while True:
        h=upd_odo()
        got=(E7+E8)/2.0-start
        if got>=dist_mm: break
        l=lidar()
        f=min(clear(l,0),clear(l,1),clear(l,15))
        if f<0.28:
            break
        steer=0
        if clear(l,1)<0.2 or clear(l,2)<0.13: steer=8
        if clear(l,15)<0.2 or clear(l,14)<0.13: steer=-8
        motors(speed+steer,speed-steer)
        time.sleep(0.12)
        if check_goal(): motors(0,0); return -1
    motors(0,0)
    return (E7+E8)/2.0-start

def grad_dir():
    # from last few measure points, weighted gradient
    if len(mpts)<3: return None
    pts=mpts[-6:]
    n=len(pts)
    mx=sum(p[0] for p in pts)/n; my=sum(p[1] for p in pts)/n; ms=sum(p[2] for p in pts)/n
    sxx=sxy=syy=sxs=sys_=0
    for x,y,s in pts:
        dx,dy,ds=(x-mx)/1000,(y-my)/1000,s-ms
        sxx+=dx*dx; sxy+=dx*dy; syy+=dy*dy; sxs+=dx*ds; sys_+=dy*ds
    det=sxx*syy-sxy*sxy
    if abs(det)<1e-9: return None
    gx=(sxs*syy-sys_*sxy)/det
    gy=(sys_*sxx-sxs*sxy)/det
    if math.hypot(gx,gy)<0.005: return None
    return (math.degrees(math.atan2(gx,gy)))%360

last_ping=0
prev=None
prev_dir=None
while True:
    if check_goal():
        time.sleep(0.5); continue
    h=upd_odo()
    sm=avg_d5()
    mpts.append((X,Y,sm))
    if len(mpts)>40: mpts=mpts[-40:]
    log.write(json.dumps(dict(t=round(time.time()-t0,1),x=round(X),y=round(Y),sm=round(sm,3)))+"\n"); log.flush()
    if time.time()-last_ping>3:
        write_port("d0", json.dumps(dict(who="A",d5=round(sm,3))))
        last_ping=time.time()
    l=lidar()
    gd=grad_dir()
    # candidate directions ranked
    cands=[]
    for i in range(16):
        c=min(clear(l,i),clear(l,(i+1)%16),clear(l,(i-1)%16))
        if c<0.45: continue
        hd=(h+22.5*i)%360
        score=0
        if prev is not None and prev_dir is not None:
            # momentum if last step improved
            dh=abs(((hd-prev_dir+180)%360)-180)
            if sm>prev+0.008: score-= dh/45.0
            else: score-= abs(90-dh)/45.0*0.5 + (1 if dh<30 else 0)
        if gd is not None:
            dg=abs(((hd-gd+180)%360)-180)
            score-=dg/60.0
        score+=min(c,1.2)*0.4+random.uniform(0,0.15)
        cands.append((score,hd,i))
    if not cands:
        motors(-80,-80); time.sleep(1.0); motors(0,0)
        turn_to((h+180)%360)
        prev=None; prev_dir=None
        continue
    cands.sort(reverse=True)
    _,hd,i=cands[0]
    prev=sm; prev_dir=hd
    turn_to(hd)
    got=drive(650)
    if got==-1: continue
    if got<120:
        prev_dir=None  # blocked, don't reward
