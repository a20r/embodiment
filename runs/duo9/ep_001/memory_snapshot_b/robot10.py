import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, math, json, random

log=open("/bot/src/tele10.log","a")
t0=time.time()
X=Y=0.0
E7=read_float("d7"); E8=read_float("d8")
mpts=[]

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

def avg_d5(n=7):
    vals=[read_float("d5") for _ in range(n)]
    vals.sort()
    return sum(vals[1:-1])/(len(vals)-2)

def turn_to(target_h):
    for _ in range(100):
        h=read_float("d1")
        err=((target_h-h+180)%360)-180
        if abs(err)<8: break
        sp=max(12,min(60,abs(err)*1.5))
        motors(sp if err>0 else -sp, -sp if err>0 else sp)
        time.sleep(0.07)
        upd_odo()
    motors(0,0)

def drive(dist_mm, speed=60):
    start=(E7+E8)/2.0
    while True:
        h=upd_odo()
        got=(E7+E8)/2.0-start
        if got>=dist_mm: break
        l=lidar()
        f=min(clear(l,0),clear(l,1),clear(l,15))
        if f<0.32: break
        steer=0
        if clear(l,1)<0.2 or clear(l,2)<0.13: steer=7
        if clear(l,15)<0.2 or clear(l,14)<0.13: steer=-7
        motors(speed+steer,speed-steer)
        time.sleep(0.12)
    motors(0,0)
    return (E7+E8)/2.0-start

def fit_source():
    if len(mpts)<5: return None
    pts=mpts[-25:]
    best=None
    for gx in range(int(X)-5000,int(X)+5001,250):
        for gy in range(int(Y)-5000,int(Y)+5001,250):
            num=den=0
            for x,y,s in pts:
                d=math.hypot(gx-x,gy-y)+100
                num+=s/d; den+=1/d**2
            k=num/den
            err=sum((s-k/(math.hypot(gx-x,gy-y)+100))**2 for x,y,s in pts)
            if best is None or err<best[0]: best=(err/len(pts),gx,gy,k)
    return best

last_ping=0; step=0
prev_sm=None; prev_dir=None
while True:
    if check_goal(): time.sleep(0.5); continue
    h=upd_odo()
    sm=avg_d5()
    mpts.append((X,Y,sm))
    if len(mpts)>60: mpts=mpts[-60:]
    log.write(json.dumps(dict(t=round(time.time()-t0,1),x=round(X),y=round(Y),sm=round(sm,3)))+"\n"); log.flush()
    if time.time()-last_ping>3:
        write_port("d0", json.dumps(dict(who="A",d5=round(sm,3))))
        last_ping=time.time()
    if sm>0.80:
        motors(0,0)
        write_port("d0", json.dumps(dict(who="A",msg="A PARKED (d5 high). B do final approach; I stay.",d5=round(sm,3))))
        time.sleep(2)
        continue
    step+=1
    tgt=None
    if step%3==0:
        f=fit_source()
        if f and f[0]<0.004:
            tgt=(f[1],f[2])
            print(f"t={time.time()-t0:.0f} fit=({f[1]},{f[2]}) k={f[3]:.0f} res={f[0]:.4f} sm={sm:.3f} pos=({X:.0f},{Y:.0f})",flush=True)
    l=lidar()
    # choose heading
    hd=None
    if tgt:
        want=(math.degrees(math.atan2(tgt[0]-X,tgt[1]-Y)))%360
        # nearest clear beam to want
        cands=[]
        for i in range(16):
            c=min(clear(l,i),clear(l,(i+1)%16),clear(l,(i-1)%16))
            if c<0.5: continue
            bh=(h+22.5*i)%360
            dg=abs(((bh-want+180)%360)-180)
            cands.append((dg,bh))
        if cands: hd=min(cands)[1]
    if hd is None:
        # momentum hill climb
        cands=[]
        for i in range(16):
            c=min(clear(l,i),clear(l,(i+1)%16),clear(l,(i-1)%16))
            if c<0.5: continue
            bh=(h+22.5*i)%360
            score=min(c,1.2)*0.3+random.uniform(0,0.2)
            if prev_dir is not None and prev_sm is not None:
                dh=abs(((bh-prev_dir+180)%360)-180)
                if sm>prev_sm+0.005: score-=dh/60.0
                else: score-=abs(120-dh)/60.0
            cands.append((-score,bh))
        if cands: hd=min(cands)[1]
    if hd is None:
        motors(-70,-70); time.sleep(1.0); motors(0,0)
        turn_to((h+180)%360)
        prev_dir=None
        continue
    prev_sm=sm; prev_dir=hd
    turn_to(hd)
    got=drive(500)
    if got<100: prev_dir=None
