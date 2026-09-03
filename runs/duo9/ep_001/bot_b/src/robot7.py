import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, math, json, random

log=open("/bot/src/tele7.log","a")
t0=time.time()
sm=None
X=Y=0.0
E7=read_float("d7"); E8=read_float("d8")
samples=[]
target=None

def clear(l,i):
    v=l[i%16]
    return 3.0 if v<0 else v

def upd_odo(h):
    global X,Y,E7,E8
    e7=read_float("d7"); e8=read_float("d8")
    d=((e7-E7)+(e8-E8))/2.0
    E7,E8=e7,e8
    th=math.radians(h)
    X+=d*math.sin(th); Y+=d*math.cos(th)

def sense():
    global sm
    l=lidar(); d5=read_float("d5"); d6=read_port("d6"); h=read_float("d1")
    upd_odo(h)
    sm = d5 if sm is None else 0.75*sm+0.25*d5
    now=time.time()-t0
    samples.append((now,X,Y,d5))
    while samples and samples[0][0]<now-90: samples.pop(0)
    rec=dict(t=round(now,1),x=round(X),y=round(Y),h=round(h,1),d5=d5,sm=round(sm,3),d6=d6,l=[round(v,2) for v in l])
    log.write(json.dumps(rec)+"\n"); log.flush()
    return rec,l,h

def fit():
    if len(samples)<60: return None
    pts=[(x,y,s) for (_,x,y,s) in samples]
    x0=X; y0=Y
    best=None
    step=200
    for gx in range(int(x0)-4000,int(x0)+4001,step):
        for gy in range(int(y0)-4000,int(y0)+4001,step):
            num=den=0
            for x,y,s in pts:
                d=math.hypot(gx-x,gy-y)+50
                num+=s/d; den+=1/d**2
            k=num/den
            err=sum((s-k/(math.hypot(gx-x,gy-y)+50))**2 for x,y,s in pts)
            if best is None or err<best[0]: best=(err,gx,gy,k)
    return best

def turn_to(target_h):
    for _ in range(100):
        h=read_float("d1")
        err=((target_h-h+180)%360)-180
        if abs(err)<9: break
        sp=max(12,min(70,abs(err)*1.6))
        motors(sp if err>0 else -sp, -sp if err>0 else sp)
        time.sleep(0.07)
        upd_odo(read_float("d1"))
    motors(0,0)

def pick_turn(l,h,prefer_rel=None,strength=0.15):
    bestv=-1e9;besti=0
    for i in range(16):
        c=min(clear(l,i),clear(l,(i+1)%16),clear(l,(i-1)%16))
        v=min(c,1.5)
        if prefer_rel is not None:
            ad=min((i-prefer_rel)%16,(prefer_rel-i)%16)
            v-=ad*strength
        v+=random.uniform(0,0.2)
        if v>bestv: bestv=v;besti=i
    return (h+22.5*besti)%360

last_ping=0; last_fit=0
prev_front=None; still=0
last_steer=time.time()
while True:
    rec,l,h=sense()
    now=time.time()
    if "here=1" in rec["d6"] or "goal=1" in rec["d6"]:
        print("GOALFLAG:",rec["d6"],flush=True)
        motors(0,0)
        write_port("d0", json.dumps(dict(who="A",msg="AT_GOAL")))
        time.sleep(0.5)
        continue
    if now-last_ping>3:
        write_port("d0", json.dumps(dict(who="A",d5=round(sm,3))))
        last_ping=now
    if now-last_fit>12:
        f=fit()
        if f:
            err,gx,gy,k=f
            target=(gx,gy)
            dist=math.hypot(gx-X,gy-Y)
            print(f"t={rec['t']:.0f} fit src=({gx},{gy}) k={k:.0f} dist={dist:.0f} sm={sm:.3f}",flush=True)
        last_fit=now
    front=min(clear(l,0),clear(l,1),clear(l,15))
    if prev_front is not None and abs(front-prev_front)<0.02 and front<0.4:
        still+=1
    else: still=0
    prev_front=front
    if still>=3:
        motors(-80,-80); time.sleep(0.9); motors(0,0)
        turn_to(pick_turn(l,h)); still=0
        continue
    prefer=None
    if target:
        dx=target[0]-X; dy=target[1]-Y
        if math.hypot(dx,dy)>250:
            want=(math.degrees(math.atan2(dx,dy)))%360
            prefer=int(round((((want-h)%360)/22.5)))%16
    if front<0.35:
        motors(0,0)
        turn_to(pick_turn(l,h,prefer,0.10))
        continue
    if prefer is not None and prefer not in (0,1,15) and now-last_steer>2.0:
        motors(0,0)
        turn_to(pick_turn(l,h,prefer,0.2))
        last_steer=now
        continue
    steer=0
    if clear(l,1)<0.22 or clear(l,2)<0.14: steer=10
    if clear(l,15)<0.22 or clear(l,14)<0.14: steer=-10
    if prefer==1: steer+=8
    if prefer==15: steer-=8
    sp=90 if front>0.6 else 60
    motors(sp+steer,sp-steer)
    time.sleep(0.2)
