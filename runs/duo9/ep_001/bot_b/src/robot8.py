import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, math, json, random

log=open("/bot/src/tele8.log","a")
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
    best=None
    for gx in range(int(X)-4000,int(X)+4001,200):
        for gy in range(int(Y)-4000,int(Y)+4001,200):
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
        if abs(err)<10: break
        sp=max(12,min(70,abs(err)*1.6))
        motors(sp if err>0 else -sp, -sp if err>0 else sp)
        time.sleep(0.07)
        upd_odo(read_float("d1"))
    motors(0,0)

def bearing_to_target(h):
    dx=target[0]-X; dy=target[1]-Y
    want=(math.degrees(math.atan2(dx,dy)))%360
    rel=((want-h)%360)
    return want, rel

state="GO"
side=1   # 1: wall on right(beam4), -1: wall on left(beam12)
hit_dist=None
last_ping=0; last_fit=0
follow_since=0
still=0; prev_front=None
goal_latched=False
while True:
    rec,l,h=sense()
    now=time.time()
    d6=rec["d6"]
    if "here=1" in d6 or "goal=1" in d6:
        if not goal_latched:
            print("GOALFLAG:",d6,flush=True)
            goal_latched=True
        motors(0,0)
        write_port("d0", json.dumps(dict(who="A",msg="AT_GOAL")))
        time.sleep(0.5)
        continue
    goal_latched=False
    if now-last_ping>3:
        write_port("d0", json.dumps(dict(who="A",d5=round(sm,3),state=state)))
        last_ping=now
    if now-last_fit>12:
        f=fit()
        if f:
            err,gx,gy,k=f
            target=(gx,gy)
            print(f"t={rec['t']:.0f} fit=({gx},{gy}) k={k:.0f} dist={math.hypot(gx-X,gy-Y):.0f} sm={sm:.3f} state={state}",flush=True)
        last_fit=now
    if target is None:
        # roam forward
        front=min(clear(l,0),clear(l,1),clear(l,15))
        if front<0.4:
            turn_to((h+random.choice([90,-90,180,45,-45]))%360)
        else:
            motors(90,90); time.sleep(0.2)
        continue
    front=min(clear(l,0),clear(l,1),clear(l,15))
    # stuck check
    if prev_front is not None and abs(front-prev_front)<0.02 and front<0.45:
        still+=1
    else: still=0
    prev_front=front
    if still>=4:
        motors(-80,-80); time.sleep(0.8); motors(0,0)
        turn_to((h+side*90)%360)
        still=0
        continue
    dist=math.hypot(target[0]-X,target[1]-Y)
    want,rel=bearing_to_target(h)
    if state=="GO":
        if front<0.35:
            state="FOLLOW"; hit_dist=dist; follow_since=now
            # choose side: wall goes on the side that's more blocked toward target
            left=min(clear(l,12),clear(l,13),clear(l,11))
            right=min(clear(l,4),clear(l,5),clear(l,3))
            side=1 if right<left else -1
            # turn parallel to wall: away from wall
            turn_to((h - side*90)%360)
            continue
        # steer toward target
        if 15<rel<180:
            turn_to(want)
        elif 180<=rel<345:
            turn_to(want)
        sp=90 if front>0.6 else 60
        steer=0
        if clear(l,1)<0.22 or clear(l,2)<0.14: steer=10
        if clear(l,15)<0.22 or clear(l,14)<0.14: steer=-10
        motors(sp+steer,sp-steer)
        time.sleep(0.2)
    else: # FOLLOW
        # exit condition: target direction reasonably clear and progress made
        relbeam=int(round(rel/22.5))%16
        cleartgt=min(clear(l,relbeam),clear(l,(relbeam+1)%16),clear(l,(relbeam-1)%16))
        if (dist<hit_dist-250 and cleartgt>0.7) or now-follow_since>40:
            state="GO"
            continue
        # wall following: keep wall at side distance ~0.30
        wb = 4 if side==1 else 12
        wdist=min(clear(l,wb),clear(l,(wb+side)%16))
        if front<0.3:
            # inner corner: turn away from wall
            turn_to((h - side*90)%360)
            continue
        err=(wdist-0.30)
        steer=max(-18,min(18,int(err*60)))*side
        if wdist>0.8:
            # lost wall (outer corner): arc toward wall side
            motors(60+12*side,60-12*side)
            time.sleep(0.3)
        else:
            motors(75+steer,75-steer)
            time.sleep(0.2)
