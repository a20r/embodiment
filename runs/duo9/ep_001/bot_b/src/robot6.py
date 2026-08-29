import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, math, json, random

log=open("/bot/src/tele6.log","a")
t0=time.time()
sm=None
X=Y=0.0
E7=read_float("d7"); E8=read_float("d8")
samples=[]  # (t,x,y,d5)
best=(0.0,0.0,-1.0)

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
    global sm,best
    l=lidar(); d5=read_float("d5"); d6=read_port("d6"); h=read_float("d1")
    upd_odo(h)
    sm = d5 if sm is None else 0.75*sm+0.25*d5
    now=time.time()-t0
    samples.append((now,X,Y,d5))
    while samples and samples[0][0]<now-35: samples.pop(0)
    if sm>best[2]: best=(X,Y,sm)
    rec=dict(t=round(now,1),x=round(X),y=round(Y),h=round(h,1),d5=d5,sm=round(sm,3),d6=d6,l=[round(v,2) for v in l])
    log.write(json.dumps(rec)+"\n"); log.flush()
    return rec,l,h

def grad():
    # plane fit d5 = a + gx*x + gy*y  (x,y in meters)
    if len(samples)<30: return None
    pts=[(x/1000.0,y/1000.0,s) for (_,x,y,s) in samples]
    n=len(pts)
    mx=sum(p[0] for p in pts)/n; my=sum(p[1] for p in pts)/n; ms=sum(p[2] for p in pts)/n
    sxx=sxy=syy=sxs=sys_=0
    for x,y,s in pts:
        dx,dy,ds=x-mx,y-my,s-ms
        sxx+=dx*dx; sxy+=dx*dy; syy+=dy*dy; sxs+=dx*ds; sys_+=dy*ds
    det=sxx*syy-sxy*sxy
    if abs(det)<1e-6: return None
    gx=(sxs*syy-sys_*sxy)/det
    gy=(sys_*sxx-sxs*sxy)/det
    mag=math.hypot(gx,gy)
    if mag<0.01: return None
    return gx,gy,mag

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
    return (h+22.5*besti)%360, besti

last_ping=0
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
    front=min(clear(l,0),clear(l,1),clear(l,15))
    if prev_front is not None and abs(front-prev_front)<0.02 and front<0.4:
        still+=1
    else: still=0
    prev_front=front
    if still>=3:
        motors(-80,-80); time.sleep(0.9); motors(0,0)
        th,_=pick_turn(l,h)
        turn_to(th); still=0
        continue
    # desired direction from gradient (or back to best)
    g=grad()
    prefer=None
    if best[2]-sm>0.12 and math.hypot(best[0]-X,best[1]-Y)>500:
        want=(math.degrees(math.atan2(best[0]-X,best[1]-Y)))%360
        prefer=int(round((((want-h)%360)/22.5)))%16
    elif g:
        want=(math.degrees(math.atan2(g[0],g[1])))%360
        prefer=int(round((((want-h)%360)/22.5)))%16
    if front<0.35:
        motors(0,0)
        th,_=pick_turn(l,h,prefer,0.12)
        turn_to(th)
        continue
    # periodic re-steer toward preference
    if prefer is not None and prefer not in (0,1,15) and now-last_steer>2.0:
        motors(0,0)
        th,bi=pick_turn(l,h,prefer,0.2)
        turn_to(th)
        last_steer=now
        continue
    steer=0
    if clear(l,1)<0.22 or clear(l,2)<0.14: steer=10
    if clear(l,15)<0.22 or clear(l,14)<0.14: steer=-10
    if prefer==1: steer+=8
    if prefer==15: steer-=8
    motors(90+steer,90-steer)
    time.sleep(0.2)
