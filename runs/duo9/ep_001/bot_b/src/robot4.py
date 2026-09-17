import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, math, json, random

log=open("/bot/src/tele4.log","a")
t0=time.time()
sm=None
hist=[]  # (t, sm)

def clear(l,i):
    v=l[i%16]
    return 3.0 if v<0 else v

def sense():
    global sm
    l=lidar(); d5=read_float("d5"); d6=read_port("d6"); h=read_float("d1")
    sm = d5 if sm is None else 0.75*sm+0.25*d5
    now=time.time()-t0
    hist.append((now,sm))
    while hist and hist[0][0]<now-8: hist.pop(0)
    rec=dict(t=round(now,1),h=round(h,1),d5=d5,sm=round(sm,3),d6=d6,l=[round(v,2) for v in l])
    log.write(json.dumps(rec)+"\n"); log.flush()
    return rec,l,h

def slope():
    if len(hist)<6: return 0
    now=hist[-1][0]
    old=[s for (tt,s) in hist if tt<now-2.0]
    new=[s for (tt,s) in hist if tt>=now-1.0]
    if not old or not new: return 0
    return (sum(new)/len(new)) - (sum(old[-4:])/len(old[-4:]))

def turn_to(target_h):
    for _ in range(100):
        h=read_float("d1")
        err=((target_h-h+180)%360)-180
        if abs(err)<9: break
        sp=max(12,min(70,abs(err)*1.6))
        motors(sp if err>0 else -sp, -sp if err>0 else sp)
        time.sleep(0.07)
    motors(0,0)

def pick_turn(l,h,prefer_rel=None):
    # choose clear direction; prefer_rel = desired relative beam (0..15)
    bestv=-1e9;besti=0
    for i in range(16):
        c=min(clear(l,i),clear(l,(i+1)%16),clear(l,(i-1)%16))
        v=min(c,1.5)
        if prefer_rel is not None:
            ad=min((i-prefer_rel)%16,(prefer_rel-i)%16)
            v-=ad*0.12
        v+=random.uniform(0,0.3)
        if v>bestv: bestv=v;besti=i
    return (h+22.5*besti)%360

last_ping=0
prev_front=None; still=0
run_since=time.time()
while True:
    rec,l,h=sense()
    now=time.time()
    if "here=1" in rec["d6"] or "goal=1" in rec["d6"]:
        print("GOALFLAG:",rec["d6"],flush=True)
        motors(0,0)
        write_port("d0","AT GOAL")
        time.sleep(0.5)
        continue
    if now-last_ping>3:
        write_port("d0", f"A sm={sm:.3f}")
        last_ping=now
    front=min(clear(l,0),clear(l,1),clear(l,15))
    # stuck?
    if prev_front is not None and abs(front-prev_front)<0.02 and front<0.4:
        still+=1
    else: still=0
    prev_front=front
    if still>=3:
        motors(-80,-80); time.sleep(0.9); motors(0,0)
        turn_to(pick_turn(l,h))
        still=0; run_since=now
        continue
    if front<0.35:
        motors(0,0)
        # prefer roughly continuing (rel 0) else anything clear
        turn_to(pick_turn(l,h))
        run_since=now
        continue
    sl=slope()
    if sl<-0.006 and now-run_since>2.5:
        motors(0,0)
        print(f"t={rec['t']} tumble sm={sm:.3f} slope={sl:.4f}",flush=True)
        # turn roughly 90-180 away
        rel=random.choice([4,5,6,7,8,9,10,11,12])
        turn_to(pick_turn(l,h,prefer_rel=rel))
        run_since=now
        continue
    steer=0
    if clear(l,1)<0.22 or clear(l,2)<0.14: steer=10
    if clear(l,15)<0.22 or clear(l,14)<0.14: steer=-10
    motors(90+steer,90-steer)
    time.sleep(0.2)
