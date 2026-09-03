import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, math, json, os, random

def clear(l,i):
    v=l[i%16]
    return 3.0 if v<0 else v

def turn_to(target_h):
    for _ in range(100):
        h=read_float("d1")
        err=((target_h-h+180)%360)-180
        if abs(err)<9: break
        sp=max(12,min(60,abs(err)*1.5))
        motors(sp if err>0 else -sp, -sp if err>0 else sp)
        time.sleep(0.07)
    motors(0,0)

def b_at_goal():
    try:
        with open("/bot/src/rx.log","rb") as f:
            f.seek(max(0,os.path.getsize("/bot/src/rx.log")-2000))
            tail=f.read().decode(errors="ignore")
        for ln in tail.splitlines():
            if ln.strip().endswith("RX: GOALFOUND"): return True
        return False
    except: return False

last_ping=0; still=0; prev_front=None
mode="SWEEP"
t0=time.time()
while True:
    l=lidar(); d5=read_float("d5"); d6=read_port("d6"); h=read_float("d1")
    now=time.time()
    if "here=1" in d6 or "goal=1" in d6:
        motors(0,0)
        print("GOALFLAG:",d6,flush=True)
        write_port("d0","GOALFOUND")
        time.sleep(1.5)
        continue
    if mode=="SWEEP" and b_at_goal():
        mode="TOB"
        print("B AT GOAL -> homing to B",flush=True)
    if now-last_ping>5:
        write_port("d0", json.dumps(dict(who="A",d5=round(d5,3),msg="A sweeping right-hand")))
        last_ping=now
    front=min(clear(l,0),clear(l,1),clear(l,15))
    if prev_front is not None and abs(front-prev_front)<0.02 and front<0.45: still+=1
    else: still=0
    prev_front=front
    if still>=4:
        motors(-70,-70); time.sleep(0.8); motors(0,0)
        turn_to((h-90)%360); still=0
        continue
    if mode=="TOB":
        # simple d5 climb: reuse momentum climb
        motors(0,0)
        vals=sorted(read_float("d5") for _ in range(5)); sm=sum(vals[1:-1])/3
        if sm>0.93:
            # near B: creep
            motors(30,30); time.sleep(0.4); motors(0,0); continue
        # gradient step
        best=None
        for i in range(16):
            c=min(clear(l,i),clear(l,(i+1)%16),clear(l,(i-1)%16))
            if c<0.5: continue
            sc=c+random.uniform(0,0.3)
            if best is None or sc>best[0]: best=(sc,(h+22.5*i)%360)
        if best: 
            turn_to(best[1])
            e0=read_float("d7")
            while read_float("d7")-e0<500:
                l2=lidar()
                if min(clear(l2,0),clear(l2,1),clear(l2,15))<0.3: break
                motors(70,70); time.sleep(0.12)
            motors(0,0)
            v2=sorted(read_float("d5") for _ in range(5)); sm2=sum(v2[1:-1])/3
            if sm2<sm-0.01:
                turn_to((read_float("d1")+180)%360)  # undo next round
        continue
    # RIGHT-hand wall follow: wall on right = beams 3,4
    wdist=min(clear(l,4),clear(l,3))
    if front<0.32:
        turn_to((h-90)%360)  # turn left at wall
        continue
    if wdist>0.85:
        motors(72,48); time.sleep(0.3)  # arc right to find wall
    else:
        err=(wdist-0.32)
        steer=int(max(-15,min(15,err*55)))  # too far from right wall -> steer right(+)
        motors(65+steer,65-steer)
        time.sleep(0.18)
