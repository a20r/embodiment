import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, json

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

hand=1
sm=read_float("d5"); best=sm
last_ping=0; still=0; prev_front=None
print("start bug",flush=True)
while True:
    l=lidar(); d5=read_float("d5"); d6=read_port("d6"); h=read_float("d1")
    sm=0.75*sm+0.25*d5
    now=time.time()
    if "here=1" in d6 or "goal=1" in d6:
        motors(0,0); print("GOALFLAG:",d6,flush=True)
        write_port("d0","GOALFOUND"); time.sleep(1); continue
    if sm>best: best=sm
    if now-last_ping>4:
        write_port("d0", json.dumps(dict(who="A",d5=round(sm,3),msg="A wall-circumnavigating to you; hold position, keep beaconing")))
        last_ping=now
    if best-sm>0.12:
        hand=-hand; best=sm
        print(f"switch hand -> {hand} sm={sm:.2f}",flush=True)
        turn_to((h+180)%360)
        continue
    front=min(clear(l,0),clear(l,1),clear(l,15))
    if prev_front is not None and abs(front-prev_front)<0.02 and front<0.45: still+=1
    else: still=0
    prev_front=front
    if still>=4:
        motors(-70,-70); time.sleep(0.8); motors(0,0)
        turn_to((h-hand*90)%360); still=0
        continue
    wb = 4 if hand==1 else 12
    wdist=min(clear(l,wb),clear(l,wb-hand))
    if front<0.32:
        turn_to((h-hand*90)%360); continue
    if wdist>0.85:
        motors(60+hand*12,60-hand*12); time.sleep(0.3)
    else:
        err=(wdist-0.32)
        steer=int(max(-15,min(15,err*55)))*hand
        motors(65+steer,65-steer)
        time.sleep(0.18)
