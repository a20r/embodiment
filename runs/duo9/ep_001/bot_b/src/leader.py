import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, math, json

log=open("/bot/src/lead.log","a")
t0=time.time()

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

def goal_check(d6):
    return ("here=1" in d6) or ("goal=1" in d6)

last_ping=0; paused=False
still=0; prev_front=None
while True:
    l=lidar(); d5=read_float("d5"); d6=read_port("d6"); h=read_float("d1")
    now=time.time()
    log.write(json.dumps(dict(t=round(now-t0,1),d5=round(d5,3),d6=d6,h=round(h,1)))+"\n"); log.flush()
    if goal_check(d6):
        motors(0,0)
        print("GOALFLAG:",d6,flush=True)
        write_port("d0", json.dumps(dict(who="A",msg="A FOUND GOAL! here=1. I back off and wait for you. Climb d5 to me now!",d5=round(d5,3))))
        # back out of zone a little, wait for B close, then re-enter together
        # for now: stay and keep announcing; handle joint entry manually
        time.sleep(1)
        continue
    if now-last_ping>3:
        write_port("d0", json.dumps(dict(who="A",msg="A leading left-wall-follow, stay close",d5=round(d5,3))))
        last_ping=now
    # keep-in-range pausing
    if d5<0.5 and not paused:
        paused=True; motors(0,0)
        print(f"pause for B d5={d5:.2f}",flush=True)
    if paused:
        if d5>0.7:
            paused=False
            print("resume",flush=True)
        else:
            time.sleep(0.5); continue
    front=min(clear(l,0),clear(l,1),clear(l,15))
    if prev_front is not None and abs(front-prev_front)<0.02 and front<0.45: still+=1
    else: still=0
    prev_front=front
    if still>=4:
        motors(-70,-70); time.sleep(0.8); motors(0,0)
        turn_to((h+90)%360); still=0
        continue
    # LEFT wall follow: wall on left = beams 12..14
    wdist=min(clear(l,12),clear(l,13))
    if front<0.32:
        turn_to((h+90)%360)  # turn right at wall ahead
        continue
    if wdist>0.85:
        # lost left wall: arc left to reacquire
        motors(48,72); time.sleep(0.3)
    else:
        err=(wdist-0.32)
        steer=int(max(-15,min(15,err*-55)))  # too close -> steer right(+)
        motors(65+steer,65-steer)
        time.sleep(0.18)
