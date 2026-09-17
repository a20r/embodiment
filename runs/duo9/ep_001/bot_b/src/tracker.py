import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, math, json, statistics, random

def clear(l,i):
    v=l[i%16]
    return 3.0 if v<0 else v

def check_goal():
    d6=read_port("d6")
    if "here=1" in d6 or "goal=1" in d6:
        print("GOALFLAG:",d6,flush=True); motors(0,0)
        write_port("d0", json.dumps(dict(who="A",msg="AT_GOAL")))
        return True
    return False

def turn_to(target_h):
    for _ in range(100):
        h=read_float("d1")
        err=((target_h-h+180)%360)-180
        if abs(err)<8: break
        sp=max(12,min(60,abs(err)*1.5))
        motors(sp if err>0 else -sp, -sp if err>0 else sp)
        time.sleep(0.07)
    motors(0,0)

def drive(dist_mm, speed=70):
    e0=(read_float("d7")+read_float("d8"))/2
    while True:
        e=(read_float("d7")+read_float("d8"))/2
        if e-e0>=dist_mm: break
        l=lidar()
        f=min(clear(l,0),clear(l,1),clear(l,15))
        if f<0.30: break
        steer=0
        if clear(l,1)<0.2: steer=7
        if clear(l,15)<0.2: steer=-7
        motors(speed+steer,speed-steer)
        time.sleep(0.12)
    motors(0,0)
    return (read_float("d7")+read_float("d8"))/2-e0

def scan_for_mover(secs=9):
    motors(0,0)
    h=read_float("d1")
    scans=[]
    t0=time.time()
    while time.time()-t0<secs:
        scans.append(lidar())
        time.sleep(0.25)
    hits=[]
    for i in range(16):
        vals=[s[i] for s in scans if s[i]>0]
        if len(vals)<12: continue
        med=statistics.median(vals)
        dips=[v for v in vals if v<med-0.3]
        if len(dips)>=3:
            hits.append((i,min(dips),med,(h+22.5*i)%360))
    return h,hits

def avg_d5(n=6):
    vals=sorted(read_float("d5") for _ in range(n))
    return sum(vals[1:-1])/(len(vals)-2)

prev=None; prev_dir=None
while True:
    if check_goal(): time.sleep(0.5); continue
    h,hits=scan_for_mover()
    if hits:
        hits.sort(key=lambda x:x[1])
        b,rmin,med,brg=hits[0]
        print(f"MOVER beam{b} bearing{brg:.0f} rmin={rmin:.2f} med={med:.2f}",flush=True)
        write_port("d0", json.dumps(dict(who="A",msg=f"B: I see you at my bearing {brg:.0f} range {rmin:.1f}m. Approaching. Keep moving/wiggling.")))
        turn_to(brg)
        drive(rmin*1000-250)
        continue
    # hill climb 2 steps
    for _ in range(2):
        if check_goal(): break
        sm=avg_d5()
        write_port("d0", json.dumps(dict(who="A",d5=round(sm,3),msg="A hunting; no visual yet")))
        l=lidar(); h=read_float("d1")
        cands=[]
        for i in range(16):
            c=min(clear(l,i),clear(l,(i+1)%16),clear(l,(i-1)%16))
            if c<0.5: continue
            bh=(h+22.5*i)%360
            score=min(c,1.2)*0.3+random.uniform(0,0.25)
            if prev_dir is not None and prev is not None:
                dh=abs(((bh-prev_dir+180)%360)-180)
                if sm>prev+0.005: score-=dh/60.0
                else: score-=abs(120-dh)/60.0
            cands.append((-score,bh))
        if not cands:
            motors(-70,-70); time.sleep(0.9); motors(0,0); continue
        cands.sort()
        prev=sm; prev_dir=cands[0][1]
        turn_to(cands[0][1])
        drive(450)
