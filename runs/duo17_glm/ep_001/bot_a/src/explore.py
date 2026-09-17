import sys, time, math
sys.path.insert(0,"/bot/src")
from robot import speed, turn, stop, send
from nav import hd, sc, odo, ang_err
import json

x=y=0.0
prev_o=None; prev_h=None
t0=time.time()
log=open("/memory/track.log","a")

def logrow(h, s, extra=""):
    global x,y,prev_o,prev_h
    o=odo()
    if o is not None and prev_o is not None:
        dd=o-prev_o
        hh=h if prev_h is None else (h+prev_h)/2
        x+=dd*math.cos(math.radians(hh)); y+=dd*math.sin(math.radians(hh))
    if o is not None: prev_o=o
    prev_h=h
    log.write(json.dumps({"t":round(time.time()-t0,2),"h":round(h,1),"x":round(x,2),"y":round(y,2),"s":[round(v,2) for v in s]})+"\n")
    log.flush()

def steer_target(s):
    # prefer near-forward open directions; score = range * w(angle)
    best=None;bs=-1
    for k in range(16):
        v=s[k]
        if v<0: v=3.2  # no return = far
        a=k*22.5
        w=1.0 if a<67.5 or a>292.5 else 0.55
        sc_=v*w
        if sc_>bs: bs=sc_;best=k
    return best, bs

try:
    while time.time()-t0 < 600:
        s=sc(); h=hd()
        if not s or h is None: time.sleep(0.1); continue
        logrow(h,s)
        front=min(s[15],s[0],s[1],s[2])
        frontv=[v if v>=0 else 3.2 for v in (s[15],s[0],s[1],s[2])]
        # steering: follow widest gap near forward
        k,bs=steer_target(s)
        if bs<0.5:
            # boxed in: rotate to find opening
            speed(0); turn(30)
            time.sleep(0.6); turn(0)
            continue
        ta=k*22.5  # target relative angle (beam k)
        e=(ta+180)%360-180
        turn(max(-40,min(40,-e*1.5)))
        sp = 3 if bs>1.0 else 1.5
        if min(frontv)<0.35 and abs(e)<40: sp=min(sp,1.0)
        speed(sp)
        time.sleep(0.15)
finally:
    stop()
    log.close()
    print("explorer done", flush=True)
