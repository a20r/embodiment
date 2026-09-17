import sys, time, math, json
sys.path.insert(0,"/bot/src")
from robot import speed, turn, stop, send
from nav import hd, sc, odo, ang_err

x=y=0.0
prev_o=None; prev_h=None
t0=time.time()
log=open("/memory/track.log","a")

def logrow(h, s):
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
    best=None;bs=-1
    for k in range(16):
        v=s[k]
        if v<0: v=3.2
        a=k*22.5
        if a>180: a=360-a
        w=1.0 if a<=67.5 else 0.5
        sc_=v*w
        if sc_>bs: bs=sc_;best=k
    return best,bs

BUDGET=1500
try:
    while time.time()-t0 < BUDGET:
        s=sc(); h=hd()
        if not s or h is None: time.sleep(0.1); continue
        logrow(h,s)
        fv=[v if v>=0 else 3.2 for v in (s[15],s[0],s[1],s[2])]
        front=min(fv)
        k,bs=steer_target(s)
        ta=k*22.5
        if ta>180: ta-=360
        e=(ta+180)%360-180
        turn(max(-40,min(40,-e*1.6)))
        sp = 4 if bs>1.5 and front>1.0 else (2.5 if front>0.45 else 1.2)
        if front<0.22: sp=0.6
        speed(sp)
        time.sleep(0.12)
finally:
    stop(); log.close(); print("explore2 done", flush=True)
