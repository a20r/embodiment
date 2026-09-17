import sys,time,math,json
sys.path.insert(0,"/bot/src")
from robot import speed,turn,stop
from nav import hd,sc,odo,ang_err
x=y=0.0; prev_o=None; prev_h=None
t0=time.time()
log=open("/memory/track.log","a")
def logrow(h,s):
    global x,y,prev_o,prev_h
    o=odo()
    if o is not None and prev_o is not None:
        dd=o-prev_o; hh=h if prev_h is None else (h+prev_h)/2
        x+=dd*math.cos(math.radians(hh)); y+=dd*math.sin(math.radians(hh))
    if o is not None: prev_o=o
    prev_h=h
    log.write(json.dumps({"t":round(time.time()-t0,2),"h":round(h,1),"x":round(x,2),"y":round(y,2),"s":[round(v,2) for v in s]})+"\n"); log.flush()
BUDGET=540
try:
    while time.time()-t0<BUDGET:
        s=sc(); h=hd()
        if not s or h is None: time.sleep(0.1); continue
        logrow(h,s)
        e=ang_err(h,90)
        turn(max(-30,min(30,-e*2)))
        f0=s[0] if s[0]>=0 else 3.2
        f1=(s[1] if s[1]>=0 else 3.2); f15=(s[15] if s[15]>=0 else 3.2)
        front=min(f0,f1,f15)
        # junction detection: side opening far on beams 3..5 or 11..13
        side=[(s[k] if s[k]>=0 else 3.2) for k in range(2,14)]
        sp=4 if front>1.2 else (2 if front>0.5 else 1)
        if front<0.25:
            speed(0); print("BLOCKED at t=%.0f x=%.0f y=%.0f"%(time.time()-t0,x,y),flush=True); break
        speed(sp)
        time.sleep(0.15)
finally:
    stop(); log.close(); print("done t=%.0f x=%.1f y=%.1f"%(time.time()-t0,x,y),flush=True)
