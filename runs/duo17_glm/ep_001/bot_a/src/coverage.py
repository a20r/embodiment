import sys,time,math,json
sys.path.insert(0,"/bot/src")
from robot import speed,turn,stop,send
from nav import hd,sc,odo
from robust import rline

t0=time.time()
log=open("/memory/track.log","a")
x=y=0.0; prev_o=None; prev_h=None
last_ping=-99
def read_rss():
    v=rline("d11",0.2)
    try: return float(v)
    except: return None
def update_pose(h):
    global x,y,prev_o,prev_h
    o=odo()
    if o is not None and prev_o is not None:
        dd=o-prev_o; hh=h if prev_h is None else (h+prev_h)/2
        x+=dd*math.cos(math.radians(hh)); y+=dd*math.sin(math.radians(hh))
    if o is not None: prev_o=o
    prev_h=h
TARGET=0.5
BUDGET=2000
try:
    while time.time()-t0<BUDGET:
        now=time.time()-t0
        h=hd()
        if h is None: time.sleep(0.05); continue
        update_pose(h)
        s=sc()
        if not s: time.sleep(0.05); continue
        rss=read_rss()
        if now-last_ping>20:
            last_ping=now
            send(f"A p=({x:.0f},{y:.0f}) rss={rss} NEED CONTACT")
        g=lambda k: s[k] if s[k]>=0 else 3.2
        front=min(g(15),g(0),g(1))
        if front<0.5:
            turn(-35)
            speed(0.8 if front<0.3 else 1.3)
            time.sleep(0.12)
            continue
        rd=g(15)*math.cos(math.radians(22.5))
        err=TARGET-rd if rd<2.8 else 0
        e=-err*2.2
        left=g(2)
        if left<0.4: e-=1.5
        turn(max(-30,min(30,e)))
        sp=2.2
        speed(sp)
        log.write(json.dumps({"t":round(now,1),"h":round(h,1),"x":round(x,1),"y":round(y,1),"rss":rss,"mode":"cov","s":[round(v,2) for v in s]})+"\n"); log.flush()
        time.sleep(0.08)
finally:
    stop(); print("coverage done",flush=True)
