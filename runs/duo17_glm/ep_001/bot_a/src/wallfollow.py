import sys,time,math,json,statistics
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

TARGET=0.45   # right wall distance
BUDGET=3000
rss_hist=[]
try:
    while time.time()-t0<BUDGET:
        now=time.time()-t0
        h=hd()
        if h is None: time.sleep(0.05); continue
        update_pose(h)
        s=sc()
        if not s: s=None
        rss=read_rss()
        if rss is not None:
            rss_hist.append((now,rss))
            if len(rss_hist)>600: rss_hist.pop(0)
        if now-last_ping>25:
            last_ping=now
            send(f"A p=({x:.0f},{y:.0f}) h={h:.0f} rss={rss}")
        g=lambda k: s[k] if s[k]>=0 else 3.2
        front=min(g(15),g(0),g(1))
        right=min(g(13),g(14))
        e=0.0
        sp=3.0
        if front<0.55:
            # turn left until front opens (CCW: d7 negative)
            turn(-35)
            if front<0.3: speed(0.6)
            else: speed(1.2)
            time.sleep(0.15)
            continue
        # wall follow: aim right wall at TARGET using beams 13(=-67.5),14(=-45),15(=-22.5)
        r1=g(14)  # -45deg
        r2=g(15)  # -22.5deg
        # predicted distance ahead: project
        rd = r2*math.cos(math.radians(22.5)) if r2<2.8 else 999
        err = TARGET - rd if rd<900 else 0
        e = -err*2.0  # too far -> turn right (d7 positive decreases heading)
        # also avoid left wall
        left=g(2)
        if left<0.35: e-=1.5
        turn(max(-30,min(30,e)))
        speed(sp)
        
        if s: log.write(json.dumps({"t":round(now,1),"h":round(h,1),"x":round(x,1),"y":round(y,1),"rss":rss,"mode":"wf","s":[round(v,2) for v in s]})+"\n"); log.flush()
        else: log.write(json.dumps({"t":round(now,1),"h":round(h,1),"x":round(x,1),"y":round(y,1),"rss":rss,"mode":"wf"})+"\n"); log.flush()
        time.sleep(0.1)
finally:
    stop(); print("wallfollow done",flush=True)
