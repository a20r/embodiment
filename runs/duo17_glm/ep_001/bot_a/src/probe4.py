import sys,time,math,statistics
sys.path.insert(0,"/bot/src")
from robot import speed,turn,stop
from nav import hd,odo
from robust import rline
def rss():
    v=rline("d11",0.2)
    try: return float(v)
    except: return None
def leg(dir_deg,secs=22):
    t0=time.time()
    while time.time()-t0<6:
        h=hd()
        if h is None: continue
        e=(dir_deg-h+540)%360-180
        turn(max(-35,min(35,-e*2)))
        time.sleep(0.05)
    turn(0)
    vals=[]; o0=odo(); t0=time.time()
    while time.time()-t0<secs:
        h=hd()
        if h is not None:
            e=(dir_deg-h+540)%360-180
            turn(max(-35,min(35,-e*2)))
        speed(3)
        v=rss()
        if v is not None: vals.append(v)
        time.sleep(0.1)
    speed(0); turn(0)
    o1=odo()
    return (statistics.mean(vals) if vals else None, abs(o1-o0) if o1 and o0 else 0)
res={}
for d in (0,90,180,270):
    m,dist=leg(d)
    res[d]=(m,dist)
    print(f"dir={d}: rss={m} dist={dist:.0f}",flush=True)
    time.sleep(2)
