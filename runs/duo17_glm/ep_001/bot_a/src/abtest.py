import sys,time,math
sys.path.insert(0,"/bot/src")
from robot import speed,turn,stop
from nav import hd,sc,odo,ang_err
from robust import rline
def phase(name,target_h,secs):
    vals=[]; o0=odo(); t0=time.time()
    while time.time()-t0<secs:
        h=hd()
        if h is None: continue
        e=ang_err(h,target_h)
        turn(max(-30,min(30,-e*2)))
        speed(3)
        v=rline("d11",0.2)
        try: vals.append(float(v))
        except: pass
        time.sleep(0.1)
    stop()
    o1=odo()
    dist=abs(o1-o0) if o1 and o0 else -1
    if len(vals)>4:
        print(f"{name}: dist={dist:.0f} d11 {vals[0]:.3f}->{vals[-1]:.3f} slope={(vals[-1]-vals[0])/secs*60:.4f}/min")
    time.sleep(2)
phase("SOUTH",270,30)
phase("NORTH",90,30)
phase("SOUTH2",270,30)
