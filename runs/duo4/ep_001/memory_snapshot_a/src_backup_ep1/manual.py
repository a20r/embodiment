import sys,time
sys.path.insert(0,'/bot/src')
from bot import IO,clean
io=IO()
prev=[None]
def sensors(dur=0.4):
    t=time.time()
    while time.time()-t<dur: io.poll(0.05)
    l=clean(io.lidar(),prev[0]); prev[0]=l
    return l,io.heading()
def wrap(a): return (a+180)%360-180
def turn_to(target,tol=3):
    t0=time.time()
    while time.time()-t0<12:
        io.poll(0.03); h=io.heading()
        if h is None: continue
        d=wrap(target-h)
        if abs(d)<tol: io.drive(0,0); return
        mag=min(70,max(8,abs(d)*1.2))
        io.drive(-mag if d>0 else mag,0)
        time.sleep(0.02)
    io.drive(0,0)
def show(l,h):
    print("h=%.1f"%h, "world:", {int((h+i*22.5)%360): round(v,2) for i,v in enumerate(l)}, flush=True)
