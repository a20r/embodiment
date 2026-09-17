import sys,time
sys.path.insert(0,"/bot/src")
from robot import speed
from robust import rline
import statistics
def run(label, secs, sp=0):
    speed(sp)
    vals=[]
    t0=time.time()
    while time.time()-t0<secs:
        v=rline("d11",0.3)
        if v:
            try: vals.append(float(v))
            except: pass
        time.sleep(0.3)
    speed(0)
    if len(vals)>2:
        slope=(vals[-1]-vals[0])/(vals[-1:][0]-vals[0]+1e-9) if False else (vals[-1]-vals[0])/secs
        print(f"{label}: n={len(vals)} first={vals[0]:.3f} last={vals[-1]:.3f} slope={slope:.5f}/s mean={statistics.mean(vals):.3f}")
run("stationary", 30, 0)
run("driving fwd sp3", 40, 3)
run("stationary2", 30, 0)
