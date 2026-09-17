import sys,time,statistics
sys.path.insert(0,"/bot/src")
from robot import speed
from robust import rline
vals=[]; t0=time.time(); speed(3)
while time.time()-t0<35:
    v=rline("d11",0.3)
    try: vals.append(float(v))
    except: pass
    time.sleep(0.2)
speed(0)
print(f"driving: n={len(vals)} first={vals[0]:.3f} last={vals[-1]:.3f} slope={(vals[-1]-vals[0])/35:.5f}/s mean={statistics.mean(vals):.3f}")
