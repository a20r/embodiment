import sys,time,statistics
sys.path.insert(0,"/bot/src")
from robust import rline
vals=[]; t0=time.time()
while time.time()-t0<25:
    v=rline("d11",0.3)
    try: vals.append(float(v))
    except: pass
    time.sleep(0.2)
print(f"stationary: n={len(vals)} first={vals[0]:.3f} last={vals[-1]:.3f} slope={(vals[-1]-vals[0])/25:.5f}/s mean={statistics.mean(vals):.3f}")
