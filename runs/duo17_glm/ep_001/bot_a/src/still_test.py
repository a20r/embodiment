import sys,time,statistics
sys.path.insert(0,"/bot/src")
from robot import speed,turn,stop
from robust import rline
speed(0);turn(0)
vals=[];t0=time.time()
while time.time()-t0<170:
    v=rline("d11",0.2)
    try: vals.append((time.time()-t0,float(v)))
    except: pass
    time.sleep(0.3)
# print every 10s mean
import collections
b=collections.defaultdict(list)
for t,v in vals: b[int(t//10)*10].append(v)
for k in sorted(b): print(f"t={k:3d}s mean={statistics.mean(b[k]):.4f}",flush=True)
