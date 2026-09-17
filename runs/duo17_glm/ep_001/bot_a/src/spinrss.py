import sys,time
sys.path.insert(0,"/bot/src")
from robot import speed,turn,stop
from nav import hd
from robust import rline
vals=[]
turn(40)
t0=time.time()
while time.time()-t0<10:
    h=hd(); v=rline("d11",0.2)
    if h is not None and v:
        try: vals.append((h,float(v)))
        except: pass
    time.sleep(0.05)
stop(); turn(0)
# bin by heading
import collections,statistics
b=collections.defaultdict(list)
for h,v in vals: b[int(h//20)*20].append(v)
for k in sorted(b): print(f"h~{k:3d}: {statistics.mean(b[k]):.3f} n={len(b[k])}")
