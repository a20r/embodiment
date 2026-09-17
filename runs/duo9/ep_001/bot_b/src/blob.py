import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, math
h=read_float("d1")
scans=[]
for _ in range(60):
    l=lidar()
    scans.append(l)
    time.sleep(0.25)
n=len(scans)
import statistics
print("heading",h)
for i in range(16):
    vals=[s[i] for s in scans if s[i]>0]
    if len(vals)<10: continue
    med=statistics.median(vals); mn=min(vals); mx=max(vals); sd=statistics.pstdev(vals)
    bearing=(h+22.5*i)%360
    flag="**MOVER**" if (mx-mn)>0.4 and sd>0.08 else ""
    print(f"beam{i:2d} bearing{bearing:6.1f} med={med:.2f} min={mn:.2f} max={mx:.2f} sd={sd:.3f} {flag}")
