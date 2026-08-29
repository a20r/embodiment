import time, math
from lib import *
pts=[]
for step in range(8):
    motors(0,0); time.sleep(0.4)
    hs=[]; ls=[]
    for _ in range(3):
        h=heading(); l=lidar()
        if h is not None and l: hs.append(h); ls.append(l)
        time.sleep(0.1)
    h=sum(hs)/len(hs)
    l=[sorted(x[k] for x in ls)[1] for k in range(16)]
    print(f"h={h:.1f} l="+",".join(f"{x:.2f}" for x in l))
    # rotate ~... 22.5/2? we have 16 beams; rotating 11.25 deg fills between
    tgt=(h+11.25)%360
    for _ in range(200):
        hh=heading()
        if hh is None: continue
        err=(tgt-hh+180)%360-180
        if abs(err)<2: break
        v=max(min(err*0.8,15),-15); v=math.copysign(max(abs(v),4),v)
        motors(v,-v); time.sleep(0.04)
motors(0,0)
