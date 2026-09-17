import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, statistics, json
while True:
    h=read_float("d1")
    scans=[]
    t0=time.time()
    while time.time()-t0<8:
        scans.append(lidar()); time.sleep(0.25)
    for i in range(16):
        vals=[s[i] for s in scans if s[i]>0]
        if len(vals)<12: continue
        med=statistics.median(vals)
        dips=[v for v in vals if v<med-0.3]
        if len(dips)>=3:
            brg=(h+22.5*i)%360
            print(f"{time.time():.0f} MOVER beam{i} brg={brg:.0f} rmin={min(dips):.2f} med={med:.2f}",flush=True)
            write_port("d0", json.dumps(dict(who="A",msg=f"B: SEE YOU! You are at compass bearing {brg:.0f}, {min(dips):.1f}m from me. Come bearing {(brg+180)%360:.0f}. I am parked.")))
