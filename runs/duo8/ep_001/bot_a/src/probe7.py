import time, math
from lib import *
def xy(): return float(rline(7)), float(rline(8))
def turn_to(target):
    for _ in range(300):
        h=heading()
        err=(target-h+180)%360-180
        if abs(err)<3: break
        v=max(min(err*0.8,25),-25)
        if abs(v)<4: v=4*(1 if v>0 else -1)
        motors(v,-v); time.sleep(0.05)
    motors(0,0)
for tgt in [0,90]:
    turn_to(tgt); time.sleep(0.3)
    x0,y0=xy()
    t0=time.time()
    while time.time()-t0<1.5:
        motors(40,40); time.sleep(0.05)
    motors(0,0); time.sleep(0.3)
    x1,y1=xy()
    print(f"tgt={tgt} h={heading():.1f} disp=({x1-x0:.0f},{y1-y0:.0f}) ang={math.degrees(math.atan2(y1-y0,x1-x0)):.1f}")
    # back up
    t0=time.time()
    while time.time()-t0<1.5:
        motors(-40,-40); time.sleep(0.05)
    motors(0,0)
