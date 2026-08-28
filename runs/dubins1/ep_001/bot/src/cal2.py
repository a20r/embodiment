import sys, time
sys.path.insert(0,'/memory/code')
from robot import Robot
import math
r = Robot()
time.sleep(1)
def hdg():
    vals=[]
    for _ in range(5):
        h=r.heading()
        if h is not None: vals.append(h)
        time.sleep(0.25)
    # circular mean
    if not vals: return None
    x=sum(math.cos(math.radians(v)) for v in vals)
    y=sum(math.sin(math.radians(v)) for v in vals)
    return round(math.degrees(math.atan2(y,x))%360,1)
for steer,thr,dur in [(30,5,3),(90,5,3),(180,5,3),(90,-5,3),(-90,5,3)]:
    h0=hdg()
    r.cmd(steer,thr); time.sleep(dur); r.stop(); time.sleep(0.5)
    h1=hdg()
    dh=((h1-h0+180)%360)-180
    print(f"steer={steer} thr={thr} dur={dur} dh={dh:.1f} rate={dh/dur:.2f} deg/s", flush=True)
