import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
def enc():
    a=rd("d6"); b=rd("d9")
    return (float(a) if a is not None else 0.0, float(b) if b is not None else 0.0)
def spin(deg, rate=3):
    # deg>0 = CW (heading +): motors(rate,-rate)
    t = abs(deg)/ (rate*1.15)   # ~5.7 deg/s at rate 3? measure: earlier (3,-3)~5.5deg/s
    if deg>0: motors(rate,-rate)
    else: motors(-rate,rate)
    time.sleep(t); motors(0,0); time.sleep(0.4)
# step-scan: read beam8 at current heading, then rotate 22.5 deg CW, repeat 16 times
best=None
for k in range(16):
    time.sleep(0.4)
    l=[lidar() for _ in range(3)]
    l=[x for x in l if x]
    b8=sum(x[8] for x in l)/len(l)
    h=heading()
    print("step %d h=%.0f beam8=%.2f" % (k,h,b8))
    if 0.6<b8<1.25 and (best is None or b8<best[1]): best=(k,b8)
    spin(22.5)
print("best:",best)
if best:
    # rotate back to best heading: k steps CCW
    spin(-(16-best[0])*22.5)
    time.sleep(0.5)
    e0=enc(); h0=heading()
    print("approach from h=%.0f d0=%.2f" % (heading(), best[1]))
    motors(2,2)
    t0=time.time(); samples=[]
    while time.time()-t0<18:
        time.sleep(0.4)
        l=lidar()
        if l: samples.append((time.time()-t0, l[8], enc(), heading()))
        if l and l[8]<0.3: break
    motors(0,0)
    for s in samples[::3]: print("t=%.1f b8=%.2f enc=%s h=%.0f" % (s[0], s[1], s[2], s[3]))
    e1=enc()
    print("ticks L=%+d R=%+d" % (e1[1]-e0[1], e1[0]-e0[0]))
