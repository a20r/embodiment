import time, sys, os, math
sys.path.insert(0,"/bot/src")
from robot import *
def havg(n=5):
    vals=[heading() for _ in range(n)]
    s=sum(math.sin(math.radians(v)) for v in vals)/n
    c=sum(math.cos(math.radians(v)) for v in vals)/n
    return math.degrees(math.atan2(s,c))%360
def lid():
    for _ in range(3):
        l=lidar()
        if l: return l
        time.sleep(0.04)
F="/bot/src/GOALALERT.txt"
while True:
    s=rd("d3",timeout=0.4)
    if s and ("goal=1" in s or "here=1" in s):
        with open(F,"a") as f: f.write("%.1f TRIGGER %s h=%s\n"%(time.time(),s,havg()))
        os.system("/bot/src/killhelper.sh explore6.py")
        # bearing scan: rotate slowly, record goal flag vs heading
        res=[]
        motors(2,-2)
        t0=time.time()
        while time.time()-t0<50:
            h=havg(3); s2=rd("d3",timeout=0.2) or ""
            l=lid()
            res.append((round(h,1), 1 if "goal=1" in s2 else 0, 1 if "here=1" in s2 else 0, l))
            time.sleep(0.5)
        motors(0,0)
        with open("/bot/src/GOALSCAN.txt","w") as f:
            for r in res: f.write("%s\n"%(r,))
        # stop here; wait for manual control
        while True:
            time.sleep(5)
            with open(F,"a") as f:
                f.write("%.1f STILL %s %s\n"%(time.time(),rd("d3"),havg(3)))
    time.sleep(0.12)
