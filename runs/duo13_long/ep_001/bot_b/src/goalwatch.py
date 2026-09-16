import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
while True:
    s=rd("d3",timeout=0.4)
    if s and ("goal=1" in s or "here=1" in s):
        l=lidar(); h=heading()
        with open("/bot/src/GOALALERT.txt","a") as f:
            f.write("%.1f %s h=%s l=%s\n"%(time.time(),s,h,l))
    time.sleep(0.5)
