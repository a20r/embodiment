import time, sys, os
sys.path.insert(0,"/bot/src")
from robot import *
while True:
    wr("d8", "PING %d" % int(time.time()))
    r=rd("d10", timeout=0.8)
    if r:
        with open("/bot/src/rx.log","a") as f:
            f.write("%.1f %s\n" % (time.time(), r))
