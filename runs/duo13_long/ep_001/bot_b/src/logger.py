import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
while True:
    l=lidar()
    with open("/bot/src/telemetry.log","a") as f:
        f.write("%.1f h=%s d0=%s d5=%s d9=%s d6=%s d11=%s %s %s\n" % (time.time(), rd("d4"), rd("d0"), rd("d5"), rd("d9"), rd("d6"), rd("d11"), l, rd("d3")))
    time.sleep(0.5)
