import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
def enc(): return float(rd("d6") or 0), float(rd("d9") or 0)
for cmd in (5, 8, 10, 20):
    a=enc(); motors(cmd,cmd); time.sleep(2); motors(0,0); b=enc()
    print("cmd",cmd,"enc d6=%+.0f d9=%+.0f rate=%.1f" % (b[0]-a[0], b[1]-a[1], (b[0]-a[0])/2))
time.sleep(0.5)
h0=heading(); motors(3,3); time.sleep(8); motors(0,0)
print("after fwd8s: h %.1f->%.1f, lidar:" % (h0, heading()), lidar())
