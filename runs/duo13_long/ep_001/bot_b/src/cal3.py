import time, sys, statistics
sys.path.insert(0,"/bot/src")
from robot import *

def enc():
    return float(rd("d6") or 0), float(rd("d9") or 0)

hs=[heading() for _ in range(15)]
print("heading stats: mean=%.1f std=%.2f min=%.1f max=%.1f" % (statistics.mean(hs), statistics.pstdev(hs), min(hs), max(hs)))

def run(l,r,t):
    a=enc(); h0=heading(); motors(l,r); time.sleep(t); motors(0,0)
    b=enc(); h1=heading()
    time.sleep(0.3)
    print("cmd(%s,%s) %.1fs: d6=%+.0f d9=%+.0f dh=%+.1f" % (l,r,t,(b[0]-a[0]),(b[1]-a[1]),h1-h0))

run(0.5,0.5,2)
run(1,1,2)
run(3,3,2)
run(3,-3,2)
run(-3,3,2)
run(4,4,2)
run(0,4,2)
