import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
def snap(tag):
    time.sleep(0.3)
    ls=[lidar() for _ in range(4)]
    ls=[l for l in ls if l]
    avg=[sum(l[i] for l in ls)/len(ls) for i in range(16)]
    print(tag, ["%.2f"%v for v in avg], "enc=%s,%s h=%s" % (rd("d6"),rd("d9"),rd("d4")))
snap("before")
motors(-2,-2); time.sleep(2); motors(0,0)
snap("after back 2s")
motors(2,2); time.sleep(2); motors(0,0)
snap("after fwd 2s")
motors(3,-3); time.sleep(3); motors(0,0)
snap("after cw spin 3s")
