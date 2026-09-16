import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
def snap(tag):
    time.sleep(0.3)
    ls=[lidar() for _ in range(4)]
    ls=[l for l in ls if l]
    avg=[sum(l[i] for l in ls)/len(ls) for i in range(16)]
    print(tag, ["%.2f"%v for v in avg], "enc=%s,%s h=%s d0=%s d5=%s d11=%s" % (rd("d6"),rd("d9"),rd("d4"),rd("d0"),rd("d5"),rd("d11")))
snap("t0")
# turn right ~67.5 deg: at ~5.5deg/s need ~12s
motors(3,-3); time.sleep(12.5); motors(0,0)
snap("turned right 67")
e0=(float(rd("d6")),float(rd("d9")))
motors(3,3); time.sleep(5); motors(0,0)
e1=(float(rd("d6")),float(rd("d9")))
snap("fwd5s")
print("enc delta", e1[0]-e0[0], e1[1]-e0[1])
motors(3,3); time.sleep(5); motors(0,0)
snap("fwd5s more")
