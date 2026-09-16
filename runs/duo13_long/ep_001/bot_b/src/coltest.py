import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
h0=heading(); print("h0", h0, lidar())
# rotate LEFT by 67.5 deg so beam3 direction becomes forward: spin(-3,3) decreases heading
motors(-3,3); t0=time.time()
while time.time()-t0 < 12.5:
    time.sleep(0.5)
motors(0,0); time.sleep(0.5)
h1=heading(); l=lidar(); print("after turn h", h1, l)
e0=(float(rd("d6")),float(rd("d9")))
motors(2,2)
for i in range(40):
    time.sleep(0.3)
    l=lidar()
    b8 = l[8] if l else None
    if i%3==0: print("t=%.1f b8=%s b7=%s b9=%s enc=%s,%s d0=%s d5=%s" % (i*0.3, b8, l[7] if l else '?', l[9] if l else '?', rd("d6"), rd("d9"), rd("d0"), rd("d5")))
motors(0,0)
e1=(float(rd("d6")),float(rd("d9")))
print("ticks", e1[0]-e0[0], e1[1]-e0[1], "h", heading())
print("final", lidar())
