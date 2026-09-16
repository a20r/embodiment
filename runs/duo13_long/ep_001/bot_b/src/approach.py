import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
e0=(float(rd("d6")),float(rd("d9")))
motors(2,2)
t0=time.time()
while time.time()-t0 < 20:
    l=lidar()
    if l and l[8]>0 and l[8]<0.22:
        print("close! beam8=%.3f t=%.1f" % (l[8], time.time()-t0)); break
    if int((time.time()-t0)*5)%5==0:
        l2=lidar()
        if l2: print("t=%.1f beam8=%s enc=%s,%s d0=%s d5=%s" % (time.time()-t0, l2[8], rd("d6"), rd("d9"), rd("d0"), rd("d5")))
    time.sleep(0.2)
motors(0,0)
e1=(float(rd("d6")),float(rd("d9")))
print("ticks", e1[0]-e0[0], e1[1]-e0[1])
print("final lidar", lidar())
