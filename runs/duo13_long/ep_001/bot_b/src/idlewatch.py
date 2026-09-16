import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
motors(0,0)
t0=time.time()
while time.time()-t0<45:
    l=lidar()
    print("%.1f h=%.0f d0=%s d5=%s d11=%s l=%s" % (time.time()-t0, heading(), rd("d0"), rd("d5"), rd("d11"), l), flush=True)
    time.sleep(2.5)
