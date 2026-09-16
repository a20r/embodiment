import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
motors(3,-3)
t0=time.time()
while time.time()-t0 < 50:
    h=heading(); l=lidar()
    print(round(time.time()-t0,2), h, l)
    time.sleep(0.15)
motors(0,0)
