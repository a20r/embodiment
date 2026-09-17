import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
# spin in place, log heading+lidar
motors(3,-3)
t0=time.time()
log=[]
while time.time()-t0 < 50:
    h=heading(); l=lidar()
    log.append((round(time.time()-t0,2), h, l))
    time.sleep(0.15)
motors(0,0)
for t,h,l in log:
    print(t, h, l)
