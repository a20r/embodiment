import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
def lid():
    for _ in range(3):
        l=lidar()
        if l: return l
        time.sleep(0.04)
motors(0,0); time.sleep(0.5)
print("start b8=%.3f l=%s" % (lid()[8], [round(x,2) for x in lid()]), flush=True)
motors(10,10); t0=time.time()
n=0
while time.time()-t0<22:
    time.sleep(0.4); n+=1
    if n%3==0:
        l=lid()
        print("t=%.1f b8=%.3f b7=%.3f b9=%.3f d0=%s" % (time.time()-t0, l[8], l[7], l[9], rd("d0")), flush=True)
motors(0,0); time.sleep(0.5)
print("end b8=%.3f l=%s" % (lid()[8], [round(x,2) for x in lid()]), flush=True)
