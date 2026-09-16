import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
motors(0,0); time.sleep(1)
print("idle:", [rd("d5") for _ in range(5)], flush=True)
motors(2,-2)
t0=time.time()
while time.time()-t0<45:
    h=heading(); l=lidar()
    print("%.1f h=%.0f d5=%s d0=%s d11=%s b8=%s" % (time.time()-t0, h, rd("d5"), rd("d0"), rd("d11"), (l[8] if l else None)), flush=True)
    time.sleep(0.8)
motors(0,0)
