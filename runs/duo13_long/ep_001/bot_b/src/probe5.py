import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
# spin in place slowly, log heading + d5 + d0 + d11
motors(2,-2)
t0=time.time()
while time.time()-t0<40:
    print("%.1f h=%s d0=%s d5=%s d11=%s d9=%s" % (time.time()-t0, rd("d4"), rd("d0"), rd("d5"), rd("d11"), rd("d9")))
    time.sleep(0.7)
motors(0,0)
