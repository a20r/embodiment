import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
motors(0,0); time.sleep(1)
t0=time.time()
while time.time()-t0<50:
    print("t=%.0f d11=%s d5=%s d3=%s d0=%s" % (time.time()-t0, rd("d11"), rd("d5"), rd("d3"), rd("d0")), flush=True)
    time.sleep(2)
