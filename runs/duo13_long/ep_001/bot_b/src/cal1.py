import time, sys
sys.path.insert(0,"/bot/src")
from robot import *

# spin test: d1=1,d7=-1 measured +~3deg/s. Try both polarity to confirm, record heading.
t0=time.time()
samples=[]
h0=heading()
motors(1,-1)
for i in range(20):
    time.sleep(0.25)
    samples.append((time.time()-t0, heading(), rd("d0"), rd("d5"), rd("d6"), rd("d9"), rd("d11")))
motors(0,0)
for s in samples: print(s)
print("dh=", (samples[-1][1]-h0))
