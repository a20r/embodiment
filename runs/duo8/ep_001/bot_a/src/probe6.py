import time
from lib import *
def xy(): return float(rline(7)), float(rline(8))
print("xy0",xy(),"h",heading())
t0=time.time()
while time.time()-t0<2:
    motors(50,50); time.sleep(0.05)
motors(0,0); time.sleep(0.2)
print("xy1",xy(),"h",heading())
t0=time.time()
while time.time()-t0<2:
    motors(10,-10); time.sleep(0.05)
motors(0,0); time.sleep(0.2)
print("xy2",xy(),"h",heading(),"(turned, should be same xy)")
