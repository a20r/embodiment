import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time
t0=time.time()
while time.time()-t0<180:
    motors(50,50); time.sleep(0.5)
    motors(-50,-50); time.sleep(0.5)
motors(0,0)
