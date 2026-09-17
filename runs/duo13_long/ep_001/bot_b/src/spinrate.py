import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
for rate in (3,4,5,6):
    h0=heading(); motors(rate,-rate); time.sleep(3); motors(0,0); time.sleep(0.3)
    print("rate",rate,"dh",round(heading()-h0,1), flush=True)
    time.sleep(0.5)
