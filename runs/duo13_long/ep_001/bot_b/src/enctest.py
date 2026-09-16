import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
def enc():
    a=rd("d6"); b=rd("d9")
    return a,b
print("idle:", enc(), enc(), flush=True)
motors(3,-3); time.sleep(2.5); motors(0,0); time.sleep(0.3)
print("after spin(3,-3):", enc(), "h=", heading(), flush=True)
motors(4,4); time.sleep(2); motors(0,0); time.sleep(0.3)
print("after fwd(4,4):", enc(), "h=", heading(), flush=True)
