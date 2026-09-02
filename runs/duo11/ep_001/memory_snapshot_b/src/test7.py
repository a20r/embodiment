import time
from robot import Robot
r=Robot()
time.sleep(0.3)
def snap():
    time.sleep(0.4)
    return r.hdg, r.lidar[:]
h1,l1=snap()
print("hdg %.1f"%h1, ["%.2f"%x for x in l1])
r.motors(10,-10); time.sleep(1.2); r.stop()  # heading increases ~45deg?
h2,l2=snap()
print("hdg %.1f"%h2, ["%.2f"%x for x in l2])
