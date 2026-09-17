import time
from robot import Robot
r=Robot()
time.sleep(0.5)
r.motors(20,-20)
t0=time.time()
for i in range(12):
    time.sleep(0.5)
    print(f"t={time.time()-t0:.1f} hdg={r.hdg:.1f} b0={r.lidar[0]:.2f} b8={r.lidar[8]:.2f}")
r.stop()
