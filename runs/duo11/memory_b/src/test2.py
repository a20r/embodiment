import time
from robot import Robot
r=Robot()
time.sleep(0.5)
print("hdg0",r.hdg)
r.motors(3,-3)
t0=time.time()
for i in range(10):
    time.sleep(0.3)
    print(f"t={time.time()-t0:.1f} hdg={r.hdg:.1f} b0={r.lidar[0]:.2f} b4={r.lidar[4]:.2f} b8={r.lidar[8]:.2f} b12={r.lidar[12]:.2f}")
r.stop()
