import time
from robot import Robot
r=Robot()
time.sleep(0.5)
print("hdg",r.hdg,"b0",r.lidar[0])
r.motors(100,100)
t0=time.time()
for i in range(8):
    time.sleep(0.5)
    print(f"t={time.time()-t0:.1f} hdg={r.hdg:.1f} b0={r.lidar[0]:.2f} d0={r.flags.get('d0')} d5={r.flags.get('d5')}")
r.stop()
