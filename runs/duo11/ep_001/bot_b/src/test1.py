import time
from robot import Robot
r=Robot()
time.sleep(0.5)
print("start hdg",r.hdg,"lidar",r.lidar)
r.motors(5,5)
for i in range(6):
    time.sleep(0.5)
    print(f"{r.hdg:.1f}", ["%.2f"%x for x in r.lidar])
r.stop()
