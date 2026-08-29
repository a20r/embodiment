import time
from lib import *
for i in range(200):
    motors(8,-8); time.sleep(0.05)
    l=lidar()
    if l[0]>=max(l)-0.1 and l[0]>1.5: break
motors(0,0); time.sleep(0.2)
l0=lidar(); print("aligned h",heading(),"l",l0)
t0=time.time()
while time.time()-t0<3:
    motors(30,30); time.sleep(0.05)
motors(0,0); time.sleep(0.2)
l1=lidar(); print("after h",heading(),"l",l1)
print("beam0 change:",l1[0]-l0[0])
