from lib import *
import time
print("h0",heading(),"front",lidar()[0])
turn_by(180)
time.sleep(0.3)
L=lidar(); print("h1",heading(),"lidar",L)
# drive forward, calibrate
l0=ri('d9'); r0=ri('d6'); f0=L[0]
w('d1',20); w('d7',20)
t0=time.time()
while time.time()-t0<3:
    if bump(): print("BUMP"); break
    time.sleep(0.1)
stop(); time.sleep(0.3)
L=lidar()
print("ticks",ri('d9')-l0, ri('d6')-r0, "front", f0, "->", L[0])
print(L)
