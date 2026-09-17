import time
from robot import Robot
r=Robot()
time.sleep(0.5)
# turn to hdg ~0
def turn_to(r, target, tol=4):
    while True:
        h=r.hdg
        err=(target-h+180)%360-180
        if abs(err)<tol: break
        s=max(3,min(20,abs(err)*0.5))
        if err>0: r.motors(s,-s)
        else: r.motors(-s,s)
        time.sleep(0.1)
    r.stop()
turn_to(r,0)
time.sleep(0.5)
print("hdg",r.hdg,"b0",r.lidar[0])
d0=r.lidar[0]
r.motors(20,20)
t0=time.time()
for i in range(8):
    time.sleep(0.5)
    print(f"t={time.time()-t0:.1f} hdg={r.hdg:.1f} b0={r.lidar[0]:.2f}")
    if r.lidar[0]<0.3 and r.lidar[0]>0: break
r.stop()
