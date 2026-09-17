import time
from robot import Robot
r=Robot()
time.sleep(0.3)
r.motors(-50,-50); time.sleep(1.5); r.stop()
print("b0",r.lidar[0],"d5",r.flags.get('d5'))
for i in range(5):
    r.send("hello? robot A here")
    time.sleep(1)
    msgs=r.get_rx()
    if msgs: print("RX:",msgs)
print("done listening")
