import time
from robot import Robot
r = Robot()
r.stop(); time.sleep(0.5)
l0, r0 = r.enc(); h0 = r.heading()
print('start enc', l0, r0, 'h', h0, 'lid', r.lidar())
r.wheels(30, 30)
time.sleep(2.0)
r.stop(); time.sleep(0.5)
l1, r1 = r.enc(); h1 = r.heading()
print('after  enc', l1, r1, 'h', h1)
print('denc', l1-l0, r1-r0)
print('lid', r.lidar())
