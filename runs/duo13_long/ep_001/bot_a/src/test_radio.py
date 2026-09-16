import time
from robot import Robot
r = Robot()
r.send('HELLO from A')
time.sleep(0.5)
for i in range(3):
    v = r.recv(1.0)
    print('recv:', v)
