import time
from robot import Robot
r = Robot()
r.stop(); time.sleep(0.5)
print('h0', r.heading(), 'enc', r.enc())
r.rot(90)
print('h1', r.heading(), 'enc', r.enc())
time.sleep(0.5)
print('h2', r.heading())
