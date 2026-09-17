import sys, time
sys.path.insert(0,"/bot/src")
from robot import *
log=[]
turn(45); speed(0)
t0=time.time()
while time.time()-t0<8:
    log.append((heading(), battery(), read("d5"), read("d6"), read("d0")))
    time.sleep(0.2)
turn(0)
for h,b,d5,d6,d0 in log:
    print(f"h={h:7.1f} d11={b} d5={d5} d6={d6} d0={d0}")
