import sys, time
sys.path.insert(0,'/bot/src')
from nav import *

# turn to opening heading then drive forward until blocked
target=155.0
turn_to(target, tol=4, timeout=10)
print('after turn h', heading())
t0=time.time()
motors(30,30)
while time.time()-t0<8:
    l=lidar()
    f=min(x for x in l[15:16]+l[0:3] if x>0)
    if f<0.32:
        stop(); print('blocked, front min %.2f'%f); break
    time.sleep(0.15)
stop()
print('h',heading(),'lidar',[round(x,2) for x in lidar()],'status',status())
