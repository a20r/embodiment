import sys, time
sys.path.insert(0,'/bot/src')
from robot import R
r=R()
while True:
    v=r.read(11,0.03)
    r.write(8,f"B2 PING t={time.time():.0f} d11={v}")
    time.sleep(1.0)
