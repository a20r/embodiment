import sys, time
sys.path.insert(0,'/memory/code')
from robot import Robot
r=Robot('/bot/src/sensors.log'); time.sleep(1)
r.stop()
t0=time.time()
last=None
while time.time()-t0<75:
    v=r.get(6); g=r.get(0)
    if v!=last:
        print(f"t={time.time()-t0:.1f} d6={v} d0={g}", flush=True)
        last=v
    time.sleep(0.5)
print("end", r.get(6), r.get(0), flush=True)
