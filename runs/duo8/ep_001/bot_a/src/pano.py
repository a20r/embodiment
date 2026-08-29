import time, math
from lib import *
# back away from the corner first
t0=time.time()
while time.time()-t0<1.5:
    motors(-50,-50); time.sleep(0.05)
motors(0,0); time.sleep(0.3)
# panorama: rotate slowly, sample beam0 at headings
samples={}
last=None
t0=time.time()
while time.time()-t0<30:
    motors(10,-10)
    h=heading(); l=lidar()
    if h is not None and l:
        for k in range(16):
            ang=round((h - k*22.5)%360)
            if l[k]>0: samples.setdefault(ang//5*5,[]).append(l[k])
    if len(samples)>=72 and time.time()-t0>20: break
    time.sleep(0.03)
motors(0,0)
for a in sorted(samples):
    v=samples[a]
    print(a, round(sum(v)/len(v),2), len(v))
