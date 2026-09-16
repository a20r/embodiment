import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
def avg(n=5):
    ls=[]
    for _ in range(n):
        l=lidar()
        if l: ls.append(l)
        time.sleep(0.1)
    return [sum(l[i] for l in ls)/len(ls) for i in range(16)] if ls else None
a=avg(); h0=heading(); e0=(float(rd("d6")),float(rd("d9")))
print("before h=%.1f" % h0, ["%.2f"%v for v in a])
motors(4,4); t0=time.time()
log=[]
while time.time()-t0<30:
    log.append((round(time.time()-t0,1), rd("d0"), rd("d5"), rd("d11"), rd("d3")))
    time.sleep(1.5)
motors(0,0)
b=avg(); h1=heading(); e1=(float(rd("d6")),float(rd("d9")))
print("after h=%.1f" % h1, ["%.2f"%v for v in b])
print("ticks d6=%+.0f d9=%+.0f" % (e1[0]-e0[0], e1[1]-e0[1]))
for x in log[::4]: print(x)
