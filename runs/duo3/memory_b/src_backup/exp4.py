from drv import Bot
import time
b=Bot(); time.sleep(0.3)
print('pre', b.heading())
b.wr(5,'90')
t0=time.time()
while time.time()-t0<4:
    b.p[2].poll()
    if b.p[2].queue:
        for s in b.p[2].queue: print(round(time.time()-t0,2), s)
        b.p[2].queue.clear()
    time.sleep(0.05)
