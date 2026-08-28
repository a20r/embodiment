from drv import Bot
import time
b=Bot(); time.sleep(0.3)
b.wr(5,'0')
time.sleep(1)
h1=b.heading(); time.sleep(1); h2=b.heading()
print('stopped?', h1, h2)
# test d4 as velocity: forward slow, watch d6 and lidar beam0
s=b.scan(); print('scan0',s)
b.wr(4,'0.2')
t0=time.time()
while time.time()-t0<3:
    b.p[6].poll()
    if b.p[6].queue:
        print(round(time.time()-t0,2),'d6:',b.p[6].queue); b.p[6].queue.clear()
    time.sleep(0.05)
b.wr(4,'0')
print('scan1', b.scan())
print('d0', b.p[0].latest(), 'd7', b.p[7].latest())
