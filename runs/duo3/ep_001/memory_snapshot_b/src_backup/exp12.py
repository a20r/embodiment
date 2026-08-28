from ctl import Ctl
import time
b=Ctl(); time.sleep(0.3)
print('d7',b.p[7].latest(),'d0',b.p[0].latest())
s=b.scan(); print('scan',[round(x,2) for x in s]); print('h',b.heading())
b.wr(4,'15'); b.wr(5,'15'); time.sleep(1.0); b.wr(4,'0'); b.wr(5,'0')
time.sleep(0.3)
print('after backup d7',b.p[7].latest())
s=b.scan(); print('scan',[round(x,2) for x in s])
