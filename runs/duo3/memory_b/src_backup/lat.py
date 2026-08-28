from ctl import Ctl
import time
b=Ctl(); time.sleep(0.2)
s=b.scan()
print('front',s[0])
b.wr(4,'-60'); b.wr(5,'-60')  # backward to regain distance
time.sleep(1.5); b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.5)
print('front now',b.scan()[0])
b.wr(4,'60'); b.wr(5,'60')
t0=time.time(); prev=None
while time.time()-t0<3:
    b.p[1].poll()
    if b.p[1].queue:
        for line in b.p[1].queue:
            v=float(line.split(',')[0])
            if v!=prev:
                print(f'{time.time()-t0:5.2f} {v}')
                prev=v
        b.p[1].queue.clear()
    time.sleep(0.02)
b.wr(4,'0'); b.wr(5,'0')
