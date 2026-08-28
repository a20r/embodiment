from ctl import Ctl
import time
b=Ctl(); time.sleep(0.3)
# calibrate forward speed precisely: face open, drive l=r=-20, sample beam0
s=b.scan()
i=max(range(16), key=lambda i: s[i])
b.turn_by(((i*22.5+180)%360)-180)
s=b.scan(); d0=s[0]; print('start beam0',d0)
b.wr(4,'-20'); b.wr(5,'-20')
t0=time.time(); samples=[]
while time.time()-t0<2.0:
    sc=b.scan(); samples.append((round(time.time()-t0,2), sc[0]))
b.wr(4,'0'); b.wr(5,'0')
for t,v in samples: print(t,v)
# check d6/d0/d7 while moving
print('d6',b.p[6].latest(),'d0',b.p[0].latest(),'d7',b.p[7].latest())
