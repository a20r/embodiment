from ctl import Ctl
import time
b=Ctl(); time.sleep(0.2)
b.wr(4,'-25'); b.wr(5,'-25')
t0=time.time()
while time.time()-t0<4:
    s=b.scan(); b.p[7].poll(); b.p[2].poll()
    print(f'{time.time()-t0:4.1f}','|'.join(f'{x:4.2f}' for x in s[:2]+s[4:5]+s[8:9]+s[12:13]),'h',b.p[2].last,'d7',b.p[7].last)
    time.sleep(0.3)
b.wr(4,'0'); b.wr(5,'0')
