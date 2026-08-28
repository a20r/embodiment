from ctl import Ctl
from unstick import spin_to, sc
import time
b=Ctl(); time.sleep(0.2)
b.radio_send('hello? anyone there? I see something near me')
spin_to(b,0)
s=sc(b); print('E0',[round(x,2) for x in s])
b.wr(4,'-20'); b.wr(5,'-20'); time.sleep(1.2); b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.15)
s=sc(b); print('E1',[round(x,2) for x in s],'h',b.heading())
print('rx',b.radio_recv())
b.p[0].poll(); b.p[7].poll(); b.p[9].poll()
print('d0',b.p[0].last,'d7',b.p[7].last,'d9',b.p[9].last)
