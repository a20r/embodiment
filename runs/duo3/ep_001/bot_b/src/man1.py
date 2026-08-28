from ctl import Ctl, angdiff
from unstick import spin_to, sc
import time
b=Ctl(); time.sleep(0.2)
spin_to(b,45)
b.wr(4,'-20'); b.wr(5,'-20'); time.sleep(0.8); b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.2)
s=sc(b); print('after',[round(x,2) for x in s],'h',b.heading())
