from ctl import Ctl
from unstick import spin_to, sc
import time
b=Ctl(); time.sleep(0.2)
spin_to(b,0)
b.wr(4,'-18'); b.wr(5,'-18'); time.sleep(0.7); b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.2)
s=sc(b); print('afterE',[round(x,2) for x in s],'h',b.heading())
spin_to(b,90)
s=sc(b); print('faceN',[round(x,2) for x in s],'h',b.heading())
