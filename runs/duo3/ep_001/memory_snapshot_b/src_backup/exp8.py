from ctl import Ctl
import time
b=Ctl(); time.sleep(0.3)
h0=b.heading(); s0=b.scan()
b.wr(4,'45'); time.sleep(1.0); b.wr(4,'0')
h1=b.heading(); s1=b.scan()
print('d4=45: h',h0,'->',h1,' beam0',s0[0],'->',s1[0])
# both
h0=b.heading(); s0=b.scan()
b.wr(4,'0.3'); b.wr(5,'0.3'); time.sleep(1.0); b.wr(4,'0'); b.wr(5,'0')
h1=b.heading(); s1=b.scan()
print('both 0.3: h',h0,'->',h1,' beam0',s0[0],'->',s1[0])
