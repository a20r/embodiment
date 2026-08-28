from ctl import Ctl
import time
b=Ctl(); time.sleep(0.3)
for v in ['-3','-6','-10','-15']:
    s0=b.scan()[0]
    b.wr(4,v); b.wr(5,v); time.sleep(1.2); b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.2)
    s1=b.scan()[0]
    print('wheel',v,'moved',round(s0-s1,3))
for w in ['4','8']:
    h0=b.heading()
    b.wr(4,w); b.wr(5,str(-float(w))); time.sleep(1.2); b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.2)
    h1=b.heading()
    print('spin',w,'dh',round((h1-h0+180)%360-180,1))
