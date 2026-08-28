from ctl import Ctl
import time
b=Ctl(); time.sleep(0.3)
for v in ['-17','-20','-25','-30']:
    s0=b.scan()[0]
    if s0<0.8:
        print('too close, stop'); break
    b.wr(4,v); b.wr(5,v); time.sleep(1.0); b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.25)
    s1=b.scan()[0]
    print('wheel',v,'dist',round(s0-s1,3))
