from ctl import Ctl
import time
b=Ctl(); time.sleep(0.3)
for v in ['0.5','1','2','-1']:
    s=b.scan()
    b.wr(4,v); time.sleep(1.0); b.wr(4,'0'); time.sleep(0.2)
    s2=b.scan()
    print(v, round(s[0],3), round(s2[0],3), 'moved', round(s[0]-s2[0],3))
