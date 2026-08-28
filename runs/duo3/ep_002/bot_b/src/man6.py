from ctl import Ctl
import time
b=Ctl(); time.sleep(0.2)
def snap():
    s=b.scan(); return s,[round(x,2) for x in (s[0],s[4],s[8],s[12])]
for cmd in [('30','30',1.0),('-30','-30',1.0),('60','60',0.6),('-60','-60',0.6)]:
    s0,p0=snap()
    b.wr(4,cmd[0]); b.wr(5,cmd[1]); time.sleep(cmd[2]); b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.15)
    s1,p1=snap()
    b.p[7].poll()
    print(cmd,'before',p0,'after',p1,'d7',b.p[7].last,'h',b.heading())
