from ctl import Ctl
import time
b=Ctl(); time.sleep(0.2)
def snap():
    s=b.scan(); return '|'.join(f'{x:4.2f}' for x in (s[0],s[4],s[8],s[12]))
print('start',snap())
for cmd in [('100','100',1.0),('-100','-100',1.0),('80','-80',0.7),('-80','80',0.7),('-100','-100',2.0),('100','100',2.0)]:
    b.wr(4,cmd[0]); b.wr(5,cmd[1]); time.sleep(cmd[2]); b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.15)
    print(cmd, snap(), 'h', b.heading())
