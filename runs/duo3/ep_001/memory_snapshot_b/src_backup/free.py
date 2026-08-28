from ctl import Ctl, angdiff
from unstick import spin_to
import time
b=Ctl(); time.sleep(0.2)
def scan():
    return [x if x>0 else 0.05 for x in b.scan()]
# face most open dir
s=scan()
i=max(range(16),key=lambda i:s[i])
h=b.heading()
tgt=(h+i*22.5)%360
spin_to(b,tgt)
s=scan(); print('facing open, beam0=',s[0],'h',b.heading())
# drive forward, auto-detect sign
for sign in (-1,1):
    s0=scan()[0]
    b.wr(4,str(30*sign)); b.wr(5,str(30*sign)); time.sleep(0.6)
    b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.15)
    s1=scan()[0]
    print('sign',sign,'d beam0',round(s1-s0,3))
    if s1<s0-0.08:
        print('forward sign is',sign); break
