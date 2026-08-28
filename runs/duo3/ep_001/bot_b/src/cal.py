from ctl import Ctl
import time
b=Ctl(); time.sleep(0.2)
def s0():
    s=b.scan(); return s[0]
for w,dur in [('30',1.0),('60',1.0),('100',1.0)]:
    a=s0()
    if a<1.0: print('too close, abort'); break
    b.wr(4,w); b.wr(5,w); time.sleep(dur); b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.2)
    c=s0(); b.p[7].poll()
    print(w, 'moved', round(a-c,3), 'm in', dur,'s d7',b.p[7].last,'h',b.heading())
