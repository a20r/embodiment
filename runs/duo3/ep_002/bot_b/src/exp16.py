from ctl import Ctl
import time
b=Ctl(); time.sleep(0.3)
b.wr(4,'-20'); b.wr(5,'-20')
t0=time.time()
while time.time()-t0<3:
    s=b.scan(); h=b.p[2].last; b.p[7].poll(); d7=b.p[7].last
    print(round(time.time()-t0,2),'s0=%.2f s4=%.2f s8=%.2f s12=%.2f'%(s[0],s[4],s[8],s[12]),'h',h,'d7',d7)
b.wr(4,'0'); b.wr(5,'0')
