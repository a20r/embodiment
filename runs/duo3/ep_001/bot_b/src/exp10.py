from ctl import Ctl, angdiff
import time
b=Ctl(); time.sleep(0.3)
s=b.scan(); h0=b.heading()
b.wr(4,'-20'); b.wr(5,'-20')
time.sleep(1.5)
b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.2)
s2=b.scan(); h1=b.heading()
print('l=r=-20 1.5s beam0:', s[0],'->',s2[0], 'h', h0,'->',h1)
print('beam8:', s[8],'->',s2[8])
