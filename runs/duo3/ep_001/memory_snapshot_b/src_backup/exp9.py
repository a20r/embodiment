from ctl import Ctl, angdiff
import time
b=Ctl(); time.sleep(0.3)
# spin in place calibration
h0=b.heading()
b.wr(4,'45'); b.wr(5,'-45')
time.sleep(2.0)
b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.2)
h1=b.heading()
print('spin l=45 r=-45 2s:', h0,'->',h1, 'delta', angdiff(h1,h0))
# face most open direction
s=b.scan()
i=max(range(16), key=lambda i: s[i])
print('open beam',i,s[i])
b.turn_by(i*22.5)
s=b.scan(); print('beam0 now', s[0])
# drive straight calibration
t0=time.time()
b.wr(4,'20'); b.wr(5,'20')
time.sleep(1.5)
b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.2)
s2=b.scan()
print('fwd 20/20 1.5s beam0:', s[0],'->',s2[0], 'dist', round(s[0]-s2[0],3))
