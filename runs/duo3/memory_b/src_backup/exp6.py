from ctl import Ctl
import time
b=Ctl(); time.sleep(0.3)
print('h0',b.heading())
print('turn ok', b.turn_by(90))
print('h1',b.heading())
s=b.scan(); print('scan',s)
# now beam0 should be old beam4 (~1.4)
b.wr(4,'0.3')
time.sleep(1.0)
b.wr(4,'0')
time.sleep(0.3)
s2=b.scan(); print('scan2',s2)
print('delta fwd', s[0]-s2[0] if s and s2 else None)
