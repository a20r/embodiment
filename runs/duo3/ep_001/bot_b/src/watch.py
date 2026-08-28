from ctl import Ctl
import time
b=Ctl(); time.sleep(0.2)
t0=time.time()
while time.time()-t0<12:
    s=b.scan(); h=b.p[2].last
    print(round(time.time()-t0,1),' '.join(f'{x:4.2f}' for x in s),'h',h)
    time.sleep(1.2)
for m in b.radio_recv(): print('RX',m)
