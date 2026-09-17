import time
from drv import Driver, show
D=Driver()
D.start(-60,0)
for i in range(3):
    time.sleep(0.8); show(f'rev-60 {i}')
D.set(60,0)
for i in range(3):
    time.sleep(0.8); show(f'fwd+60 {i}')
D.set(-80,50)
for i in range(3):
    time.sleep(0.8); show(f'rev-80 steer+50 {i}')
D.halt()
show('end')
