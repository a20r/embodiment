import time
from drv import Driver, show
D=Driver()
D.start(-60,50)
for i in range(6):
    time.sleep(0.7); show(f'rev-60 steer+50 {i}')
D.halt()
show('reassess')
D.start(40,-40)
for i in range(3):
    time.sleep(0.7); show(f'fwd+40 steer-40 {i}')
D.halt()
show('end')
