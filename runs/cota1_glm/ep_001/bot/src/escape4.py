import time
from drv import Driver, show
D=Driver()
D.start(40,60)
for i in range(4):
    time.sleep(0.7); show(f'fwd+40 steer+60 {i}')
D.set(40,20)
for i in range(3):
    time.sleep(0.7); show(f'fwd+40 steer+20 {i}')
D.set(20,0)
time.sleep(0.7); show('fwd+20 straight')
D.halt()
show('end')
