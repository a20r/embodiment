import time
from drv import Driver, show
D=Driver()
D.start(-50,0)
for i in range(2):
    time.sleep(0.6); show(f'back {i}')
D.set(40,80)
for i in range(4):
    time.sleep(0.6); show(f'fwd hard left {i}')
D.set(40,30)
for i in range(4):
    time.sleep(0.6); show(f'fwd left {i}')
D.halt()
show('end')
