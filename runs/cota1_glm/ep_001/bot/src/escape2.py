import time
from drv import Driver, show
D=Driver()
show('start')
D.start(20,-100)
for i in range(4):
    time.sleep(0.8); show(f'fwd d7=-100 {i}')
D.halt()
show('stopped')
