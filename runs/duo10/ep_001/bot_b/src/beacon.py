import time
from rob import *
from nav import d5val
while True:
    try:
        d=d5val(3)
        st=rd('d6')
        tx('botB HOLDING d5=%.3f %s'%(d,st))
    except Exception: pass
    time.sleep(4)
