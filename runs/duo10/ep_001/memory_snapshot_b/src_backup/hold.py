import time
from rob import *
from nav import d5val
from pushlib import athere
LOG=open('/tmp/nav.log','a')
stop()
while True:
    st=status()
    d=d5val(3)
    if athere(st):
        while True:
            tx('botB HERE=1 AT GOAL! staying!'); time.sleep(2)
            LOG.write('HD ATGOAL '+status()+'\n'); LOG.flush()
    tx('botB HOLDING still d5=%.3f %s'%(d,st))
    LOG.write('HD d5=%.3f %s\n'%(d,st)); LOG.flush()
    time.sleep(5)
