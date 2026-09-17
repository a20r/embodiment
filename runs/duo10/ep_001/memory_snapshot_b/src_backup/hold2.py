import time
from rob import *
from nav import d5val
LOG=open('/tmp/hold.log','a')
def log(*a):
    LOG.write(' '.join(str(x) for x in a)+'\n'); LOG.flush()
last=0
while True:
    st=status()
    if 'here=1' in st or 'goal=1' in st:
        stop()
        while True:
            tx('botB HERE=1 AT GOAL! staying. come to me (climb d5).')
            log('ATGOAL', time.time(), status()); time.sleep(2)
    d5=d5val(2)
    if time.time()-last>5:
        tx('botB HOLDING for you at your-frame ~(6,4). d5=%.3f. climb d5 to me.'%d5)
        last=time.time()
        log(round(time.time(),1),'d5',round(d5,3),st)
    if d5>0.955:
        tx('botB: you are ADJACENT (d5=%.3f)! stop here; lets both sit. watching for goal flag.'%d5)
    time.sleep(1.5)
