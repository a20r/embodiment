import time, math
from rob import *
from nav import rot_to, havg, d5val
from man import fsweep
from pushlib import push, fb, athere
LOG=open('/tmp/nav.log','a')
def log(*a):
    LOG.write('FL '+' '.join(str(x) for x in a)+'\n'); LOG.flush()
DIRS=[30.0,225.0]
last_tx=0
while True:
    st=status()
    d5=d5val(3)
    if athere(st):
        stop()
        while True:
            tx('botB HERE=1 AT GOAL, staying put! come stand with me, climb d5.'); time.sleep(2)
            LOG.write('FL ATGOAL '+status()+'\n'); LOG.flush()
    if time.time()-last_tx>6:
        tx('botB d5=%.3f %s'%(d5,st)); last_tx=time.time()
    if d5>0.90:
        time.sleep(1); continue
    # chase: try my two corridor directions, pick one that raises d5
    log('chasing d5',round(d5,3))
    best=None
    for base in DIRS:
        b,m=fsweep(base, span=30, dur=5)
        if m<0.5: continue
        rot_to(b,tol=4)
        why,mv=push(b, tmax=12)
        d5n=d5val(3)
        log('probe dir',round(b,1),why,round(mv,2),'d5',round(d5n,3))
        if athere(status()): break
        if d5n>d5+0.01:
            # keep going this way while d5 rises
            while d5n>d5+0.005 and d5n<0.93:
                d5=d5n
                why,mv=push(b, tmax=15)
                d5n=d5val(3)
                log('chase leg',why,round(mv,2),'d5',round(d5n,3))
                if mv<0.15: break
            break
        elif mv>0.2:
            # went wrong way; go back
            bb=(b+180)%360
            b2,m2=fsweep(bb, span=25, dur=5)
            rot_to(b2,tol=4); push(b2, tmax=12)
