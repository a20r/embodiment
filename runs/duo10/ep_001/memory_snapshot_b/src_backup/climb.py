import time, math
from rob import *
from nav import rot_to, havg, d5val
from man import fsweep
from pushlib import push, fb, athere
LOG=open('/tmp/nav.log','a')
def log(*a):
    LOG.write('CL '+' '.join(str(x) for x in a)+'\n'); LOG.flush()

cur=50.0
d5=d5val(4)
for it in range(200):
    st=status()
    if athere(st):
        stop(); log('ATGOAL',st)
        while True:
            tx('botB HERE=1 AT GOAL staying! climb d5!'); time.sleep(2); log(status())
    tx('botB climbing d5=%.3f'%d5)
    if d5>0.965:
        tx('botB ADJACENT (d5=%.3f). holding. lets both pause 15s then joint sweep: you lead slow.'%d5)
        log('adjacent hold', d5)
        time.sleep(10)
        d5=d5val(4)
        continue
    b,m=fsweep(cur, span=32, dur=6)
    if m<0.55:
        for dd in [90,-90,180]:
            b2,m2=fsweep(cur+dd, span=32, dur=6)
            if m2>0.55: b,m=b2,m2; break
        else:
            log('boxed'); cur=(cur+180)%360; continue
    rot_to(b, tol=3.5)
    why,mv=push(b, tmax=20)
    if mv<0.2 and why not in ('wall','HERE'):
        rot_to((b+180)%360, tol=3.5)
        why,mv=push(b, tmax=15, rev=True)
    d5n=d5val(4)
    log('it',it,'dir',round(b,1),why,'mv',round(mv,2),'d5',round(d5,3),'->',round(d5n,3))
    if why=='HERE': continue
    if mv>0.2:
        if d5n<d5-0.01: cur=(b+180)%360   # wrong way
        else: cur=b
    else:
        cur=(cur+90)%360
    d5=d5n
stop()
