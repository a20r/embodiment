import time, math
from rob import *
from nav import rot_to, havg, d5val
from man import fsweep
from pushlib import push, fb, athere
LOG=open('/tmp/nav.log','a')
def log(*a):
    LOG.write('SW '+' '.join(str(x) for x in a)+'\n'); LOG.flush()

pos=[0.0,0.0]
def announce():
    st=status()
    tx('botB sweep pos=(%.1f,%.1f) d5=%.3f %s'%(pos[0],pos[1],d5val(2),st))
    return st

def goal_watch():
    st=status()
    if athere(st):
        stop(); log('ATGOAL',st)
        while True:
            tx('botB HERE=1 AT GOAL pos=(%.1f,%.1f) staying! climb d5 to me!'%(pos[0],pos[1]))
            time.sleep(2); log(status())
    return st

# sweep pattern: follow corridors, pause each leg, prefer unvisited via simple zigzag
cur=232.0
for it in range(300):
    goal_watch()
    announce()
    b=None
    for dd in [-90,0,90,180]:
        b2,m2=fsweep(cur+dd, span=32, dur=6)
        if m2>0.55: b,m=b2,m2; break
    if b is None:
        log('boxed at',round(pos[0],1),round(pos[1],1)); cur=(cur+180)%360; continue
    rot_to(b, tol=3.5)
    why,mv=push(b, tmax=22)
    if mv<0.2 and why not in ('wall','HERE'):
        rot_to((b+180)%360, tol=3.5)
        why,mv=push(b, tmax=20, rev=True)
        log('rev try',why,round(mv,2))
    a=math.radians(b)
    pos[0]+=mv*math.sin(a); pos[1]+=mv*math.cos(a)
    d5c=d5val(3)
    log('it',it,'dir',round(b,1),why,'moved',round(mv,2),'pos',round(pos[0],2),round(pos[1],2),'d5',round(d5c,3))
    goal_watch()
    if d5c>0.962:
        stop(); log('ADJACENT hold at',round(pos[0],2),round(pos[1],2))
        tx('botB ADJACENT d5=%.3f! I hold here. come stand next to me; then lead sweep.'%d5c)
        t0=time.time()
        while d5val(3)>0.9 and time.time()-t0<120:
            goal_watch(); time.sleep(3)
    time.sleep(3)  # pause: simultaneity chance + goal latch
    if mv>0.25: cur=b
    else: cur=(b+180)%360
stop()
