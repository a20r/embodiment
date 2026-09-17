import sys,time,random
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== gowest ===')
AXES=[5,95,185,275]
while r.h is None or r.rays is None: r.update(); time.sleep(0.05)
goal_seen=False; last_tx=0
def poll():
    global goal_seen,last_tx
    r.update()
    for m in r.msgs: L('RX:',m)
    r.msgs[:]=[]
    for e in r.events:
        if 'goal=1' in e: goal_seen=True; L('EV:',e)
    r.events[:]=[]
    if time.time()-last_tx>2:
        r.tx.write('A moving to you. B keep climbing d5.'); last_tx=time.time()
def d5s(t=0.3):
    vals=[]; end=time.time()+t
    while time.time()<end:
        poll()
        if r.d5.last: vals.append(float(r.d5.last))
        time.sleep(0.03)
    return sum(vals)/max(1,len(vals)) if vals else 0.0
def clr():
    best={ax:0.0 for ax in AXES}
    for s in range(2):
        r.update()
        for ax in AXES:
            rel=((ax-r.h)%360)/22.5
            k0=int(rel)%16; k1=(k0+1)%16
            vals=[v for v in (r.ray(k0),r.ray(k1)) if v is not None]
            if vals: best[ax]=max(best[ax],min(vals))
        time.sleep(0.05)
    return best
def move(ax):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(0.5,target_h=ax,front_stop=0.24,speed=85)
    return tr
def spinhold():
    global last_tx
    L('gowest park+spin')
    while True:
        v=d5s(0.3)
        if goal_seen:
            r.wheels(0,0)
            if time.time()-last_tx>1: r.tx.write('A at_goal 1 GOAL STAY'); last_tx=time.time()
        else:
            r.wheels(30,-30)
            if time.time()-last_tx>1.5:
                r.tx.write('B COME: home on d5, I am parked spinning. d5 %.2f'%v); last_tx=time.time()
        if v>1.2 and not goal_seen:
            r.wheels(0,0); L('CLOSE d5=%.2f standing'%v)
            r.tx.write('A STOP TEST both stand still')
            t1=time.time()
            while time.time()-t1<40:
                poll()
                if goal_seen: break
                time.sleep(0.1)
        time.sleep(0.05)
lastdir=185
while True:
    poll()
    v=d5s(0.3)
    if goal_seen or v>0.65: spinhold()
    c=clr()
    # prefer 185, then whichever of 95/275 open, then 5
    for ax in (185,95,275,5):
        if c[ax]>=0.42:
            tr=move(ax)
            L('gw %s tr=%.2f d5=%.2f'%(ax,tr,v))
            break
    else:
        time.sleep(0.3)
