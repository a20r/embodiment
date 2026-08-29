import sys,time,math,collections
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== climb2 start ===')
AXES=[5,95,185,275]
while r.h is None or r.rays is None: r.update(); time.sleep(0.05)
goal_seen=False; last_tx=0
def poll():
    global goal_seen,last_tx
    r.update()
    for m in r.msgs: L('RX:',m)
    r.msgs[:]=[]
    for e in r.events:
        if 'goal=0' not in e: L('EV:',e)
        if 'goal=1' in e: goal_seen=True
    r.events[:]=[]
    if time.time()-last_tx>2:
        r.tx.write('A seekingB goal %d'%(1 if goal_seen else 0)); last_tx=time.time()
def d5avg(t=0.8):
    vals=[]; end=time.time()+t
    while time.time()<end:
        poll()
        if r.d5.last:
            try: vals.append(float(r.d5.last))
            except: pass
        time.sleep(0.04)
    return sum(vals)/max(1,len(vals))
def clearance():
    best={ax:0.0 for ax in AXES}
    for s in range(3):
        r.update()
        for ax in AXES:
            rel=((ax-r.h)%360)/22.5
            k0=int(rel)%16; k1=(k0+1)%16
            vals=[v for v in (r.ray(k0),r.ray(k1)) if v is not None]
            if vals: best[ax]=max(best[ax],min(vals))
        time.sleep(0.07)
    return best
def move(ax,dist=0.5):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(dist,target_h=ax,front_stop=0.23,speed=75)
    return tr
prev=None; prevd=None
while True:
    poll()
    if goal_seen:
        L('GOAL SEEN (climb2)'); r.wheels(0,0)
        while True:
            poll(); time.sleep(0.2)
            if time.time()-last_tx>1: r.tx.write('A at_goal 1'); last_tx=time.time()
    v0=d5avg(0.8)
    if v0>2.0:
        r.update()
        L('c2 CLOSE d5=%.2f rays=%s h=%s'%(v0,[None if x is None else round(x,2) for x in [r.ray(i) for i in range(16)]],r.h))
    c=clearance()
    opens=[ax for ax in AXES if c[ax]>0.55]
    L('c2 d5=%.3f c=%s'%(v0,{a:round(x,2) for a,x in c.items()}))
    if not opens:
        L('c2 boxed'); time.sleep(0.5); continue
    if len(opens)==1:
        best=opens[0]
    else:
        best=None; bestv=-1
        for ax in opens:
            if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
            tr,_=d.forward(0.22,target_h=ax,front_stop=0.2,speed=70)
            pv=d5avg(0.7)
            bax=min(AXES,key=lambda x:abs(angdiff(x,(ax+180)%360)))
            d.turn_to(bax); d.forward(max(tr,0.05),target_h=bax,front_stop=0.15,speed=70)
            L('c2 peek %s d5=%.3f base=%.3f'%(ax,pv,v0))
            if pv>bestv: bestv=pv; best=ax
        if bestv < v0-0.05: L('c2 all peeks worse; at max? d5=%.3f'%v0)
    tr=move(best)
    L('c2 moved %s tr=%.2f'%(best,tr))
