import sys,time,math,collections,random
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== rdv start ===')
AXES=[5,95,185,275]
DIRV={5:(1,0),95:(0,1),185:(-1,0),275:(0,-1)}
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
        r.tx.write('A rdv homing goal %d'%(1 if goal_seen else 0))
        if int(time.time())%10<2:
            r.tx.write('RENDEZVOUS NOW. B park+spin until I arrive (d5>1.5) then stand still, co-location test.')
        last_tx=time.time()
def d5s(t=0.6):
    vals=[]; end=time.time()+t
    while time.time()<end:
        poll()
        if r.d5.last:
            try: vals.append(float(r.d5.last))
            except: pass
        time.sleep(0.03)
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
        time.sleep(0.05)
    return best
def move(ax,dist=0.5,fs=0.30,sp=85):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(dist,target_h=ax,front_stop=fs,speed=sp)
    return tr
def hold_goal():
    global last_tx
    L('GOAL! holding')
    while True:
        r.wheels(0,0); poll()
        if time.time()-last_tx>1:
            r.tx.write('A at_goal 1 GOAL CONFIRMED stay here'); last_tx=time.time()
        time.sleep(0.1)
# LRV-walk biased by d5 gradient: at each cell, peek only if d5>0.8; else pick LRV neighbor weighted by d5 trend
lastvisit={}; step=0; cx,cy=0,0; cur=5
best_d5=0
while True:
    poll()
    if goal_seen: hold_goal()
    v0=d5s(0.5)
    c=clearance()
    opens=[ax for ax in AXES if c[ax]>=0.42]
    if not opens:
        L('rdv boxed'); time.sleep(0.4); continue
    if v0>1.5:
        L('rdv VERY CLOSE d5=%.2f: stopping for co-location test. rays=%s'%(v0,[None if x is None else round(x,2) for x in r.rays]))
        r.wheels(0,0)
        t0=time.time()
        while time.time()-t0<20:
            poll()
            if goal_seen: hold_goal()
            if time.time()-last_tx>1:
                r.tx.write('A adjacent d5 %.2f co-location test: STOP and stand still'%v0); last_tx=time.time()
            time.sleep(0.1)
        v0=d5s(0.8)
        if v0>1.5:
            # try to get even closer: move toward most open dir, small step
            ax=max(opens,key=lambda a:c[a])
            move(ax,0.3,fs=0.18,sp=60)
            L('rdv nudge %s d5 now %.2f'%(ax,d5s(0.6)))
        continue
    if v0>0.7 and len(opens)>1:
        # peek mode
        best=None; bv=-1
        for ax in opens:
            if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
            tr,_=d.forward(0.2,target_h=ax,front_stop=0.2,speed=70)
            pv=d5s(0.5)
            bax=min(AXES,key=lambda x:abs(angdiff(x,(ax+180)%360)))
            d.turn_to(bax); d.forward(max(tr,0.05),target_h=bax,front_stop=0.13,speed=70)
            if pv>bv: bv=pv; best=ax
        ax=best
    else:
        def key(ax):
            vv=DIRV[ax]; n=(cx+vv[0],cy+vv[1])
            return (lastvisit.get(n,-1), 0 if ax==cur else 1, random.random())
        ax=min(opens,key=key)
    tr=move(ax); step+=1
    if tr>0.25:
        cur=ax; vv=DIRV[ax]; cx+=vv[0]; cy+=vv[1]; lastvisit[(cx,cy)]=step
        L('r%d %s pos %d,%d d5=%.2f'%(step,ax,cx,cy,v0))
    else:
        vv=DIRV[ax]; lastvisit[(cx+vv[0],cy+vv[1])]=step+200
        L('r%d %s SHORT'%(step,ax))
