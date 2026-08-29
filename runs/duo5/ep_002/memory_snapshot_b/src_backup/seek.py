import sys,time,random,collections
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== seek start ===')
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
        r.tx.write('B: WANDER/EXPLORE too. If your d5>1.3 broadcast STOP TEST and freeze; I do same. goal %d'%(1 if goal_seen else 0)); last_tx=time.time()
def hold():
    global last_tx
    L('HOLD (goal or close)')
    while True:
        r.wheels(0,0); poll()
        if time.time()-last_tx>1:
            r.tx.write('A at_goal %d holding'%(1 if goal_seen else 0)); last_tx=time.time()
        time.sleep(0.1)
def d5s(t=0.35):
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
def move(ax):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(0.5,target_h=ax,front_stop=0.24,speed=80)
    return tr
lastvisit={(0,0):0}; cx,cy=0,0; cur=5; step=0
best=(0.0,(0,0))
while True:
    poll()
    if goal_seen: hold()
    v=d5s(0.35)
    if v>best[0]: best=(v,(cx,cy))
    if v>1.2:
        L('seek CLOSE d5=%.2f at %d,%d'%(v,cx,cy))
        r.tx.write('A adjacent d5 %.2f STOP TEST'%v)
        hold()
    c=clearance()
    opens=[ax for ax in AXES if c[ax]>=0.42]
    if not opens:
        L('seek boxed'); time.sleep(0.4); continue
    def key(ax):
        vv=DIRV[ax]; n=(cx+vv[0],cy+vv[1])
        return (lastvisit.get(n,-1), 0 if ax==cur else 1, random.random())
    ax=min(opens,key=key)
    tr=move(ax); step+=1
    if tr>0.20:
        cur=ax; vv=DIRV[ax]; cx+=vv[0]; cy+=vv[1]; lastvisit[(cx,cy)]=step
        L('s%d %s pos %d,%d d5=%.2f best=%.2f@%s'%(step,ax,cx,cy,v,best[0],best[1]))
    else:
        vv=DIRV[ax]; lastvisit[(cx+vv[0],cy+vv[1])]=step+80
        L('s%d %s SHORT'%(step,ax))
