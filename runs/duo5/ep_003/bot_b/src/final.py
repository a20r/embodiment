import sys,time,random
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== final start ===')
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
def hold(spin=False):
    global last_tx
    L('HOLD spin=%s'%spin)
    t0=time.time()
    while True:
        if spin and not goal_seen: r.wheels(30,-30)
        else: r.wheels(0,0)
        poll()
        v=float(r.d5.last) if r.d5.last else 0
        if time.time()-last_tx>1.5:
            if goal_seen: r.tx.write('A at_goal 1 GOAL CONFIRMED STAY')
            else: r.tx.write('B COME TO ME: home on d5 NOW. I am parked spinning at my d5-max. When your d5>1.2 STOP TEST stand still. goal 0')
            last_tx=time.time()
            L('hold d5=%.2f goal=%s'%(v,goal_seen))
        if v>1.2 and not goal_seen:
            r.wheels(0,0)
            L('hold: B CLOSE d5=%.2f standing still'%v)
            r.tx.write('A sees you close d5 %.2f. STOP TEST both stand still.'%v)
            t1=time.time()
            while time.time()-t1<45:
                poll()
                if goal_seen:
                    r.tx.write('A at_goal 1 GOAL CONFIRMED STAY'); L('GOAL during test!')
                    break
                time.sleep(0.1)
            if not goal_seen:
                L('test over no goal; resume spin')
        time.sleep(0.05)
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
bestv=0.0; bestpos=(0,0)
while True:
    poll()
    if goal_seen: hold()
    v=d5s(0.35)
    if v>bestv: bestv=v; bestpos=(cx,cy)
    if v>0.70:
        L('final at high d5=%.2f pos %d,%d -> park+spin'%(v,cx,cy))
        r.tx.write('B COME TO ME now, home on d5. I park+spin here.')
        hold(spin=True)
    c=clearance()
    opens=[ax for ax in AXES if c[ax]>=0.42]
    if not opens:
        time.sleep(0.4); continue
    def key(ax):
        vv=DIRV[ax]; n=(cx+vv[0],cy+vv[1])
        return (lastvisit.get(n,-1), abs(n[0]-bestpos[0])+abs(n[1]-bestpos[1]), random.random())
    ax=min(opens,key=key)
    tr=move(ax); step+=1
    if tr>0.20:
        cur=ax; vv=DIRV[ax]; cx+=vv[0]; cy+=vv[1]; lastvisit[(cx,cy)]=step
        L('f%d %s pos %d,%d d5=%.2f'%(step,ax,cx,cy,v))
    else:
        vv=DIRV[ax]; lastvisit[(cx+vv[0],cy+vv[1])]=step+80
