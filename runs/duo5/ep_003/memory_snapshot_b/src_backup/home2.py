import sys,time,random
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== home2 start ===')
AXES=[5,95,185,275]
DIRV={5:(1,0),95:(0,1),185:(-1,0),275:(0,-1)}
OPP={5:185,185:5,95:275,275:95}
while r.h is None or r.rays is None: r.update(); time.sleep(0.05)
goal_seen=False; last_tx=0
MSG='B: STAY PARKED at your pos. SPIN WHEELS +40/-40 NONSTOP, do not stop, do not move. Your sound = my d5 beacon. A is homing to you. If d5>1.1 FREEZE.'
def poll():
    global goal_seen,last_tx
    r.update()
    for m in r.msgs: L('RX:',m)
    r.msgs[:]=[]
    for e in r.events:
        if 'goal=1' in e: goal_seen=True; L('EV:',e)
    r.events[:]=[]
    if time.time()-last_tx>2:
        r.tx.write(MSG); last_tx=time.time()
def d5now(t=0.6):
    vals=[]; end=time.time()+t
    while time.time()<end:
        poll()
        if r.d5.last:
            try: vals.append(float(r.d5.last))
            except: pass
        time.sleep(0.03)
    return sum(vals)/max(1,len(vals)) if vals else 0.0
def d5max(t=12.0):
    r.wheels(0,0)
    best=0.0; end=time.time()+t
    while time.time()<end:
        v=d5now(0.5)
        if v>best: best=v
        if goal_seen: return best
    return best
def clr():
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
def move(ax,dist=0.5,fs=0.20,sp=30):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(dist,target_h=ax,front_stop=fs,speed=sp)
    return tr
def hold_goal():
    global last_tx
    L('GOAL HOLD')
    while True:
        r.wheels(0,0); poll()
        if time.time()-last_tx>1: r.tx.write('A at_goal 1 GOAL STAY PUT'); last_tx=time.time()
        time.sleep(0.1)
def freeze_test(v):
    global last_tx
    r.wheels(0,0); L('TRIPWIRE d5=%.2f freeze+test'%v)
    t0=time.time()
    while time.time()-t0<70:
        poll()
        if goal_seen: hold_goal()
        if time.time()-last_tx>1.5:
            r.tx.write('A STOP TEST both freeze d5 %.2f'%v); last_tx=time.time()
        time.sleep(0.05)
    L('freeze test no goal')
def chk(v):
    if goal_seen: hold_goal()
    if v>1.1: freeze_test(v)
# grid memory in local frame
cx,cy=0,0; step=0
vis={}  # cell -> (step, bestd5)
def rec(v):
    global step
    step+=1
    old=vis.get((cx,cy),(0,0.0))
    vis[(cx,cy)]=(step,max(old[1],v))
mode='WAIT'
v=d5max(10); rec(v); L('home cell 0,0 d5max=%.2f'%v)
while True:
    poll()
    if goal_seen: hold_goal()
    # burst-gated: quick check
    v=d5now(0.8); chk(v)
    if v<0.40:
        # wait for burst, but not forever: after 45s idle, do one exploratory LRV move
        t0=time.time(); got=False
        while time.time()-t0<45:
            v=d5now(0.6); chk(v)
            if v>0.45: got=True; break
        if not got:
            c=clr()
            opens=[ax for ax in AXES if c[ax]>=0.40]
            if opens:
                def key(ax):
                    n=(cx+DIRV[ax][0],cy+DIRV[ax][1])
                    return (vis.get(n,(0,0))[0], -vis.get(n,(0,0))[1], random.random())
                ax=min(opens,key=key)
                tr=move(ax)
                if tr>0.2:
                    cx+=DIRV[ax][0]; cy+=DIRV[ax][1]
                    v=d5max(8); rec(v); L('idlewander %s -> %d,%d d5max=%.2f'%(ax,cx,cy,v))
            continue
    # burst active: greedy neighbor moves while signal high
    L('burst active v=%.2f at %d,%d'%(v,cx,cy))
    while True:
        poll()
        if goal_seen: hold_goal()
        base=d5now(0.9); chk(base)
        if base<0.35: L('burst ended at %d,%d'%(cx,cy)); break
        c=clr()
        opens=[ax for ax in AXES if c[ax]>=0.40]
        if not opens: time.sleep(0.3); continue
        # prefer unvisited/high-value neighbors
        def key(ax):
            n=(cx+DIRV[ax][0],cy+DIRV[ax][1])
            st,bv=vis.get(n,(0,0.0))
            return (st, -bv, random.random())
        moved=False
        for ax in sorted(opens,key=key):
            tr=move(ax)
            if tr<=0.2: continue
            nv=d5now(1.0); chk(nv)
            L('try %s %d,%d %.2f->%.2f'%(ax,cx+DIRV[ax][0],cy+DIRV[ax][1],base,nv))
            if nv>base-0.04:
                cx+=DIRV[ax][0]; cy+=DIRV[ax][1]; rec(nv); moved=True; break
            else:
                move(OPP[ax],dist=tr,fs=0.13,sp=22)
        if not moved:
            L('stuck at %d,%d base=%.2f'%(cx,cy,base))
            time.sleep(0.5)
