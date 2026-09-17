import sys,time,random
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== fin2: peak then south-crawl tight-door hunt ===')
AXES=[5,95,185,275]
DIRV={5:(1,0),95:(0,1),185:(-1,0),275:(0,-1)}
OPP={5:185,185:5,95:275,275:95}
goal_seen=False; last_tx=0
MSG='A: hunting tight doors along wall to your corridor mouth (south side). You stay rocking. FREEZE at d5>1.05.'
def poll():
    global goal_seen,last_tx
    r.update()
    for m in r.msgs: L('RX:',m)
    r.msgs[:]=[]
    for e in r.events:
        if 'goal=1' in e: goal_seen=True; L('EV:',e)
    r.events[:]=[]
    if time.time()-last_tx>3:
        r.tx.write(MSG); last_tx=time.time()
def d5s(t=0.5):
    vals=[]; end=time.time()+t
    while time.time()<end:
        poll()
        if r.d5.last:
            try: vals.append(float(r.d5.last))
            except: pass
        time.sleep(0.03)
    return sum(vals)/max(1,len(vals)) if vals else 0.0
def clr():
    best={ax:0.0 for ax in AXES}
    for s in range(3):
        r.update()
        for ax in AXES:
            rel=((ax-r.h)%360)/22.5
            k0=int(rel)%16; k1=(k0+1)%16
            vals=[v for v in (r.ray(k0),r.ray(k1)) if v is not None and v>=0]
            if vals: best[ax]=max(best[ax],min(vals))
        time.sleep(0.05)
    return best
def hold_goal():
    global last_tx
    L('GOAL HOLD')
    while True:
        r.wheels(0,0); poll()
        if time.time()-last_tx>1: r.tx.write('A at_goal 1 GOAL STAY PUT'); last_tx=time.time()
        time.sleep(0.1)
def chk(v):
    global last_tx
    if goal_seen: hold_goal()
    if v>1.05:
        r.wheels(0,0); L('TRIPWIRE %.2f'%v)
        t0=time.time()
        while time.time()-t0<90:
            poll()
            if goal_seen: hold_goal()
            if time.time()-last_tx>1.5:
                r.tx.write('A STOP TEST freeze d5 %.2f'%v); last_tx=time.time()
            time.sleep(0.05)
        L('trip no goal, continue')
def mv(ax,fs=0.17,sp=28,dist=0.5):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(dist,target_h=ax,front_stop=fs,speed=sp)
    return tr
def door_test(ax):
    """attempt tight passage; return True if got through (>0.35), else revert."""
    tr=mv(ax,fs=0.13,sp=20)
    if tr>0.35:
        return True
    if tr>0.03: mv(OPP[ax],fs=0.11,sp=18,dist=tr)
    return False
while r.h is None or r.rays is None: r.update(); time.sleep(0.05)
# phase A: strict hill climb to local max (using LRV wander if low)
cx,cy=0,0; step=0; vis={(0,0):0}; best=0.0; bestpos=(0,0)
def wander_until(th):
    global cx,cy,step,best,bestpos
    while True:
        poll()
        if goal_seen: hold_goal()
        v=d5s(0.4); chk(v)
        if v>best: best=v; bestpos=(cx,cy)
        if v>=th: L('fin2 zone %.2f at %d,%d'%(v,cx,cy)); return v
        c=clr()
        opens=[ax for ax in AXES if c[ax]>=0.40]
        if not opens: time.sleep(0.3); continue
        def key(ax):
            n=(cx+DIRV[ax][0],cy+DIRV[ax][1])
            return (vis.get(n,-1), abs(n[0]-bestpos[0])+abs(n[1]-bestpos[1]), random.random())
        ax=min(opens,key=key)
        tr=mv(ax); step+=1
        if tr>0.2:
            cx+=DIRV[ax][0]; cy+=DIRV[ax][1]; vis[(cx,cy)]=step
            if step%5==0: L('f2w%d %d,%d d5=%.2f'%(step,cx,cy,v))
        else:
            vis[(cx+DIRV[ax][0],cy+DIRV[ax][1])]=step+300
v=wander_until(0.82)
# strict climb
while True:
    poll(); base=d5s(0.6); chk(base)
    c=clr(); moved=False
    for ax in sorted(AXES,key=lambda a:-c[a]):
        if c[ax]<0.40: continue
        tr=mv(ax)
        if tr<=0.2: continue
        nv=d5s(0.6); chk(nv)
        if nv>base+0.015:
            L('f2 climb %s %.2f->%.2f'%(ax,base,nv)); moved=True; break
        mv(OPP[ax],fs=0.13,sp=22,dist=tr)
    if not moved: break
peak=d5s(0.8)
L('f2 PEAK %.2f'%peak)
# phase B: crawl south testing E and W tight doors each cell; then north
for cdir in (275,95):
    steps=0
    while steps<14:
        poll()
        if goal_seen: hold_goal()
        base=d5s(0.4); chk(base)
        for side in (5,185):
            if door_test(side):
                nv=d5s(0.6); chk(nv)
                L('f2 DOOR %s d5 %.2f->%.2f'%(side,base,nv))
                if nv>base-0.05:
                    # inside: head north climbing
                    while True:
                        poll()
                        if goal_seen: hold_goal()
                        b2=d5s(0.5); chk(b2)
                        c=clr()
                        cand=[a for a in (95,5,185,275) if c[a]>=0.36]
                        adv=False
                        for a in cand:
                            tr=mv(a,fs=0.15)
                            if tr<=0.2: continue
                            n2=d5s(0.5); chk(n2)
                            L('f2 in %s %.2f->%.2f'%(a,b2,n2))
                            if n2>b2-0.01: adv=True; break
                            mv(OPP[a],fs=0.13,sp=22,dist=tr)
                        if not adv: break
                else:
                    mv(OPP[side],fs=0.13,sp=20)
        tr=mv(cdir,fs=0.15)
        steps+=1
        if tr<=0.2:
            # tight gap straight ahead?
            if not door_test(cdir):
                L('f2 crawl %s end after %d'%(cdir,steps)); break
L('f2 crawl done, holding+rebroadcast')
while True:
    poll()
    if goal_seen: hold_goal()
    r.wheels(0,0); time.sleep(0.1)
