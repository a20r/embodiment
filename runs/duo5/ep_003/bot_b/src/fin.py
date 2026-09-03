import sys,time,random
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== fin: to 0.8 zone, then mouth at south, corridor north ===')
AXES=[5,95,185,275]
DIRV={5:(1,0),95:(0,1),185:(-1,0),275:(0,-1)}
OPP={5:185,185:5,95:275,275:95}
goal_seen=False; last_tx=0
MSG='A: understood. Finding south mouth then heading 350 up corridor to you. FREEZE at d5>1.05, will say STOP TEST.'
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
def d5s(t=0.45):
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
def mv(ax,fs=0.17,sp=28):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(0.5,target_h=ax,front_stop=fs,speed=sp)
    return tr
while r.h is None or r.rays is None: r.update(); time.sleep(0.05)
# phase 1: LRV to 0.78 zone
cx,cy=0,0; step=0; vis={(0,0):0}; best=0.0; bestpos=(0,0)
while True:
    poll()
    if goal_seen: hold_goal()
    v=d5s(0.4); chk(v)
    if v>best: best=v; bestpos=(cx,cy)
    if v>=0.76: L('fin zone reached %.2f at %d,%d'%(v,cx,cy)); break
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
        L('f%d %s %d,%d d5=%.2f'%(step,ax,cx,cy,v))
    else:
        vis[(cx+DIRV[ax][0],cy+DIRV[ax][1])]=step+300
# phase 2: hunt mouth: go SOUTH along wall, testing EAST and NORTH each cell; enter corridor north when d5 climbs
while True:
    poll()
    if goal_seen: hold_goal()
    base=d5s(0.5); chk(base)
    c=clr()
    # try north first (maybe already at mouth)
    prog=False
    for ax in (95,5,275,185):
        if c[ax]<0.40: continue
        tr=mv(ax)
        if tr<=0.2: continue
        nv=d5s(0.5); chk(nv)
        L('fin try %s %.2f->%.2f'%(ax,base,nv))
        if nv>base-0.02:
            prog=True; break
        mv(OPP[ax],fs=0.13,sp=22)
    if not prog:
        L('fin stuck, base=%.2f'%base)
        # jiggle south
        if c[275]>=0.40: mv(275)
        elif c[5]>=0.40: mv(5)
        elif c[185]>=0.40: mv(185)
