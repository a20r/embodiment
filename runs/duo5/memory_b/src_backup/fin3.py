import sys,time,random
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== fin3 strict climb + tight doors everywhere ===')
AXES=[5,95,185,275]
OPP={5:185,185:5,95:275,275:95}
goal_seen=False; last_tx=0
MSG='A: very close (0.9 zone), testing every tight door. Keep rocking. FREEZE at d5>1.05.'
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
def d5s(t=0.55):
    vals=[]; end=time.time()+t
    while time.time()<end:
        poll()
        if r.d5.last:
            try: vals.append(float(r.d5.last))
            except: pass
        time.sleep(0.03)
    return sum(vals)/max(1,len(vals)) if vals else 0.0
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
    if v>1.02:
        r.wheels(0,0); L('TRIPWIRE %.2f'%v)
        t0=time.time()
        while time.time()-t0<90:
            poll()
            if goal_seen: hold_goal()
            if time.time()-last_tx>1.5:
                r.tx.write('A STOP TEST freeze d5 %.2f'%v); last_tx=time.time()
            time.sleep(0.05)
        L('trip no goal, continue')
def mv(ax,fs=0.15,sp=26,dist=0.5):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(dist,target_h=ax,front_stop=fs,speed=sp)
    return tr
while r.h is None or r.rays is None: r.update(); time.sleep(0.05)
visited=set()
lastax=None
while True:
    poll()
    if goal_seen: hold_goal()
    base=d5s(0.6); chk(base)
    # strict gradient step
    improved=False
    order=[lastax]+[a for a in (275,95,5,185) if a!=lastax] if lastax else [275,95,5,185]
    for ax in order:
        tr=mv(ax,fs=0.13,sp=22)
        if tr<=0.25:
            if tr>0.03: mv(OPP[ax],fs=0.11,sp=18,dist=tr)
            continue
        nv=d5s(0.55); chk(nv)
        if nv>base+0.012:
            L('f3 %s %.2f->%.2f'%(ax,base,nv)); improved=True; lastax=ax; break
        mv(OPP[ax],fs=0.11,sp=20,dist=tr)
    if improved: continue
    # local max: exhaustive tight door passes incl diag jiggle
    L('f3 MAX %.2f, tight door sweep'%base)
    lastax=None
    found=False
    for ax in (5,185,275,95):
        tr=mv(ax,fs=0.115,sp=18)
        if tr>0.35:
            nv=d5s(0.6); chk(nv)
            L('f3 tightdoor %s %.2f->%.2f'%(ax,base,nv))
            if nv>base-0.02: found=True; lastax=ax; break
            mv(OPP[ax],fs=0.11,sp=18,dist=tr)
        elif tr>0.03:
            mv(OPP[ax],fs=0.11,sp=18,dist=tr)
    if found: continue
    # nothing: random sidestep to escape micro-basin
    ax=random.choice([275,95,5,185])
    mv(ax,fs=0.15)
