import sys,time,random
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== west2: enter x=-2 col, climb north ===')
AXES=[5,95,185,275]
DIRV={5:(1,0),95:(0,1),185:(-1,0),275:(0,-1)}
goal_seen=False; last_tx=0
MSG='B: I found doors on the far west side, coming around to your column from the west. KEEP ROCKING/SPINNING LOUD. FREEZE when d5>1.05.'
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
def d5s(t=0.6):
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
        while time.time()-t0<70:
            poll()
            if goal_seen: hold_goal()
            if time.time()-last_tx>1.5:
                r.tx.write('A STOP TEST freeze d5 %.2f'%v); last_tx=time.time()
            time.sleep(0.05)
        L('trip no goal, continue')
def mv(ax,fs=0.17,sp=26):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(0.5,target_h=ax,front_stop=fs,speed=sp)
    return tr
while r.h is None or r.rays is None: r.update(); time.sleep(0.05)
# phase A: go north up to 8 cells, take first west opening
entered=False
for i in range(9):
    poll(); v=d5s(0.5); chk(v)
    c=clr()
    L('w2 A%d d5=%.2f c=%s'%(i,v,{a:round(x,2) for a,x in c.items()}))
    if c[185]>=0.50:
        tr=mv(185)
        if tr>0.3:
            L('w2 entered west col'); entered=True; break
    if c[95]<0.34: L('w2 north blocked'); break
    mv(95)
# phase B: climb north in west column, LRV w/ north bias
cx,cy=0,0; step=0; vis={(0,0):0}
best=0.0
while True:
    poll()
    if goal_seen: hold_goal()
    v=d5s(0.6); chk(v)
    if v>best: best=v
    c=clr()
    opens=[ax for ax in AXES if c[ax]>=0.38]
    if not opens: time.sleep(0.3); continue
    PR={95:0,185:1,5:2,275:3} if v>=best-0.1 else {275:0,185:1,5:2,95:3}
    def key(ax):
        n=(cx+DIRV[ax][0],cy+DIRV[ax][1])
        return (vis.get(n,-1), PR[ax], random.random())
    ax=min(opens,key=key)
    tr=mv(ax); step+=1
    if tr>0.2:
        cx+=DIRV[ax][0]; cy+=DIRV[ax][1]; vis[(cx,cy)]=step
        L('w2 B%d %s %d,%d d5=%.2f c=%s'%(step,ax,cx,cy,v,{a:round(x,2) for a,x in c.items()}))
    else:
        vis[(cx+DIRV[ax][0],cy+DIRV[ax][1])]=step+200
        L('w2 B%d %s BLOCK tr=%.2f'%(step,ax,tr))
