import sys,time,random
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== push start ===')
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
        r.tx.write('A pushing toward you. B STAY PARKED SPINNING.'); last_tx=time.time()
def hold():
    global last_tx
    L('HOLD')
    while True:
        r.wheels(0,0); poll()
        if time.time()-last_tx>1: r.tx.write('A at_goal %d holding'%(1 if goal_seen else 0)); last_tx=time.time()
        time.sleep(0.1)
def d5s(t=0.5):
    vals=[]; end=time.time()+t
    while time.time()<end:
        poll()
        if r.d5.last:
            try: vals.append(float(r.d5.last))
            except: pass
        time.sleep(0.03)
    return sum(vals)/max(1,len(vals))
def move(ax,fs=0.19,sp=60,dist=0.5):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(dist,target_h=ax,front_stop=fs,speed=sp)
    return tr
step=0
v=d5s(0.6)
while True:
    poll()
    if goal_seen: hold()
    if v>1.3:
        L('push CLOSE d5=%.2f'%v); r.tx.write('A adjacent STOP TEST'); hold()
    # try all four dirs aggressively, sorted by resulting d5 after trial step
    tried=[]
    baseline=v
    improved=False
    order=sorted(AXES,key=lambda ax:-{5:0,95:0,185:0,275:0}.get(ax,0))
    random.shuffle(order)
    for ax in order:
        poll()
        tr=move(ax,fs=0.19,sp=55,dist=0.5)
        step+=1
        nv=d5s(0.5)
        L('p%d %s tr=%.2f d5 %.2f->%.2f'%(step,ax,tr,baseline,nv))
        if goal_seen: hold()
        if nv>1.3:
            L('push CLOSE d5=%.2f'%nv); r.tx.write('A adjacent STOP TEST'); hold()
        if tr>0.2 and nv>baseline+0.03:
            v=nv; improved=True; break
        if tr>0.2:
            # go back
            bax=min(AXES,key=lambda x:abs(angdiff(x,(ax+180)%360)))
            move(bax,fs=0.16,sp=55,dist=tr)
    if not improved:
        # wander one LRV-ish random open step to escape plateau
        ax=random.choice(AXES)
        tr=move(ax,fs=0.19,sp=60)
        v=d5s(0.5)
        L('p%d wander %s tr=%.2f d5=%.2f'%(step,ax,tr,v))
