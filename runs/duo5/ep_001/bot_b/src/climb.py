import sys,time,math,collections
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff

r=Robot(); d=Drive(r)
log=open('/memory/run.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== climb start ===')
AXES=[5,95,185,275]
DIRV={5:(1,0),95:(0,1),185:(-1,0),275:(0,-1)}
while r.h is None or r.rays is None:
    r.update(); time.sleep(0.05)

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
        r.tx.write('A climb d5 %s goal %d'%(r.d5.last,1 if goal_seen else 0))
        last_tx=time.time()

def d5avg(t=1.0):
    vals=[]
    end=time.time()+t
    while time.time()<end:
        poll()
        if r.d5.last:
            try: vals.append(float(r.d5.last))
            except: pass
        time.sleep(0.05)
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

def move(ax):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(0.5,target_h=ax,front_stop=0.23,speed=75)
    return tr>0.28

pos=(0,0)
cur_d5=d5avg(1.2)
samples={pos:cur_d5}
tried=collections.defaultdict(set)  # cell -> axes tried
back=[]  # path stack
while True:
    poll()
    if goal_seen:
        L('AT GOAL (climb) holding.')
        r.wheels(0,0)
        while True:
            poll()
            if time.time()-last_tx>1:
                r.tx.write('A at_goal 1'); last_tx=time.time()
            time.sleep(0.2)
    c=clearance()
    cur_d5=d5avg(0.9)
    samples[pos]=cur_d5
    L('climb at %s d5=%.3f c=%s'%(pos,cur_d5,{a:round(x,2) for a,x in c.items()}))
    # choose untried open direction
    cands=[ax for ax in AXES if c[ax]>0.55 and ax not in tried[pos]]
    if cands:
        # prefer direction of best guess: unexplored neighbor
        ax=cands[0]
        # prefer neighbor with no sample (exploration) - all equal, pick first
        tried[pos].add(ax)
        v=DIRV[ax]; n=(pos[0]+v[0],pos[1]+v[1])
        if move(ax):
            newv=d5avg(0.9)
            if newv > cur_d5 - 0.03:
                back.append((pos,ax)); pos=n
                L('climb fwd %s -> %s d5 %.3f->%.3f'%(ax,pos,cur_d5,newv))
            else:
                # worse: go back
                rax=(ax+180)%360
                rax=min(AXES,key=lambda x:abs(angdiff(x,rax)))
                L('climb worse (%.3f->%.3f), returning'%(cur_d5,newv))
                move(rax)
        else:
            L('climb blocked %s'%ax)
    else:
        # local max? all dirs tried
        better=[p for p,v2 in samples.items() if v2>cur_d5+0.05]
        L('climb local max at %s d5=%.3f samples=%d better=%s'%(pos,cur_d5,len(samples),better[:3]))
        if back:
            p,ax=back.pop()
            rax=min(AXES,key=lambda x:abs(angdiff(x,(ax+180)%360)))
            move(rax); pos=p
        else:
            tried.clear(); time.sleep(1)
