import sys,time,math,collections
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== meet start ===')
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
        if 'goal=0' not in e: L('EV:',e)
        if 'goal=1' in e: goal_seen=True
    r.events[:]=[]
    if time.time()-last_tx>2:
        r.tx.write('A meet goal %d'%(1 if goal_seen else 0)); last_tx=time.time()
def d5avg(t=1.0):
    vals=[]; end=time.time()+t
    while time.time()<end:
        poll()
        if r.d5.last:
            try: vals.append(float(r.d5.last))
            except: pass
        time.sleep(0.04)
    return sum(vals)/max(1,len(vals)) if vals else 0.0
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
    tr,_=d.forward(0.5,target_h=ax,front_stop=0.30,speed=85)
    return tr
pos=(0,0)
samples={}; edges=collections.defaultdict(set)
stack=[]
while True:
    poll()
    if goal_seen:
        L('GOAL SEEN'); r.wheels(0,0)
        while True:
            poll(); time.sleep(0.2)
            if time.time()-last_tx>1: r.tx.write('A at_goal 1'); last_tx=time.time()
    v=d5avg(1.0); samples[pos]=v
    c=clearance()
    L('meet at %s d5=%.3f c=%s'%(pos,v,{a:round(x,2) for a,x in c.items()}))
    # candidate neighbors within radius 3 of (0,0), unsampled first
    cands=[]
    for ax in AXES:
        if c[ax]<0.55: continue
        dv=DIRV[ax]; n=(pos[0]+dv[0],pos[1]+dv[1])
        if abs(n[0])+abs(n[1])>7: continue
        edges[pos].add(n); edges[n].add(pos)
        if n not in samples: cands.append((ax,n))
    if cands:
        ax,n=cands[0]
        if move(ax)>0.28:
            stack.append(pos); pos=n
        continue
    # all local neighbors sampled: move toward best sampled cell if better than here
    best=max(samples,key=samples.get)
    if samples[best]>v+0.05 and best!=pos:
        # BFS to best
        q=[pos]; par={pos:None}
        while q:
            cu=q.pop(0)
            if cu==best: break
            for n in edges.get(cu,()):
                if n not in par: par[n]=cu; q.append(n)
        if best in par:
            path=[]; cu=best
            while cu is not None: path.append(cu); cu=par[cu]
            path=path[::-1]
            if len(path)>1:
                nxt=path[1]
                ax=[a for a,dv in DIRV.items() if (pos[0]+dv[0],pos[1]+dv[1])==nxt][0]
                if move(ax)>0.28: pos=nxt
                continue
    L('meet: settled at %s d5=%.3f samples=%s'%(pos,v,{k:round(x,2) for k,x in sorted(samples.items())}))
    time.sleep(3)
