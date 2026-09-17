import sys,time,math,collections
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== west dfs start ===')
AXES=[5,95,185,275]
DIRV={5:(1,0),95:(0,1),185:(-1,0),275:(0,-1)}
while r.h is None or r.rays is None: r.update(); time.sleep(0.05)
goal_seen=False; last_tx=0; b_at_goal=False
d5buf=collections.deque(maxlen=40)
cx,cy=0,0
def poll():
    global goal_seen,last_tx,b_at_goal
    r.update()
    for m in r.msgs:
        L('RX:',m)
        if 'at_goal 1' in m: b_at_goal=True
    r.msgs[:]=[]
    for e in r.events:
        if 'goal=1' in e: goal_seen=True; L('EV:',e)
    r.events[:]=[]
    if r.d5.last:
        try: d5buf.append(float(r.d5.last))
        except: pass
    if time.time()-last_tx>2:
        r.tx.write('A pos %d %d goal %d'%(cx,cy,1 if goal_seen else 0)); last_tx=time.time()
def d5a(): return sum(d5buf)/len(d5buf) if d5buf else 0.0
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
    tr,_=d.forward(0.5,target_h=ax,front_stop=0.30,speed=90)
    return tr
def at_goal_hold():
    global last_tx
    L('AT GOAL park+spin pos %d,%d'%(cx,cy))
    while True:
        r.wheels(40,-40); poll()
        if time.time()-last_tx>1:
            r.tx.write('A GOAL FOUND at_goal 1 come climb d5'); last_tx=time.time()
        time.sleep(0.1)
def climb_to_B():
    # simple: greedy d5 climb with peeks
    L('CLIMB to B start')
    while True:
        poll()
        if goal_seen: at_goal_hold()
        c=clearance(); opens=[ax for ax in AXES if c[ax]>0.5]
        base=d5a()
        best=None; bv=-1
        if len(opens)==1: best=opens[0]
        else:
            for ax in opens:
                if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
                tr,_=d.forward(0.2,target_h=ax,front_stop=0.22,speed=70)
                end=time.time()+0.5; vals=[]
                while time.time()<end:
                    poll(); 
                    if r.d5.last: vals.append(float(r.d5.last))
                    time.sleep(0.03)
                pv=sum(vals)/max(1,len(vals))
                bax=min(AXES,key=lambda x:abs(angdiff(x,(ax+180)%360)))
                d.turn_to(bax); d.forward(max(tr,0.05),target_h=bax,front_stop=0.14,speed=70)
                if pv>bv: bv=pv; best=ax
        if best is None: time.sleep(0.3); continue
        move(best)
        L('climb %s d5=%.2f'%(best,d5a()))
visited={}   # (x,y) -> d5
walls=set()
stack=[]
step=0
while True:
    poll()
    if goal_seen: at_goal_hold()
    if b_at_goal: climb_to_B()
    c=clearance()
    visited[(cx,cy)]=d5a()
    nbrs=[]
    for ax in AXES:
        dv=DIRV[ax]; n=(cx+dv[0],cy+dv[1])
        if c[ax]<0.42: walls.add((cx,cy,ax)); continue
        if n[0]>1: continue
        if n not in visited: nbrs.append((ax,n))
    if nbrs:
        # prefer lowest resulting x (westward), then straight ahead
        nbrs.sort(key=lambda t:(t[1][0], 0 if abs(angdiff(t[0],r.h))<10 else 1))
        ax,n=nbrs[0]
        tr=move(ax); step+=1
        if tr>0.25:
            stack.append((cx,cy)); cx,cy=n
            L('e%d %s pos %d,%d d5=%.2f c=%s'%(step,ax,cx,cy,d5a(),{a:round(v,2) for a,v in c.items()}))
        else:
            visited[n]=-1; walls.add((cx,cy,ax))
            L('e%d %s SHORT tr=%.2f'%(step,ax,tr))
    else:
        if not stack:
            L('DFS exhausted at %d,%d visited=%d'%(cx,cy,len(visited))); 
            # restart fresh frame
            visited={(cx,cy):d5a()}; stack=[]
            time.sleep(0.5); continue
        px,py=stack.pop()
        ax=[a for a,dv in DIRV.items() if (cx+dv[0],cy+dv[1])==(px,py)]
        if not ax:
            L('backtrack jump fail'); stack=[]; visited={(cx,cy):0}; continue
        tr=move(ax[0]); step+=1
        if tr>0.25: cx,cy=px,py
        else: L('backtrack blocked!?')
