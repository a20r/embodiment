import sys,time,math,collections
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== meet2 start ===')
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
        r.tx.write('A homing d5map goal %d. B keep spinning in place.'%(1 if goal_seen else 0)); last_tx=time.time()
def hold_goal():
    global last_tx
    L('GOAL! holding'); 
    while True:
        r.wheels(0,0); poll()
        if time.time()-last_tx>1: r.tx.write('A at_goal 1 GOAL'); last_tx=time.time()
        time.sleep(0.1)
def d5s(t=0.7):
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
def move(ax,dist=0.5,fs=0.30,sp=85):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(dist,target_h=ax,front_stop=fs,speed=sp)
    return tr
pos=(0,0); samples={}; opens={}; R=8
blocked=set()
def axto(p,n):
    for a,dv in DIRV.items():
        if (p[0]+dv[0],p[1]+dv[1])==n: return a
    return None
while True:
    poll()
    if goal_seen: hold_goal()
    if pos not in samples:
        v=d5s(0.7); c=clearance()
        samples[pos]=v; opens[pos]=[ax for ax in AXES if c[ax]>=0.45]
        L('m at %s d5=%.3f opens=%s'%(pos,v,opens[pos]))
        if v>1.5:
            L('m VERY CLOSE d5=%.2f co-location attempt'%v)
            r.tx.write('A adjacent d5 %.2f. B STOP spinning, stand still.'%v)
            # try to enter B cell: move toward the most open direction slowly
            for ax in sorted(AXES,key=lambda a:-c[a]):
                if c[ax]>0.5:
                    move(ax,0.45,fs=0.16,sp=55)
                    nv=d5s(0.8); L('m nudge %s d5=%.2f'%(ax,nv))
                    if goal_seen: hold_goal()
                    if nv>v: break
                    bax=min(AXES,key=lambda x:abs(angdiff(x,(ax+180)%360)))
                    move(bax,0.45,fs=0.16,sp=55)
            t0=time.time()
            while time.time()-t0<15:
                poll()
                if goal_seen: hold_goal()
                time.sleep(0.1)
            samples.pop(pos,None)  # resample
            continue
    # find action: unsampled open neighbor here?
    cand=None
    for ax in opens.get(pos,[]):
        dv=DIRV[ax]; n=(pos[0]+dv[0],pos[1]+dv[1])
        if n not in samples and (pos,n) not in blocked and abs(n[0])+abs(n[1])<=R:
            cand=(ax,n); break
    if cand:
        ax,n=cand
        tr=move(ax)
        if tr>0.25: pos=n
        else: blocked.add((pos,n)); L('m blocked %s->%s'%(pos,n))
        continue
    # BFS to nearest cell with frontier
    def hasfrontier(p):
        for ax in opens.get(p,[]):
            dv=DIRV[ax]; n=(p[0]+dv[0],p[1]+dv[1])
            if n not in samples and (p,n) not in blocked and abs(n[0])+abs(n[1])<=R: return True
        return False
    par={pos:None}; q=[pos]; target=None
    while q:
        cu=q.pop(0)
        if cu!=pos and hasfrontier(cu): target=cu; break
        for ax in opens.get(cu,[]):
            dv=DIRV[ax]; n=(cu[0]+dv[0],cu[1]+dv[1])
            if n in samples and n not in par and (cu,n) not in blocked:
                par[n]=cu; q.append(n)
    if target is None:
        best=max(samples,key=samples.get)
        L('m NO FRONTIER. best=%s d5=%.2f samples=%d. going there.'%(best,samples[best],len(samples)))
        if best==pos:
            L('m at best. holding 10s'); 
            t0=time.time()
            while time.time()-t0<10: poll(); time.sleep(0.1)
            samples.pop(pos,None); continue
        # BFS path to best
        par={pos:None}; q=[pos]
        while q:
            cu=q.pop(0)
            if cu==best: break
            for ax in opens.get(cu,[]):
                dv=DIRV[ax]; n=(cu[0]+dv[0],cu[1]+dv[1])
                if n in samples and n not in par and (cu,n) not in blocked:
                    par[n]=cu; q.append(n)
        target=best
        if best not in par: L('m best unreachable, reset'); samples={}; opens={}; blocked=set(); pos=(0,0); continue
    path=[]; cu=target
    while cu is not None: path.append(cu); cu=par[cu]
    path=path[::-1]
    nxt=path[1]
    ax=axto(pos,nxt)
    tr=move(ax)
    if tr>0.25: pos=nxt
    else: blocked.add((pos,nxt)); L('m blocked path %s->%s'%(pos,nxt))
