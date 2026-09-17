import sys,time,math,json
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff

r=Robot(); d=Drive(r)
log=open('/memory/run.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== explore2 start ===')

CELL=0.5
AXES=[5,95,185,275]        # world bearings of +x,+y,-x,-y in grid frame
DIRV={5:(1,0),95:(0,1),185:(-1,0),275:(0,-1)}

t0=time.time()
while r.h is None or r.rays is None:
    r.update(); time.sleep(0.05)

pos=(0,0)
visited={pos}
blocked=set()   # (cell,dircode)
stack=[]
last_tx=0
goal_seen=False
peer_goal=False

def sense_clear():
    # clearance toward each axis using 2 nearest rays; take max of a few samples
    best={ax:0.0 for ax in AXES}
    for s in range(3):
        r.update()
        for ax in AXES:
            rel=((ax-r.h)%360)/22.5
            k0=int(rel)%16; k1=(k0+1)%16
            vals=[v for v in (r.ray(k0),r.ray(k1)) if v is not None]
            if vals: best[ax]=max(best[ax],min(vals))
        time.sleep(0.08)
    return best

def relocalize():
    # snap along axes using nearest wall distances
    r.update()
    if r.h is None: return
    for ax in AXES:
        rel=((ax-r.h)%360)/22.5
        k=int(round(rel))%16
        # only use rays closely aligned with axis
        if abs(rel-round(rel))>0.15: continue
        v=r.ray(k)
        if v is None or v>1.4: continue
        # wall plane at (m+0.5)*CELL from cell center
        # our coordinate along axis: cell center + delta; measured dist to wall v
        m=round((v-0.25)/CELL)
        exp=0.25+CELL*m
        delta=exp-v   # we are delta past center toward wall
        ux,uy=DIRV[ax]
        # world-axis frame: adjust dead reckoning x,y
        r.x+= (ux*delta) if False else 0
    return

def comm(force=False):
    global last_tx,peer_goal
    now=time.time()
    if force or now-last_tx>2:
        r.tx.write(json.dumps({'id':'A','c':list(pos),'g':1 if goal_seen else 0}))
        last_tx=now
    for m in r.msgs:
        L('RX:',m)
        if '"g": 1' in m or '"g":1' in m or 'at_goal' in m: peer_goal=True
    r.msgs[:]=[]
    for e in r.events:
        if 'goal=0' not in e: L('EV:',e)
    r.events[:]=[]

def move(ax):
    # turn and advance one cell; returns True if full cell traversed
    if abs(angdiff(ax,r.h))>8:
        d.turn_to(ax)
    tr,reason=d.forward(CELL,target_h=ax,front_stop=0.23,speed=75)
    if reason=='wall' and tr<0.30:
        # bumped early: wall; back to center-ish
        if tr>0.07: d.forward(-0)  # noop
        return False,tr
    return True,tr

step=0
while True:
    r.update(); comm()
    if r.goal=='1' or goal_seen:
        goal_seen=True
        L('AT GOAL. waiting for peer. pos',pos)
        r.wheels(0,0)
        while True:
            r.update(); comm()
            time.sleep(0.3)
    clear=sense_clear()
    # neighbor candidates
    cands=[]
    for ax in AXES:
        v=DIRV[ax]; nc=(pos[0]+v[0],pos[1]+v[1])
        if clear[ax]<0.40:
            blocked.add((pos,ax)); continue
        if (pos,ax) in blocked: continue
        if nc not in visited:
            cands.append((clear[ax],ax,nc))
    step+=1
    if cands:
        cands.sort(reverse=True)
        _,ax,nc=cands[0]
        ok,tr=move(ax)
        if ok:
            stack.append(pos); pos=nc; visited.add(pos)
            L('step%d fwd %s -> %s tr=%.2f clear=%s'%(step,ax,pos,tr,{a:round(c,2) for a,c in clear.items()}))
        else:
            blocked.add((pos,ax))
            L('step%d blocked %s at %s tr=%.2f'%(step,ax,pos,tr))
    else:
        if not stack:
            L('explored everything?? resetting visited')
            visited={pos}; blocked=set(); continue
        prev=stack.pop()
        dx=prev[0]-pos[0]; dy=prev[1]-pos[1]
        ax=[a for a,v in DIRV.items() if v==(dx,dy)][0]
        ok,tr=move(ax)
        if ok:
            pos=prev
            L('step%d back %s -> %s'%(step,ax,pos))
        else:
            L('step%d BACKTRACK FAIL %s at %s tr=%.2f'%(step,ax,pos,tr))
            # try to recover: recenter by nudging
