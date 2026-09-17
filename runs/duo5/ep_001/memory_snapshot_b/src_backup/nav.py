import sys,time,math,json,re,ast,collections
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff

r=Robot(); d=Drive(r)
log=open('/memory/run.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== nav start ===')

AXES=[5,95,185,275]
DIRV={5:(1,0),95:(0,1),185:(-1,0),275:(0,-1)}

# build prior graph from wallrun logs
open_e=collections.defaultdict(set)   # cell -> set(neighbor)
blocked_e=set()                        # (cell,nbr)
visited=set()
for line in open('/memory/run.log'):
    m=re.search(r'w\d+ (\d+) tr=[\d.]+ pos (-?\d+),(-?\d+) c=(\{.*\})',line)
    if not m: continue
    ax=int(m.group(1)); x,y=int(m.group(2)),int(m.group(3))
    dv=DIRV[ax]; px,py=x-dv[0],y-dv[1]
    c=ast.literal_eval(m.group(4))
    visited.add((x,y)); visited.add((px,py))
    for a,(ux,uy) in DIRV.items():
        n=(px+ux,py+uy)
        if c.get(a,0)>0.58:
            open_e[(px,py)].add(n); open_e[n].add((px,py))
        elif c.get(a,3)<0.50:
            blocked_e.add(((px,py),n)); blocked_e.add((n,(px,py)))

pos=(4,-2)
while r.h is None or r.rays is None:
    r.update(); time.sleep(0.05)

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
        r.tx.write('A pos %d %d goal %d'%(pos[0],pos[1],1 if goal_seen else 0))
        last_tx=time.time()

def hold_goal():
    L('AT GOAL, holding at',pos)
    r.wheels(0,0)
    while True:
        poll(); time.sleep(0.2)

def bfs(start,targets,graph_open):
    # BFS over edges considered open
    q=[start]; par={start:None}
    while q:
        cur=q.pop(0)
        if cur in targets:
            path=[]
            while cur is not None: path.append(cur); cur=par[cur]
            return path[::-1]
        for n in graph_open.get(cur,()):
            if n not in par and (cur,n) not in blocked_e:
                par[n]=cur; q.append(n)
    return None

def frontiers():
    f=set()
    for cell,nbrs in open_e.items():
        for n in nbrs:
            if n not in visited and (cell,n) not in blocked_e: f.add(n)
    return f

step=0
while True:
    poll()
    if goal_seen: hold_goal()
    c=clearance()
    # update map at pos
    for a,(ux,uy) in DIRV.items():
        n=(pos[0]+ux,pos[1]+uy)
        if c[a]>0.58:
            open_e[pos].add(n); open_e[n].add(pos)
            blocked_e.discard((pos,n)); blocked_e.discard((n,pos))
        elif c[a]<0.50:
            blocked_e.add((pos,n)); blocked_e.add((n,pos))
            open_e[pos].discard(n); open_e[n].discard(pos)
    visited.add(pos)
    f=frontiers()
    if not f:
        L('NO FRONTIERS LEFT. map size',len(visited))
        # fall back: reset visited to re-explore
        visited={pos}
        continue
    path=bfs(pos,f,open_e)
    if not path or len(path)<2:
        L('no path to frontier from',pos,'frontiers',sorted(f)[:6])
        visited={pos}; continue
    nxt=path[1]
    ax=[a for a,v in DIRV.items() if (pos[0]+v[0],pos[1]+v[1])==nxt][0]
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,reason=d.forward(0.5,target_h=ax,front_stop=0.23,speed=75)
    step+=1
    if tr>0.28:
        pos=nxt
        L('n%d %s -> %s tgt=%s c=%s'%(step,ax,pos,path[-1],{a:round(x,2) for a,x in c.items()}))
    else:
        blocked_e.add((pos,nxt)); blocked_e.add((nxt,pos))
        open_e[pos].discard(nxt); open_e[nxt].discard(pos)
        L('n%d %s SHORT tr=%.2f at %s'%(step,ax,tr,pos))
