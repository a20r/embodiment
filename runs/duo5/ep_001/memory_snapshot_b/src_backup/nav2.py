import sys,time,math,json,re,ast,collections
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff

r=Robot(); d=Drive(r)
log=open('/memory/run.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== nav2 start ===')

AXES=[5,95,185,275]
DIRV={5:(1,0),95:(0,1),185:(-1,0),275:(0,-1)}
OPEN_T=0.60; BLK_T=0.45

open_e=collections.defaultdict(set)
blocked=set()   # frozenset pairs
unknown=set()
tested_blocked=set()
visited=set()
def edge(a,b): return frozenset((a,b))

def parse_prior():
    pats=[r'w\d+ (\d+) tr=[\d.]+ pos (-?\d+),(-?\d+) c=(\{.*\})',
          r'n\d+ (\d+) -> \((-?\d+), (-?\d+)\) tgt=.* c=(\{.*\})']
    for line in open('/memory/run.log'):
        for p in pats:
            m=re.search(p,line)
            if not m: continue
            ax=int(m.group(1)); x,y=int(m.group(2)),int(m.group(3))
            dv=DIRV[ax]; px,py=x-dv[0],y-dv[1]
            c=ast.literal_eval(m.group(4))
            visited.add((x,y)); visited.add((px,py))
            for a,(ux,uy) in DIRV.items():
                n=(px+ux,py+uy); e=edge((px,py),n)
                v=c.get(a)
                if v is None: continue
                if v>OPEN_T:
                    open_e[(px,py)].add(n); open_e[n].add((px,py))
                elif v<BLK_T: blocked.add(e)
                else: unknown.add(e)
            # traversed edge is definitely open
            open_e[(px,py)].add((x,y)); open_e[(x,y)].add((px,py))
parse_prior()
unknown-={e for e in unknown if e in blocked}
for cell,ns in open_e.items():
    for n in ns: unknown.discard(edge(cell,n))
L('prior: %d cells, %d open cells, %d unknown edges'%(len(visited),len(open_e),len(unknown)))

pos=(0,2)
while r.h is None or r.rays is None:
    r.update(); time.sleep(0.05)

def clearance():
    best={ax:0.0 for ax in AXES}
    aligned={ax:0.0 for ax in AXES}
    for s in range(3):
        r.update()
        for ax in AXES:
            rel=((ax-r.h)%360)/22.5
            k0=int(rel)%16; k1=(k0+1)%16
            vals=[v for v in (r.ray(k0),r.ray(k1)) if v is not None]
            if vals: best[ax]=max(best[ax],min(vals))
            ka=int(round(rel))%16
            va=r.ray(ka)
            if va is not None: aligned[ax]=max(aligned[ax],va)
        time.sleep(0.07)
    return best,aligned

goal_seen=False; last_tx=0; last_probe=0
def poll():
    global goal_seen,last_tx
    r.update()
    global last_probe
    if r.msgs:
        for m in r.msgs: L('RX:',m)
        if time.time()-last_probe>10:
            for pm in ('R1 hello','R1 where is goal','R1 follow me','A goal 0 seek'):
                r.tx.write(pm)
            last_probe=time.time()
    r.msgs[:]=[]
    for e in r.events:
        if 'goal=0' not in e: L('EV:',e)
        if 'goal=1' in e: goal_seen=True
    r.events[:]=[]
    if time.time()-last_tx>2:
        r.tx.write('A pos %d %d goal %d'%(pos[0],pos[1],1 if goal_seen else 0))
        last_tx=time.time()

def bfs_to(targets):
    # targets: set of cells; returns path over open_e
    if pos in targets: return [pos]
    q=[pos]; par={pos:None}
    while q:
        cur=q.pop(0)
        for n in open_e.get(cur,()):
            if n in par: continue
            par[n]=cur
            if n in targets:
                path=[n]
                c=cur
                while c is not None: path.append(c); c=par[c]
                return path[::-1]
            q.append(n)
    return None

def sense_update():
    c,ca=clearance()
    for a,(ux,uy) in DIRV.items():
        n=(pos[0]+ux,pos[1]+uy); e=edge(pos,n)
        if c[a]>OPEN_T:
            open_e[pos].add(n); open_e[n].add(pos)
            blocked.discard(e); unknown.discard(e)
        elif ca[a]>OPEN_T:
            # possible doorway: physically test later
            if n not in open_e.get(pos,set()) and e not in tested_blocked:
                unknown.add(e); blocked.discard(e)
        elif c[a]<BLK_T:
            if e not in unknown or e in tested_blocked:
                blocked.add(e); unknown.discard(e)
                open_e[pos].discard(n); open_e[n].discard(pos)
        else:
            if n not in open_e.get(pos,set()) and e not in blocked and e not in tested_blocked:
                unknown.add(e)
    visited.add(pos)
    return c

def try_move(nxt):
    global pos
    ax=[a for a,v in DIRV.items() if (pos[0]+v[0],pos[1]+v[1])==nxt][0]
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,reason=d.forward(0.5,target_h=ax,front_stop=0.23,speed=75)
    e=edge(pos,nxt)
    if tr>0.28:
        pos=nxt
        open_e[pos].add(nxt); # no-op safe
        return True,tr
    else:
        blocked.add(e); unknown.discard(e); tested_blocked.add(e)
        open_e[pos].discard(nxt); open_e[nxt].discard(pos)
        return False,tr

step=0
while True:
    poll()
    if goal_seen:
        L('AT GOAL, holding at',pos)
        r.wheels(0,0)
        while True:
            poll(); time.sleep(0.2)
    c=sense_update()
    # 1) frontier cells: unvisited neighbor via open edge
    fr=set()
    for cell,ns in open_e.items():
        for n in ns:
            if n not in visited: fr.add(n)
    path=bfs_to(fr) if fr else None
    if path is None and unknown:
        # 2) go test an unknown edge: target = its endpoint cells that are visited
        ends=set()
        for e in unknown:
            for cell in e:
                if cell in visited: ends.add(cell)
        path=bfs_to(ends)
        if path is not None and len(path)==1:
            # we are at an endpoint: attempt crossing
            here=path[0]
            for e in list(unknown):
                if here in e:
                    other=[c2 for c2 in e if c2!=here][0]
                    ok,tr=try_move(other)
                    step+=1
                    L('u%d test %s->%s ok=%s tr=%.2f'%(step,here,other,ok,tr))
                    break
            continue
    if path is None:
        L('NOTHING LEFT: visited=%d unknown=%d. wander reset.'%(len(visited),len(unknown)))
        visited={pos}
        time.sleep(1)
        continue
    if len(path)<2:
        time.sleep(0.2); continue
    nxt=path[1]
    ok,tr=try_move(nxt)
    step+=1
    L('n%d ->%s ok=%s tr=%.2f tgt=%s d5=%s c=%s'%(step,pos,ok,tr,path[-1],r.d5.last,{a:round(x,2) for a,x in c.items()}))
