from cell import *
import json, sys
from collections import deque
DIRS={0:(1,0),90:(0,1),180:(-1,0),270:(0,-1)}; NM={0:'N',90:'E',180:'S',270:'W'}
m=json.load(open('/memory/map.json')); pos=tuple(m['pos']); cells={tuple(json.loads(k)):v for k,v in m['cells'].items()}
def save(): json.dump({'pos':list(pos),'path':[],'cells':{json.dumps(list(k)):v for k,v in cells.items()}},open('/memory/map.json','w'))
def nb(c,w): return (c[0]+DIRS[w][0],c[1]+DIRS[w][1])
def path_to(start,goal):
    prev={start:None}; q=deque([start])
    while q:
        c=q.popleft()
        if c==goal: break
        for w in (90,0,180,270):
            if not cells.get(c,{}).get(str(w)): continue
            n=nb(c,w)
            if n in cells and n not in prev: prev[n]=(c,w); q.append(n)
    if goal not in prev: return None
    p=[]; x=goal
    while prev[x]: p.append(prev[x][1]); x=prev[x][0]
    return p[::-1]
def sense():
    wd=walls_here(); return {str(w):(wd[w]>0.4) for w in wd}
def mv(w):
    global pos
    trav,reason=step(w)
    if trav>0.3:
        pos=nb(pos,w)
        cells[pos]=sense(); log(f'start {pos} {cells[pos]}')
        cells[pos][str((w+180)%360)]=True; save(); return True
    log(f"  move {w} from {pos} FAILED {reason}; resensing")
    cells[pos]=sense(); cells[pos][str(w)]=False; save(); return False
def listen(t=15):
    out=[]; t0=time.time()
    while time.time()-t0<t:
        m=rd('d10',1.0)
        if m: out.append(m); log("RX: "+m)
        time.sleep(0.3)
    return out
sigmap={}; tried=set()
cells[pos]=sense(); log(f'start {pos} {cells[pos]}')
for it in range(60):
    s=sig(); sigmap[pos]=s; st=rd('d3')
    log(f"[S{it}] at {pos} sig {s:.3f} {st}")
    if s>0.85:
        wr('d8',f"R1 here, close to you (sig {s:.2f}). I know the goal route. Stay still 20s and tell me which compass dirs are open around you.")
        if listen(20): log("GOT REPLY, stopping"); break
    cands=[w for w in (0,90,180,270) if cells[pos].get(str(w)) and (pos,w) not in tried]
    cands.sort(key=lambda w: (nb(pos,w) in cells, -sigmap.get(nb(pos,w),0)))
    improved=False
    for w in cands:
        tried.add((pos,w))
        if not mv(w): continue
        s2=sig(); sigmap[pos]=s2; log(f"  {NM[w]} -> {pos} sig {s2:.3f}")
        if s2>s-0.01: improved=True; break
        mv((w+180)%360)
    if not improved:
        # frontier openings: (cell, dir) with open wall into unknown cell, not tried
        fr=[(c,w) for c in cells for w in (0,90,180,270) if cells[c].get(str(w)) and nb(c,w) not in cells and (c,w) not in tried]
        if not fr: log("no frontier"); break
        def score(cw):
            p=path_to(pos,cw[0]); return (sigmap.get(cw[0],0.45) - 0.02*(len(p) if p else 99))
        c,w=max(fr,key=score); p=path_to(pos,c); log(f"  frontier {c} {NM[w]} via {p}")
        if p is None: tried.add((c,w)); continue
        ok=True
        for u in p:
            if not mv(u): ok=False; break
        if ok:
            tried.add((c,w)); mv(w)
stop(); save(); log("SHEPHERD END")
