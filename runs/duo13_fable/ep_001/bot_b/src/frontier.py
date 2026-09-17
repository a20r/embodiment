from cell import *
import json, sys
from collections import deque
DIRS={0:(1,0),90:(0,1),180:(-1,0),270:(0,-1)}
N=int(sys.argv[1]) if len(sys.argv)>1 else 40
m=json.load(open('/memory/map.json')); pos=tuple(m['pos']); cells={tuple(json.loads(k)):v for k,v in m['cells'].items()}
def save(): json.dump({'pos':list(pos),'path':[],'cells':{json.dumps(list(k)):v for k,v in cells.items()}},open('/memory/map.json','w'))
def nb(c,w): return (c[0]+DIRS[w][0],c[1]+DIRS[w][1])
def plan(start, goal=None):
    """BFS; if goal None -> nearest frontier (returns dirs incl. final move into unknown)"""
    prev={start:None}; q=deque([start])
    while q:
        c=q.popleft()
        if goal is not None and c==goal: break
        for w in (90,0,180,270):
            if not cells.get(c,{}).get(str(w)): continue
            n=nb(c,w)
            if n not in cells:
                if goal is None:
                    path=[w]; x=c
                    while prev[x] is not None: path.append(prev[x][1]); x=prev[x][0]
                    return path[::-1]
                continue
            if n not in prev: prev[n]=(c,w); q.append(n)
    if goal is not None and goal in prev:
        path=[]; x=goal
        while prev[x] is not None: path.append(prev[x][1]); x=prev[x][0]
        return path[::-1]
    return None
def sense():
    wd=walls_here(); return {str(w):(wd[w]>0.4) for w in wd}, wd
k=0
while k<N:
    if pos not in cells:
        cells[pos],wd=sense()
    path=plan(pos)
    st=rd('d3'); s=sig()
    log(f"[{k}] at {pos} sig {s:.3f} {st} plan {path}")
    if st and ('goal=0' not in st or 'here=0' not in st): log('!!! GOAL '+st); save(); break
    if not path: log('no frontier left'); break
    for w in path:
        k+=1
        trav,reason=step(w)
        if trav>0.3:
            pos=nb(pos,w)
            if pos not in cells:
                cells[pos],wd=sense()
                log(f"  new {pos} N{wd[0]:.2f} E{wd[90]:.2f} S{wd[180]:.2f} W{wd[270]:.2f} sig {sig():.3f} {rd('d3')}")
            # consistency: wall behind us must be open
            cells[pos][str((w+180)%360)]=True
        else:
            cells[pos][str(w)]=False
            log(f"  move {w} from {pos} FAILED trav={trav:.2f} {reason}"); break
        save()
        st=rd('d3')
        if st and ('goal=0' not in st or 'here=0' not in st): log('!!! GOAL '+st); break
stop(); save(); log('END')
