from cell import *
import json, sys
DIRS={0:(1,0),90:(0,1),180:(-1,0),270:(0,-1)}
N=int(sys.argv[1]) if len(sys.argv)>1 else 30
pref=[int(x) for x in sys.argv[2].split(',')] if len(sys.argv)>2 else [270,180,0,90]
state=json.load(open('/memory/map.json')) if len(sys.argv)>3 and sys.argv[3]=='resume' else {'pos':[0,0],'path':[],'cells':{}}
pos=tuple(state['pos']); path=state['path']; cells={tuple(json.loads(k)):v for k,v in state['cells'].items()}
def save():
    json.dump({'pos':list(pos),'path':path,'cells':{json.dumps(list(k)):v for k,v in cells.items()}},open('/memory/map.json','w'))
for k in range(N):
    wd=walls_here(); s=sig(); st=rd('d3')
    log(f"[{k}] cell {pos} walls N{wd[0]:.2f} E{wd[90]:.2f} S{wd[180]:.2f} W{wd[270]:.2f} sig {s:.3f} {st}")
    if pos not in cells: cells[pos]={str(w):(wd[w]>0.4) for w in wd}
    if st and ('goal=0' not in st or 'here=0' not in st): log('!!! GOAL '+st); save(); break
    nxt=None; backtrack=False
    for w in pref:
        nb=(pos[0]+DIRS[w][0],pos[1]+DIRS[w][1])
        if cells[pos][str(w)] and nb not in cells: nxt=w; break
    if nxt is None:
        if not path: log('explored all reachable'); break
        nxt=(path.pop()+180)%360; backtrack=True
    trav,reason=step(nxt)
    if trav>0.3:
        pos=(pos[0]+DIRS[nxt][0],pos[1]+DIRS[nxt][1])
        if not backtrack: path.append(nxt)
    else:
        cells[pos][str(nxt)]=False
        log(f'  move {nxt} FAILED trav={trav:.2f} {reason}')
    log(f'  moved {nxt} trav={trav:.2f} {reason} -> {pos} path={path}')
    save()
stop(); log('END')
