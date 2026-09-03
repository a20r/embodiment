# appended frontier explorer
import json, collections, random as _rnd

MAPF='/tmp/map.json'
MAP={'pos':[0,0],'cells':{}}
def loadmap():
    global MAP
    try: MAP=json.load(open(MAPF))
    except: pass
def savemap():
    try: json.dump(MAP,open(MAPF,'w'))
    except: pass
DIRS={0:(1,0),90:(0,1),180:(-1,0),270:(0,-1)}
BACK={0:180,90:270,180:0,270:90}
def ckey(x,y): return f"{x},{y}"
def sense_cell():
    w=walls_here()
    if not w: return None
    x,y=MAP['pos']
    c=MAP['cells'].setdefault(ckey(x,y),{})
    for b in (0,90,180,270):
        st='open' if w[b]>0.50 else 'wall'
        if c.get(str(b))!='fail':
            c[str(b)]=st
    return w
def edge_open(x,y,b):
    c=MAP['cells'].get(ckey(x,y),{})
    s=c.get(str(b))
    if s=='fail' or s=='wall': return False
    if s=='open': return True
    # unknown from this side: check other side
    d=DIRS[b]; oc=MAP['cells'].get(ckey(x+d[0],y+d[1]),{})
    os_=oc.get(str(BACK[b]))
    return os_=='open'
def bfs_to_frontier(prefer_far_from=None):
    # returns list of bearings to walk; target = visited cell w/ open edge to unknown cell, then step through it
    start=tuple(MAP['pos'])
    q=collections.deque([start]); prev={start:None}
    results=[]
    while q:
        cur=q.popleft()
        cc=MAP['cells'].get(ckey(*cur),{})
        for b in (0,90,180,270):
            if not edge_open(cur[0],cur[1],b): continue
            d=DIRS[b]; nxt=(cur[0]+d[0],cur[1]+d[1])
            if ckey(*nxt) not in MAP['cells']:
                results.append((cur,b))
                if len(results)>=1 and prefer_far_from is None:
                    q.clear(); break
            elif nxt not in prev:
                prev[nxt]=(cur,b); q.append(nxt)
        if results and prefer_far_from is None: break
    if not results: return None
    cur,b=results[0]
    path=[]
    node=cur
    while prev[node] is not None:
        p,pb=prev[node]; path.append(pb); node=p
    path.reverse(); path.append(b)
    return path
def fexplore(secs=9999):
    t0=time.time()
    loadmap()
    lastsave=0
    while not stop_flag and time.time()-t0<secs:
        if goalflag()=='1':
            flog('GOAL (fexplore)')
            with open('/tmp/grid.log','a') as g: g.write(f"GOAL at map {MAP['pos']}\n")
            savemap(); spin_at_goal(); return
        if heard_goal():
            flog('fexplore: heard goal, climbing'); savemap(); climb(300); continue
        sense_cell()
        path=bfs_to_frontier()
        if path is None:
            flog('fexplore: no frontier! wandering')
            gofar(_rnd.choice((0,90,180,270)),60); 
            # position now unknown-ish; keep going with same map pos (gofar updates gridpos only)
            continue
        if len(path)>1:
            flog(f"fexplore: walk {len(path)} steps to frontier from {MAP['pos']}")
        for b in path:
            if stop_flag or time.time()-t0>secs: break
            if goalflag()=='1': break
            recenter()
            if move_cell(b):
                d=DIRS[b]; MAP['pos'][0]+=d[0]; MAP['pos'][1]+=d[1]
                gridpos[0],gridpos[1]=MAP['pos']
                sense_cell()
                with open('/tmp/grid.log','a') as g: g.write(f"{time.time():.0f} fx {MAP['pos']} n={len(MAP['cells'])}\n")
            else:
                x,y=MAP['pos']
                MAP['cells'].setdefault(ckey(x,y),{})[str(b)]='fail'
                flog(f'fexplore: move fail at {MAP["pos"]} b={b}')
                recenter()
                break
        if time.time()-lastsave>10:
            savemap(); lastsave=time.time()
    savemap(); motors(0,0)
def brain2():
    time.sleep(1)
    tx("B proto: goal-finder parks+spins+broadcasts GOAL FOUND. other homes on d5 sound climb. I run frontier explore.")
    while not stop_flag:
        if goalflag()=='1': spin_at_goal(); return
        if heard_goal():
            flog('BRAIN2: heard goal 1 -> climb'); climb(240)
        else:
            fexplore(120)
        time.sleep(0.3)
