import sys,time,math,json,os
sys.path.insert(0,'/bot/src')
from nav import *
LOG=open('/memory/explore_log.txt','a')
def log(*a):
    s=time.strftime('%H:%M:%S')+' '+' '.join(str(x) for x in a)
    print(s,flush=True); LOG.write(s+'\n'); LOG.flush()
DIRS={0:(0,1),90:(1,0),180:(0,-1),270:(-1,0)}
def status():
    s=readline(3); return dict(kv.split('=') for kv in s.split()) if s else {}
def sig():
    s=readline(11); return float(s) if s else None
def openings(view):
    return {b:(d>0.42 or d<0) for b,d in view.items()}   # -1 (dropout) treated as far
def recenter(bearing):
    """if too close to a side wall, sidestep toward the middle"""
    view,h,sc=cardinal_view()
    r=view[(bearing+90)%360]; l=view[(bearing+270)%360]
    if r<0 or l<0 or r>0.6 or l>0.6: return
    off=(r-l)/2            # + => need to move right (toward +90)
    if abs(off)<0.07: return
    ang=35 if off>0 else -35
    turn_to((bearing+ang)%360,tol=3)
    dist=abs(off)/math.sin(math.radians(35))
    # drive forward dist using the bearing+ang front beam
    v2,h2,sc2=cardinal_view()
    f=sc2[0]
    if 0<f<SAT: drive_front(max(0.15,f-dist),(bearing+ang)%360,maxt=15)
    else: drive_blind((bearing+ang)%360, dist/0.13, spd=40)
    turn_to(bearing,tol=3)
    v3,h3,sc3=cardinal_view()
    log(f"recenter: off={off:+.2f} -> r={v3[(bearing+90)%360]:.2f} l={v3[(bearing+270)%360]:.2f}")
def step(bearing):
    """move one cell in direction bearing. returns True if moved."""
    turn_to(bearing,tol=3)
    recenter(bearing)
    view,h,sc=cardinal_view()
    f=view[bearing]
    if 0<f<0.42: log("blocked toward",bearing,"f=",f); return False
    if 0<f<SAT:
        target=f-0.5
        if target<0.15: target=max(0.15,f-0.5)
        ok,f2=drive_front(target,bearing)
        log(f"step {bearing}: front {f:.2f}->{f2:.2f} ok={ok}")
        if not ok and abs(f2-target)>0.2: log("step failed, not counting"); return False
        if 0<f2<SAT:
            k=round((f2-0.25)/0.5); snap=0.25+0.5*k
            if abs(snap-f2)>0.05 and snap>=0.15:
                ok2,f3=drive_front(snap,bearing,maxt=15); log(f"snap {f2:.2f}->{f3:.2f}")
    else:
        drive_blind(bearing,3.8,spd=40)
        f2=cardinal_view()[0][bearing]
        # snap to a cell center if a wall is visible now
        if 0<f2<SAT:
            k=round((f2-0.25)/0.5); target=0.25+0.5*k
            if abs(target-f2)>0.06 and target>=0.15:
                drive_front(target,bearing); f2=cardinal_view()[0][bearing]
        log(f"blind step {bearing}: now front {f2:.2f}")
    return True
def frontier_route(pos,cells,visited):
    from collections import deque
    prev={pos:None}; q=deque([pos])
    while q:
        c=q.popleft()
        info=cells.get(str(c))
        if info is None: continue
        for b in info['open']:
            n=(c[0]+DIRS[b][0],c[1]+DIRS[b][1])
            if n not in visited:
                # reconstruct path to c, then b
                path=[b]; x=c
                while prev[x] is not None:
                    x,bb=prev[x]; path.append(bb)
                return path[::-1]
            if n not in prev: prev[n]=(c,b); q.append(n)
    return None
def main(maxsteps=60, start=(0,0), route=(), previsited=()):
    pos=start
    for b in route:
        if step(b): pos=(pos[0]+DIRS[b][0],pos[1]+DIRS[b][1]); log("routed to",pos)
        else: log("route blocked at",pos,"toward",b); break
    visited=set(map(tuple,previsited)); visited.add(pos); stack=[]; cells={}
    if os.path.exists('/memory/map.json'):
        try: cells=json.load(open('/memory/map.json'))['cells']; log("loaded",len(cells),"cells")
        except Exception as e: log("map load failed",e)
    for n in range(maxsteps):
        turn_to(min((0,90,180,270,360),key=lambda b:abs(angdiff(b,hd(3)))) % 360, tol=3)
        view,h,sc=cardinal_view()
        op=openings(view); st=status(); sg=sig()
        cells[str(pos)]={'view':{k:round(v,2) for k,v in view.items()},'open':[b for b in op if op[b]]}
        log(f"AT {pos} h={h:.0f} view={cells[str(pos)]['view']} open={cells[str(pos)]['open']} status={st} sig={sg}")
        json.dump({'pos':pos,'cells':cells},open('/memory/map.json','w'))
        if st.get('goal')=='1': log("GOAL REACHED at",pos); stop(); return
        # choose unvisited open neighbor
        nxt=None
        for b in (90,0,270,180):
            if op[b]:
                q=(pos[0]+DIRS[b][0],pos[1]+DIRS[b][1])
                if q not in visited: nxt=(b,q); break
        if nxt is None and not stack:
            route=frontier_route(pos,cells,visited)
            if not route: log("exploration exhausted"); return
            log("routing to frontier via",route)
            for b in route[:-1]:
                if step(b): pos=(pos[0]+DIRS[b][0],pos[1]+DIRS[b][1])
                else: log("frontier route blocked"); break
            continue
        if nxt is None:
            back=stack.pop()   # bearing we came from -> go back
            b=(back+180)%360; q=(pos[0]+DIRS[b][0],pos[1]+DIRS[b][1])
            if step(b): pos=q
            continue
        b,q=nxt
        if step(b):
            stack.append(b); pos=q; visited.add(q)
        else:
            visited.add(q)  # mark as blocked
    log("maxsteps reached")
if __name__=='__main__':
    import ast
    n=int(sys.argv[1]) if len(sys.argv)>1 else 60
    start=ast.literal_eval(sys.argv[2]) if len(sys.argv)>2 else (0,0)
    route=ast.literal_eval(sys.argv[3]) if len(sys.argv)>3 else ()
    prev=ast.literal_eval(sys.argv[4]) if len(sys.argv)>4 else ()
    main(n,start,route,prev)
