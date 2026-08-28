import sys, time, json
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import stop
import mouse
from mouse import Bot, walls_here, rd6, goal_check, load_map, save_map, DIRS, scan, log

def open_dirs(M, cell):
    out=[]
    if cell in M:
        for D,(dx,dy) in DIRS.items():
            if not M[cell]['w'][str(D)]:
                out.append((D,(cell[0]+dx,cell[1]+dy)))
    return out

def bfs(M, src, dst):
    from collections import deque
    q=deque([src]); par={src:None}
    while q:
        u=q.popleft()
        if u==dst: break
        for D,v in open_dirs(M,u):
            if v not in par and v in M:
                par[v]=(u,D); q.append(v)
        # allow stepping into unknown cell only if it's dst
        for D,v in open_dirs(M,u):
            if v not in par and v==dst:
                par[v]=(u,D); q.append(v)
    if dst not in par: return None
    path=[]
    cur=dst
    while par[cur]:
        u,D=par[cur]; path.append(D); cur=u
    return path[::-1]

def ensure_cell(bot,M):
    key=(bot.x,bot.y)
    if key not in M:
        w,L=walls_here(bot.h)
        s=rd6()
        M[key]={'w':{str(k):v for k,v in w.items()}, 's':s}
        log(cell=[bot.x,bot.y], w=M[key]['w'], s=s)
        save_map(M)
    else:
        s=rd6()
        if s is not None:
            M[key]['s']=max(M[key]['s'] or 0, 0) * 0 + s
            save_map(M)
    return M[key]

def main():
    x,y=int(sys.argv[1]),int(sys.argv[2])
    dur=float(sys.argv[3]) if len(sys.argv)>3 else 180
    t_end=time.time()+dur
    M=load_map()
    bot=Bot(x,y)
    # align to nearest cardinal
    c=compass()
    while c is None: c=compass()
    h=min(DIRS, key=lambda d: abs(((d-c+540)%360)-180))
    bot.align(h)
    ensure_cell(bot,M)
    # go to best-s cell first
    best=max((k for k in M if M[k].get('s')), key=lambda k: M[k]['s'])
    print("navigating to", best, M[best]['s'], flush=True)
    path=bfs(M,(bot.x,bot.y),best)
    if path:
        for D in path:
            if time.time()>t_end or goal_check(): break
            bot.face(D); bot.step()
            write_port("d8", f"PING A cell={bot.x},{bot.y}")
    # hill climb
    visits={}
    while time.time()<t_end:
        if goal_check():
            print("GOAL!", flush=True); break
        key=(bot.x,bot.y)
        visits[key]=visits.get(key,0)+1
        info=ensure_cell(bot,M)
        s_here=info['s']
        print("at",key,"s=",s_here, flush=True)
        write_port("d8", f"PING A cell={key[0]},{key[1]} s={s_here}")
        # live rescan of walls (trust fresh)
        w,L=walls_here(bot.h)
        if w: M[key]['w']={str(k):v for k,v in w.items()}; save_map(M)
        cands=[]
        for D,v in open_dirs(M,key):
            sv = M.get(v,{}).get('s')
            unknown = v not in M
            cands.append((D,v,sv,unknown))
        if not cands:
            print("boxed in?!", flush=True); break
        # prefer unmeasured neighbors, then highest s; penalize visits
        def score(c):
            D,v,sv,unknown=c
            base = 0.6 if unknown else (sv if sv is not None else 0.3)
            return base - 0.05*visits.get(v,0)
        cands.sort(key=score, reverse=True)
        D,v,sv,unknown=cands[0]
        bot.face(D)
        l=scan(2)
        if l and 0<l[0]<0.3:
            M[key]['w'][str(D)]=True; save_map(M); continue
        bot.step()
    stop()
    log(event='SEEK_END', cell=[bot.x,bot.y])
    print("end at", bot.x,bot.y, flush=True)

if __name__=='__main__':
    main()
