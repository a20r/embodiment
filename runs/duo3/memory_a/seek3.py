import sys, time, json
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import stop
from mouse import walls_here, rd6, DIRS, scan, log
from mouse3 import Bot3, sensors_hot

def savg(n=4):
    vs=[v for v in (rd6() for _ in range(n)) if v is not None]
    return sum(vs)/len(vs) if vs else None

def main():
    dur=float(sys.argv[1]) if len(sys.argv)>1 else 600
    t_end=time.time()+dur
    bot=Bot3(0,0)
    c=compass()
    while c is None: c=compass()
    bot.align(min(DIRS,key=lambda d: abs(((d-c+540)%360)-180)))
    M={}   # cell -> walls dict
    S={}   # cell -> (t,s)
    fails={}
    hold_until=0
    while time.time()<t_end:
        if sensors_hot(): break
        key=(bot.x,bot.y)
        s=savg()
        if s is not None: S[key]=(time.time(),s)
        write_port("d8", f"B homing on your signal s={round(s,2) if s else s}. HOLD STILL pls")
        if key not in M:
            w,_=walls_here(bot.h)
            if w is None: continue
            M[key]={str(k):v for k,v in w.items()}
        print("at",key,"s=",round(s,3) if s else s,flush=True)
        if s and s>1.25:
            print("ADJACENT: holding",flush=True)
            stop()
            for i in range(8):
                write_port("d8","B ADJACENT to you now. holding. check goal state?")
                time.sleep(1.5)
                if sensors_hot(): return
                s2=savg()
                if s2 and s2<1.0: break
            continue
        # candidates: visited cells with open-unknown dirs
        best=None
        from collections import deque
        # BFS distances from current over known open edges
        dist={key:0}; q=deque([key]); par={key:None}
        while q:
            u=q.popleft()
            if u not in M: continue
            for D,(dx,dy) in DIRS.items():
                if not M[u][str(D)]:
                    v=(u[0]+dx,u[1]+dy)
                    if v not in dist:
                        dist[v]=dist[u]+1; par[v]=(u,D)
                        if v in M: q.append(v)
        now=time.time()
        for u in list(M.keys()):
            if u not in dist: continue
            opens=[D for D in DIRS if not M[u][str(D)] and (u[0]+DIRS[D][0],u[1]+DIRS[D][1]) not in M and fails.get((u,D),0)<2]
            if not opens: continue
            ts,sv=S.get(u,(0,0.1))
            sc=(sv or 0.1) - 0.004*(now-ts) - 0.03*dist[u]
            if best is None or sc>best[0]: best=(sc,u,opens)
        if best is None:
            print("no candidates; wandering",flush=True)
            w,_=walls_here(bot.h)
            opts=[D for D in DIRS if w and not w[D]]
            if opts:
                import random
                D=random.choice(opts); bot.face(D); bot.step()
            continue
        _,u,opens=best
        # route to u
        pathd=[]
        cur=u
        while par.get(cur):
            p,D=par[cur]; pathd.append(D); cur=p
        pathd=pathd[::-1]
        for D in pathd:
            if time.time()>t_end or sensors_hot(): return
            bot.face(D)
            l=scan(2)
            if l and 0<l[0]<0.33:
                M[(bot.x,bot.y)][str(D)]=True; break
            if not bot.step():
                fails[((bot.x-DIRS[D][0],bot.y-DIRS[D][1]),D)]=9; break
            sm=rd6()
            if sm: S[(bot.x,bot.y)]=(time.time(),sm)
        if (bot.x,bot.y)!=u: continue
        # step into unknown
        D=opens[0]
        bot.face(D)
        l=scan(2)
        if l and 0<l[0]<0.33:
            M[u][str(D)]=True; continue
        if not bot.step():
            fails[(u,D)]=fails.get((u,D),0)+1
    stop()
    print("seek3 end",flush=True)

if __name__=='__main__':
    main()
