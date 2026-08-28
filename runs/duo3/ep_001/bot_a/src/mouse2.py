import sys, time, json
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import stop, turn_to
from mouse import Bot, walls_here, rd6, goal_check, DIRS, scan, log

MAPF='/memory/map2.json'
def save(M, bot):
    with open(MAPF,'w') as f:
        json.dump({'cells':{f"{k[0]},{k[1]}":v for k,v in M.items()},
                   'pos':[bot.x,bot.y], 'h':bot.h}, f)

def main():
    dur=float(sys.argv[1]) if len(sys.argv)>1 else 300
    t_end=time.time()+dur
    M={}
    bot=Bot(0,0)
    c=compass()
    while c is None: c=compass()
    h=min(DIRS, key=lambda d: abs(((d-c+540)%360)-180))
    bot.align(h)
    path=[]
    last_ping=0
    fails={}
    while time.time()<t_end:
        if goal_check(): print("GOAL!",flush=True); break
        key=(bot.x,bot.y)
        if key not in M:
            w,L=walls_here(bot.h); s=rd6()
            if w is None: continue
            M[key]={'w':{str(k):v for k,v in w.items()},'s':s}
            log(cell=[bot.x,bot.y], w=M[key]['w'], s=s)
            print("cell",key,"s=",s,flush=True)
            save(M,bot)
        if time.time()-last_ping>3:
            write_port("d8", f"PING A cell={bot.x},{bot.y}")
            last_ping=time.time()
        w=M[key]['w']
        # prefer east(0)/north(90) toward suspected beacon, then straight
        order=[0,90,bot.h,(bot.h+90)%360,(bot.h+270)%360,(bot.h+180)%360]
        nxt=None
        for D in order:
            if not w[str(D)]:
                dx,dy=DIRS[D]
                v=(bot.x+dx,bot.y+dy)
                if v not in M and fails.get((key,D),0)<2:
                    nxt=D; break
        if nxt is not None:
            bot.face(nxt)
            l=scan(2)
            if l and 0<l[0]<0.33:
                M[key]['w'][str(nxt)]=True; save(M,bot); continue
            ok=bot.step()
            if not ok:
                # didn't fully make it; count fail, rescan from wherever we are
                fails[(key,nxt)]=fails.get((key,nxt),0)+1
                # assume we did move a cell anyway (step updates pos); rescan walls
                w2,_=walls_here(bot.h)
                if w2: M[(bot.x,bot.y)]={'w':{str(k):v for k,v in w2.items()},'s':rd6()}
                save(M,bot)
                path.append((nxt+180)%360)
            else:
                path.append((nxt+180)%360)
            continue
        if not path:
            print("explored all",flush=True); log(event='EXPLORED_ALL'); break
        D=path.pop()
        bot.face(D)
        bot.step()
        save(M,bot)
    stop(); save(M,bot)
    log(event='END2', cell=[bot.x,bot.y])
    print("end at",bot.x,bot.y,flush=True)

if __name__=='__main__':
    main()
