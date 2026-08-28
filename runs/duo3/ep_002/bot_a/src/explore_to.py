import sys, time, json
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import stop, motors
from mouse import Bot, walls_here, rd6, goal_check, load_map, save_map, DIRS, scan, log
from seek2 import open_dirs, bfs

def recenter(bot):
    # if wall ahead within 0.55, snap distance to 0.23
    l=scan(2)
    if l and 0<l[0]<0.55:
        d=l[0]-0.23
        if abs(d)>0.04:
            sp=90 if d>0 else -90
            dur=abs(d)/ (0.0026*90)
            motors(sp,sp); time.sleep(min(dur,1.5)); stop()

def frontiers(M):
    out=[]
    for cell in M:
        for D,v in open_dirs(M,cell):
            if v not in M:
                out.append((cell,D,v))
    return out

def main():
    x,y=int(sys.argv[1]),int(sys.argv[2])
    tx,ty=int(sys.argv[3]),int(sys.argv[4])
    dur=float(sys.argv[5]) if len(sys.argv)>5 else 240
    t_end=time.time()+dur
    M=load_map()
    bot=Bot(x,y)
    c=compass()
    while c is None: c=compass()
    h=min(DIRS, key=lambda d: abs(((d-c+540)%360)-180))
    bot.align(h)
    recenter(bot)
    while time.time()<t_end:
        if goal_check(): print("GOAL!",flush=True); break
        key=(bot.x,bot.y)
        if key not in M:
            w,L=walls_here(bot.h); s=rd6()
            M[key]={'w':{str(k):v for k,v in w.items()},'s':s}
            log(cell=[bot.x,bot.y], w=M[key]['w'], s=s); save_map(M)
            print("new cell",key,"s=",s,flush=True)
            write_port("d8", f"PING A cell={key} s={s}")
        if key==(tx,ty):
            print("reached target cell",key,flush=True); break
        F=frontiers(M)
        if not F:
            print("no frontiers",flush=True); break
        # choose frontier minimizing manhattan dist to target
        F.sort(key=lambda f: abs(f[2][0]-tx)+abs(f[2][1]-ty))
        cell,D,v=F[0]
        path=bfs(M,key,cell) or []
        path.append(D)
        moved=False
        for D2 in path:
            if time.time()>t_end or goal_check(): break
            bot.face(D2)
            l=scan(2)
            if l and 0<l[0]<0.3:
                # unexpected wall; update and replan
                M[(bot.x,bot.y)]['w'][str(D2)]=True; save_map(M)
                print("wall found at",(bot.x,bot.y),D2,flush=True)
                break
            if not bot.step():
                M[(bot.x,bot.y)]['w']=({str(k):vv for k,vv in (walls_here(bot.h)[0] or {}).items()})
                save_map(M); break
            moved=True
        if not moved and not path: break
    stop(); log(event='EXPTO_END', cell=[bot.x,bot.y])
    print("end at",bot.x,bot.y,flush=True)

if __name__=='__main__':
    main()
