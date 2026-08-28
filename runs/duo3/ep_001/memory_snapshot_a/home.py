import sys, time, json
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import stop
from mouse import Bot, walls_here, rd6, goal_check, DIRS, scan

def savg(n=6):
    vs=[]
    for _ in range(n):
        v=rd6()
        if v is not None: vs.append(v)
    return sum(vs)/len(vs) if vs else 0

def main():
    dur=float(sys.argv[1]) if len(sys.argv)>1 else 240
    t_end=time.time()+dur
    bot=Bot(0,0)
    c=compass()
    while c is None: c=compass()
    import math
    h=min(DIRS, key=lambda d: abs(((d-c+540)%360)-180))
    bot.align(h)
    s_prev=savg()
    last_dir=None
    while time.time()<t_end:
        if goal_check(): print("GOAL!",flush=True); break
        write_port("d8","H HOLD STILL homing on you")
        s=savg()
        print("s=",round(s,3),flush=True)
        if s>1.4:
            print("adjacent! spinning to find blob",flush=True)
            # look for the robot in lidar: cell adjacent
            L=scan(3)
            print([round(v,2) for v in L],flush=True)
            write_port("d8","H ADJACENT s=%.2f - HOLD STILL"%s)
            time.sleep(2)
            if savg()>1.4: 
                # try to bump into it? just wait & watch goal flag
                time.sleep(3)
                continue
            else: continue
        w,_=walls_here(bot.h)
        if w is None: continue
        # evaluate each open dir by peeking: step in, measure, possibly return
        best=None
        dirs=[D for D in DIRS if not w[D]]
        # prefer last successful direction first
        if last_dir in dirs: dirs.remove(last_dir); dirs.insert(0,last_dir)
        moved=False
        for D in dirs:
            if time.time()>t_end: break
            bot.face(D)
            l=scan(2)
            if l and 0<l[0]<0.33: continue
            if not bot.step(): 
                pass
            s2=savg()
            print(" tried",D,"->s",round(s2,3),flush=True)
            if s2>s+0.01:
                s_prev=s2; last_dir=D; moved=True; break
            # step back
            back=(D+180)%360
            bot.face(back)
            bot.step()
            bot.face(D)  # restore-ish
        if not moved:
            print("no improving dir; waiting 5s",flush=True)
            for _ in range(5):
                write_port("d8","H stuck; HOLD STILL, I retry")
                time.sleep(1)
    stop()
    print("home end",flush=True)

if __name__=='__main__':
    main()
