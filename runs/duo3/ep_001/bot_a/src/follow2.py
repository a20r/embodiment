import sys, time
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import stop, motors
from mouse import walls_here, rd6, DIRS, scan
from mouse3 import Bot3, sensors_hot

def savg(n=3):
    vs=[v for v in (rd6() for _ in range(n)) if v is not None]
    return sum(vs)/len(vs) if vs else None

def main():
    dur=float(sys.argv[1]) if len(sys.argv)>1 else 2400
    t_end=time.time()+dur
    bot=Bot3(0,0)
    c=compass()
    while c is None: c=compass()
    bot.align(min(DIRS,key=lambda d: abs(((d-c+540)%360)-180)))
    lastdir=None
    while time.time()<t_end:
        if sensors_hot(): break
        s=savg()
        if s is None: continue
        if s>=1.25:
            stop(); write_port("d8",f"B adjacent holding s={round(s,2)}")
            print("adjacent",round(s,2),flush=True); time.sleep(1.0); continue
        if s>=0.85:
            stop(); write_port("d8",f"B tailing you s={round(s,2)}")
            time.sleep(0.5); continue
        if s<0.45:
            stop()
            write_port("d8",f"B anchored, low sig; loop back near me to collect me pls. s={round(s,2)}")
            print("anchor wait",round(s,2),flush=True)
            time.sleep(1.5); continue
        # 0.4<=s<0.8: active pursue, close-range gradient
        print("pursue",round(s,3),flush=True)
        w,_=walls_here(bot.h)
        if w is None: continue
        dirs=[D for D in DIRS if not w[D]]
        if lastdir in dirs: dirs.remove(lastdir); dirs.insert(0,lastdir)
        moved=False
        for D in dirs:
            if time.time()>t_end or sensors_hot(): return
            bot.face(D)
            l=scan(2)
            if l and 0<l[0]<0.33: continue
            if not bot.step(): continue
            s2=savg()
            print("  ",D,"->",round(s2,3) if s2 else s2,flush=True)
            if s2 and s2>=s-0.03:
                lastdir=D; moved=True; break
            bot.face((D+180)%360); bot.step()   # revert
        if not moved: time.sleep(0.8)
    stop(); print("follow2 end",flush=True)

if __name__=='__main__':
    main()
