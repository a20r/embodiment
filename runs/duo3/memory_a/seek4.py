import sys, time, json, os
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import stop, motors, turn_to
from mouse4 import Bot, walls_here, front, rd6, hot, FOUND, DIRS, scan, log

def savg(n=12):
    vs=[]
    for _ in range(n):
        v=rd6()
        if v is not None: vs.append(v)
    vs.sort()
    return sum(vs)/len(vs) if vs else None

RXF='/tmp/rx.log'
_rxpos=[0]
def rx_new():
    out=[]
    try:
        with open(RXF) as f:
            f.seek(_rxpos[0])
            data=f.read()
            _rxpos[0]=f.tell()
        for line in data.strip().splitlines():
            if line: out.append(line)
    except FileNotFoundError: pass
    return out

GOALMSG=[False]
def check_rx():
    for line in rx_new():
        print("RX:",line,flush=True)
        log(rx=line)
        u=line.upper()
        if 'GOALFOUND' in u and 'PLAN' not in u and 'IF EITHER' not in u:
            GOALMSG[0]=True
    return GOALMSG[0]

def left_of(h): return (h+90)%360
def right_of(h): return (h+270)%360
def back(h): return (h+180)%360

def main():
    dur=float(sys.argv[1]) if len(sys.argv)>1 else 4000
    t_end=time.time()+dur
    bot=Bot(0,0)
    c=compass()
    while c is None: c=compass()
    bot.align(min(DIRS,key=lambda d: abs(((d-c+540)%360)-180)))
    last_ping=0; lastdir=None
    # skip old rx contents? no: process all (catch earlier GOALFOUND)
    while time.time()<t_end and not FOUND[0]:
        if hot(): break
        check_rx()
        s=savg()
        if s is None: continue
        if time.time()-last_ping>4:
            mode = 'HOMING-ON-GOALFOUND' if GOALMSG[0] else ('tail' if s>=0.8 else 'seek')
            write_port("d8",f"B {mode} s={round(s,3)}. ACK your plan: finder parks at goal and beacons; other homes on d6.")
            last_ping=time.time()
        if s>=0.85 and not GOALMSG[0]:
            stop(); print("tail hold s=",round(s,3),flush=True)
            time.sleep(0.8); continue
        if s>=1.2 and GOALMSG[0]:
            stop(); print("AT PARTNER (goal?) s=",round(s,3),flush=True)
            write_port("d8",f"B arrived s={round(s,3)}")
            time.sleep(1.0); continue
        if s>=0.10 or GOALMSG[0]:
            # gradient: peek each open dir
            print("grad s=",round(s,3),flush=True)
            w=walls_here()
            if w is None: continue
            dirs=[D for D in DIRS if not w[D]]
            if lastdir in dirs: dirs.remove(lastdir); dirs.insert(0,lastdir)
            moved=False
            for D in dirs:
                if time.time()>t_end or hot(): return finish(bot)
                bot.face(D)
                if 0<front()<0.33: continue
                if not bot.step(): continue
                s2=savg()
                print("  ",D,"->",round(s2,3) if s2 else s2,flush=True)
                if s2 and s2>=s-0.02:
                    lastdir=D; moved=True; break
                bot.face(back(D)); bot.step(); bot.face(D)  # revert, restore heading sense
            if not moved: time.sleep(0.5)
            continue
        # FAR: left-wall follow, cell steps
        w=walls_here()
        if w is None: continue
        order=[left_of(bot.h), bot.h, right_of(bot.h), back(bot.h)]
        moved=False
        for D in order:
            if w[D]: continue
            bot.face(D)
            if 0<front()<0.33: continue
            if bot.step():
                moved=True; break
        sl=savg(4)
        print("wf cell",(bot.x,bot.y),"s=",round(sl,3) if sl else sl,flush=True)
        log(wf=[bot.x,bot.y], s=sl)
        if not moved:
            # trapped? turn around
            bot.face(back(bot.h))
    return finish(bot)

def finish(bot):
    stop()
    print("seek4 end pos",bot.x,bot.y,"FOUND",FOUND[0],flush=True)
    if FOUND[0]:
        while True:
            write_port("d8","GOALFOUND GOALFOUND B is parked on goal. Home on my d6 signal!")
            time.sleep(2)

if __name__=='__main__':
    main()
