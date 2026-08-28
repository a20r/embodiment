import sys, time, json, math
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import motors, stop, turn_to

CELL=0.5; WALLC=0.23  # dist from center to adjacent wall
K_V=0.0026
LOG=open('/tmp/mouse.log','a')
def log(**kw):
    kw['t']=round(time.time(),2); LOG.write(json.dumps(kw)+"\n"); LOG.flush()

DIRS={0:(1,0),90:(0,1),180:(-1,0),270:(0,-1)}

def rd6():
    s=read_port("d6"); return float(s) if s else None

def scan(n=3):
    Ls=[]
    for _ in range(n):
        l=lidar()
        if l: Ls.append(l)
    if not Ls: return None
    out=[]
    for i in range(16):
        vs=[l[i] for l in Ls]
        big=[v for v in vs if v<0]
        good=[v for v in vs if v>=0]
        out.append(-1.0 if len(big)>len(good) else sum(good)/len(good))
    return out

def aligned_beams(h):
    # robot aligned to cardinal h; beam index for world dir D: ((D-h)/22.5)%16 -> but beams relative to actual compass; assume aligned
    return {D: int(((D-h)/22.5)%16) for D in DIRS}

def walls_here(h):
    L=scan()
    if L is None: return None,None
    b=aligned_beams(h)
    d={}
    for D,i in b.items():
        v=L[i]
        d[D]= (0<v<0.37)  # wall adjacent
    return d, L

def goal_check():
    d9=read_port("d9")
    if d9 and 'goal=' in d9 and 'goal=0' not in d9:
        log(event='GOAL', d9=d9); return True
    return False

class Bot:
    def __init__(bs, x=0,y=0):
        bs.x=x; bs.y=y; bs.h=None
    def align(bs, h):
        turn_to(h, tol=3)
        bs.h=h
    def face(bs, D):
        if bs.h!=D: bs.align(D)
    def step(bs):
        """move one cell in direction h. Return True ok, False blocked."""
        h=bs.h
        L=scan(2)
        f=L[0]
        tgt=None
        if 0<f<2.3:
            m=round((f-WALLC)/CELL)  # cells to wall
            if m<=0: return False
            tgt=WALLC+(m-1)*CELL
        t0=time.time(); dist=0; last=t0
        ok=True
        while True:
            now=time.time(); dt=now-last; last=now
            c=compass()
            err=((h-c+540)%360-180) if c is not None else 0
            l=lidar()
            steer=max(min(err*4,40),-40)
            # lateral centering
            if l:
                lv=l[4]; rv=l[12]
                if 0<lv<0.4 and 0<rv<0.4: steer+=max(min((lv-rv)*150,30),-30)
                elif 0<lv<0.33: steer-=25
                elif 0<rv<0.33: steer+=25
            base=140
            fr=l[0] if l and l[0]>0 else 9
            if tgt is not None and fr-tgt<0.25: base=70
            motors(base-steer, base+steer)
            dist+=K_V*base*dt
            if read_port("d0")=='1':
                motors(-100,-100); time.sleep(0.25); stop(); ok=(dist>0.3); break
            if tgt is not None and 0<fr<=tgt+0.03: break
            if tgt is None and dist>=CELL: break
            if fr<0.16: break
            if time.time()-t0>4.5: ok=(dist>0.3); break
        stop()
        dx,dy=DIRS[h]; bs.x+=dx; bs.y+=dy
        return ok

def load_map():
    try:
        with open('/memory/map.json') as f: 
            m=json.load(f)
            return {tuple(map(int,k.split(','))):v for k,v in m.items()}
    except Exception: return {}

def save_map(M):
    with open('/memory/map.json','w') as f:
        json.dump({f"{k[0]},{k[1]}":v for k,v in M.items()}, f)

def main():
    dur=float(sys.argv[1]) if len(sys.argv)>1 else 120
    t_end=time.time()+dur
    bot=Bot()
    M={}
    bot.align(0)
    path=[]
    last_ping=0
    while time.time()<t_end:
        if goal_check():
            print("GOAL!"); break
        key=(bot.x,bot.y)
        if key not in M:
            w,L=walls_here(bot.h)
            if w is None: continue
            s=rd6()
            M[key]={'w':{str(k):v for k,v in w.items()}, 's':s}
            log(cell=[bot.x,bot.y], w=M[key]['w'], s=s)
            save_map(M)
        if time.time()-last_ping>3:
            write_port("d8", f"PING A cell={bot.x},{bot.y}")
            last_ping=time.time()
        # choose unvisited open neighbor; prefer straight
        w=M[key]['w']
        order=[bot.h,(bot.h+90)%360,(bot.h+270)%360,(bot.h+180)%360]
        nxt=None
        for D in order:
            if not w[str(D)]:
                dx,dy=DIRS[D]
                if (bot.x+dx,bot.y+dy) not in M:
                    nxt=D; break
        if nxt is not None:
            bot.face(nxt)
            # re-verify wall ahead after turn
            l=scan(2)
            if l and 0<l[0]<0.3:
                M[key]['w'][str(nxt)]=True; save_map(M)
                continue
            if bot.step():
                path.append((nxt+180)%360)
            continue
        # backtrack
        if not path:
            log(event='EXPLORED_ALL'); print("explored all"); break
        D=path.pop()
        bot.face(D)
        bot.step()
    stop()
    log(event='END', cell=[bot.x,bot.y])

if __name__=='__main__':
    main()
