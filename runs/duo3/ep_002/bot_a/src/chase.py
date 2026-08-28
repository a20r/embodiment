import sys, time, json, math, os
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import stop
from mouse import Bot, walls_here, rd6, goal_check, DIRS, scan, log

MAPF='/memory/map2.json'
def load():
    with open(MAPF) as f: d=json.load(f)
    M={tuple(map(int,k.split(','))):v for k,v in d['cells'].items()}
    return M, d['pos'], d.get('h',0)
def save(M,bot):
    with open(MAPF,'w') as f:
        json.dump({'cells':{f"{k[0]},{k[1]}":v for k,v in M.items()},
                   'pos':[bot.x,bot.y],'h':bot.h}, f)

def savg(n=5):
    vs=[]
    for _ in range(n):
        v=rd6()
        if v is not None: vs.append(v)
    return sum(vs)/len(vs) if vs else None

def radio_latest():
    try:
        lines=open('/tmp/radio.log').read().strip().split('\n')
    except Exception: return None
    for line in reversed(lines):
        if 'cell=' in line:
            t,rest=line.split(' ',1)
            c=rest.split('cell=')[1].split()[0].strip('()')
            try:
                a,b=map(int,c.split(','))
                return (float(t),a,b)
            except Exception: continue
    return None

def open_dirs_live(bot):
    w,L=walls_here(bot.h)
    return w

def main():
    dur=float(sys.argv[1]) if len(sys.argv)>1 else 400
    t_end=time.time()+dur
    M,pos,h=load()
    bot=Bot(pos[0],pos[1])
    c=compass()
    while c is None: c=compass()
    hh=min(DIRS, key=lambda d: abs(((d-c+540)%360)-180))
    bot.align(hh)
    svisit={}   # cell -> (t, s)
    delta=None
    last_msg_used=0
    while time.time()<t_end:
        if goal_check(): print("GOAL!",flush=True); break
        key=(bot.x,bot.y)
        s=savg()
        svisit[key]=(time.time(), s)
        w,L=walls_here(bot.h)
        if w is None: continue
        M[key]={'w':{str(k):v for k,v in w.items()},'s':s}
        save(M,bot)
        write_port("d8", f"C me={key} s={round(s,2)} PLEASE HOLD STILL")
        m=radio_latest()
        tgt=None
        if m and time.time()-m[0]<40:
            if m[0]>last_msg_used:
                delta=(bot.x-m[1], bot.y-m[2])  # assume they're close when heard
                last_msg_used=m[0]
            tgt=(m[1]+delta[0], m[2]+delta[1])
        print(f"at {key} s={round(s,3)} tgt={tgt}",flush=True)
        if s and s>1.1:
            print("VERY CLOSE - holding & hailing",flush=True)
            stop()
            for _ in range(10):
                write_port("d8", f"C me={key} s={round(savg(),2)} I SEE YOU - HOLD")
                time.sleep(1)
                if (savg() or 0)>1.1: break
            continue
        # candidate dirs (live walls)
        cands=[]
        for D,(dx,dy) in DIRS.items():
            if not w[D]:
                v=(bot.x+dx,bot.y+dy)
                sc=0.0
                if tgt:
                    sc -= (abs(tgt[0]-v[0])+abs(tgt[1]-v[1]))*0.3
                info=svisit.get(v)
                if info and time.time()-info[0]<45 and info[1] is not None and s is not None:
                    sc += (info[1]) * 0.5
                elif s is not None:
                    sc += s*0.5 + 0.05  # unknown bonus
                if info and time.time()-info[0]<8:
                    sc -= 0.4   # just visited penalty
                cands.append((sc,D,v))
        if not cands:
            print("boxed in",flush=True); break
        cands.sort(reverse=True)
        _,D,v=cands[0]
        bot.face(D)
        l=scan(2)
        if l and 0<l[0]<0.33:
            continue
        bot.step()
    stop(); save(M,bot)
    print("chase end at",bot.x,bot.y,flush=True)

if __name__=='__main__':
    main()
