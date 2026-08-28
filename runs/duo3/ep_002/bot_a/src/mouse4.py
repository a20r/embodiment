import sys, time, json, random
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import stop, motors, turn_to

CELL=0.5; WALLC=0.23; K_V=0.0026
DIRS={0:(1,0),90:(0,1),180:(-1,0),270:(0,-1)}
MAPF='/memory/map5.json'
LOG=open('/tmp/mouse4.log','a')
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+"\n"); LOG.flush()

FOUND=[False]
def hot():
    d7=read_port("d7"); d9=read_port("d9")
    if d7 and d7.strip() not in ('0',''):
        log(event='D7HOT',d7=d7); print("D7 HOT:",d7,flush=True); FOUND[0]=True; return True
    if d9 and 'goal=' in d9 and 'goal=0' not in d9:
        log(event='GOALFLAG',d9=d9); print("GOALFLAG:",d9,flush=True); FOUND[0]=True; return True
    return False

def rd6():
    s=read_port("d6")
    try: return float(s)
    except: return None

def scan(n=3):
    Ls=[]
    for _ in range(n):
        l=lidar()
        if l: Ls.append(l)
    if not Ls: return None
    out=[]
    for i in range(16):
        vs=[l[i] for l in Ls]
        good=[v for v in vs if v>=0]
        out.append(sum(good)/len(good) if len(good)>len(vs)//2 else -1.0)
    return out

def beam_for(D,c):
    return int(round(((D-c)%360)/22.5))%16

def walls_here():
    c=compass()
    while c is None: c=compass()
    L=scan(3)
    if L is None: return None
    w={}
    for D in DIRS:
        v=L[beam_for(D,c)]
        w[D]=(0<v<0.37)
    return w

def front(n=4):
    vs=[]
    for _ in range(n):
        l=lidar()
        if l and l[0]>0: vs.append(l[0])
    return sum(vs)/len(vs) if vs else -1.0

class Bot:
    def __init__(b,x=0,y=0):
        b.x=x; b.y=y; b.h=0
    def align(b,h):
        turn_to(h,tol=3); b.h=h
    def face(b,D):
        if b.h!=D: b.align(D)
    def step(b):
        """One cell forward. Returns True if moved (position updated)."""
        h=b.h
        f=front()
        tgt=None
        if 0<f<2.3:
            m=round((f-WALLC)/CELL)
            if m<=0: return False
            tgt=WALLC+(m-1)*CELL
        t0=time.time(); dist=0.0; last=t0; moved=True
        while True:
            if hot(): stop(); return True
            now=time.time(); dt=now-last; last=now
            c=compass()
            err=((h-c+540)%360-180) if c is not None else 0
            l=lidar()
            steer=max(min(err*4,40),-40)
            if l:
                lv,rv=l[4],l[12]
                if 0<lv<0.4 and 0<rv<0.4: steer+=max(min((lv-rv)*150,30),-30)
                elif 0<lv<0.33: steer-=25
                elif 0<rv<0.33: steer+=25
            base=140
            fr=l[0] if l and l[0]>0 else 9
            if tgt is not None and fr-tgt<0.25: base=70
            motors(base-steer,base+steer)
            dist+=K_V*base*dt
            if read_port("d0")=='1':
                motors(-110,-110); time.sleep(0.3); stop()
                moved=(dist>0.32); break
            if tgt is not None and 0<fr<=tgt+0.03: break
            if tgt is None and dist>=CELL: break
            if fr<0.16:
                moved=(dist>0.32); break
            if time.time()-t0>4.5:
                moved=(dist>0.32); break
        stop()
        if moved:
            dx,dy=DIRS[h]; b.x+=dx; b.y+=dy
        return moved

def save(M,bot):
    json.dump({'cells':{f"{k[0]},{k[1]}":v for k,v in M.items()},
               'pos':[bot.x,bot.y],'h':bot.h}, open(MAPF,'w'))

def bfs_path(M,src,dst):
    from collections import deque
    q=deque([src]); prev={src:None}
    while q:
        u=q.popleft()
        if u==dst: break
        if u not in M: continue
        w=M[u]['w']
        for D,(dx,dy) in DIRS.items():
            if not w[str(D)]:
                v=(u[0]+dx,u[1]+dy)
                if v not in prev and (v in M or v==dst):
                    prev[v]=D; q.append(v)
    if dst not in prev: return None
    path=[]; u=dst
    while prev[u] is not None:
        D=prev[u]; path.append(D)
        dx,dy=DIRS[D]; u=(u[0]-dx,u[1]-dy)
    return list(reversed(path))

def goto(bot,M,dst):
    p=bfs_path(M,(bot.x,bot.y),dst)
    if p is None: return False
    for D in p:
        if FOUND[0]: return True
        bot.face(D)
        if not bot.step():
            # blocked where map says open: mark wall
            src=(bot.x,bot.y)
            if src in M: M[src]['w'][str(D)]=True
            dx,dy=DIRS[D]; n=(src[0]+dx,src[1]+dy)
            if n in M: M[n]['w'][str((D+180)%360)]=True
            return False
    return True

def sense_cell(bot,M):
    key=(bot.x,bot.y)
    w=walls_here()
    if w is None: return
    s=rd6()
    if key in M:
        # merge: only trust closer looks equally; keep OR of walls? prefer new
        M[key]['w']={str(k):v for k,v in w.items()}; M[key]['s']=s
    else:
        M[key]={'w':{str(k):v for k,v in w.items()},'s':s}
        print("cell",key,"s=",round(s,3) if s else s,flush=True)
    log(cell=[bot.x,bot.y],w=M[key]['w'],s=s)
    save(M,bot)

def frontiers(M):
    out=[]
    for k,v in M.items():
        for D,(dx,dy) in DIRS.items():
            if not v['w'][str(D)] and (k[0]+dx,k[1]+dy) not in M:
                out.append((k,D))
    return out

def sealing_walls(M):
    # walls adjacent to unknown cells (candidates for false walls)
    out=[]
    for k,v in M.items():
        for D,(dx,dy) in DIRS.items():
            if v['w'][str(D)] and (k[0]+dx,k[1]+dy) not in M:
                out.append((k,D))
    return out

def main():
    dur=float(sys.argv[1]) if len(sys.argv)>1 else 3000
    t_end=time.time()+dur
    M={}; bot=Bot(0,0)
    c=compass()
    while c is None: c=compass()
    bot.align(min(DIRS,key=lambda d: abs(((d-c+540)%360)-180)))
    verified=set()
    last_ping=0
    last_new=time.time(); attempts={}
    while time.time()<t_end and not FOUND[0]:
        if hot(): break
        key=(bot.x,bot.y)
        if key not in M:
            sense_cell(bot,M); last_new=time.time()
        if time.time()-last_new>90:
            log(event='STUCK', pos=[bot.x,bot.y])
            print('STUCK, random escape',flush=True)
            import random as _r
            for _ in range(4):
                w=walls_here()
                if not w: break
                opts=[D for D in DIRS if not w[D]]
                if not opts: break
                bot.face(_r.choice(opts))
                if front()>0.4: bot.step()
            last_new=time.time()
            continue
        if time.time()-last_ping>4:
            write_port("d8",f"B exploring cell={key} sig={rd6()}. ACK PLAN: finder parks+sends GOALFOUND repeatedly; other homes on d6.")
            last_ping=time.time()
        F=frontiers(M)
        if F:
            # nearest frontier by BFS: try each sorted by manhattan
            F.sort(key=lambda f: abs(f[0][0]-bot.x)+abs(f[0][1]-bot.y))
            k,D=F[0]
            attempts[(k,D)]=attempts.get((k,D),0)+1
            if attempts[(k,D)]>3:
                M[k]['w'][str(D)]=True; save(M,bot)
                log(event='BLACKLIST', k=list(k), D=D); continue
            if (bot.x,bot.y)!=k:
                if not goto(bot,M,k):
                    save(M,bot); continue
            bot.face(D)
            fdist=front()
            if 0<fdist<0.33:
                M[k]['w'][str(D)]=True; save(M,bot); continue
            if bot.step():
                sense_cell(bot,M)
            else:
                M[k]['w'][str(D)]=True; save(M,bot)
            continue
        # no frontiers: verify sealing walls not yet verified
        S=[sw for sw in sealing_walls(M) if sw not in verified]
        if not S:
            print("fully explored+verified n=",len(M),flush=True)
            log(event='DONE',n=len(M))
            break
        S.sort(key=lambda f: abs(f[0][0]-bot.x)+abs(f[0][1]-bot.y))
        k,D=S[0]
        if (bot.x,bot.y)!=k:
            if not goto(bot,M,k):
                save(M,bot); continue
        bot.face(D)
        fdist=front(6)
        verified.add((k,D))
        if fdist<0 or fdist>0.40:
            print("false wall cleared at",k,D,"front=",round(fdist,2),flush=True)
            M[k]['w'][str(D)]=False; save(M,bot)
        else:
            # push test: try to drive through
            if 0.33<fdist<=0.40:
                if bot.step():
                    sense_cell(bot,M)
                    M[k]['w'][str(D)]=False; save(M,bot)
                    print("pushed through wall at",k,D,flush=True)
    stop(); save(M,bot)
    log(event='END',cell=[bot.x,bot.y],n=len(M))
    print("END n=",len(M),"pos",bot.x,bot.y,flush=True)

if __name__=='__main__':
    main()
