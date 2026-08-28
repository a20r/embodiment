import sys, time, json, random
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import stop, motors, turn_to
from mouse import Bot, walls_here, rd6, DIRS, scan, log

FOUND=[False]
def sensors_hot():
    d7=read_port("d7"); d9=read_port("d9")
    if d7 and d7.strip() not in ('0',''):
        log(event='D7', d7=d7); print("D7 HOT:",d7,flush=True); FOUND[0]=True; return True
    if d9 and 'goal=' in d9 and 'goal=0' not in d9:
        log(event='GOALFLAG', d9=d9); print("GOALFLAG:",d9,flush=True); FOUND[0]=True; return True
    return False

class Bot3(Bot):
    def step(bs):
        h=bs.h
        L=scan(2); f=L[0]
        tgt=None
        if 0<f<2.3:
            m=round((f-0.23)/0.5)
            if m<=0: return False
            tgt=0.23+(m-1)*0.5
        t0=time.time(); dist=0; last=t0; ok=True
        K_V=0.0026
        while True:
            if sensors_hot(): stop(); return True
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
            motors(base-steer, base+steer)
            dist+=K_V*base*dt
            if read_port("d0")=='1':
                motors(-100,-100); time.sleep(0.25); stop(); ok=(dist>0.3); break
            if tgt is not None and 0<fr<=tgt+0.03: break
            if tgt is None and dist>=0.5: break
            if fr<0.16: break
            if time.time()-t0>4.5: ok=(dist>0.3); break
        stop()
        dx,dy=DIRS[h]; bs.x+=dx; bs.y+=dy
        return ok

MAPF='/memory/map4.json'
def save(M,bot):
    with open(MAPF,'w') as f:
        json.dump({'cells':{f"{k[0]},{k[1]}":v for k,v in M.items()},'pos':[bot.x,bot.y],'h':bot.h},f)

def main():
    dur=float(sys.argv[1]) if len(sys.argv)>1 else 900
    t_end=time.time()+dur
    M={}; bot=Bot3(0,0)
    c=compass()
    while c is None: c=compass()
    bot.align(min(DIRS,key=lambda d: abs(((d-c+540)%360)-180)))
    path=[]; fails={}; last_ping=0
    while time.time()<t_end and not FOUND[0]:
        if sensors_hot(): break
        key=(bot.x,bot.y)
        if key not in M:
            w,L=walls_here(bot.h)
            if w is None: continue
            s=rd6()
            M[key]={'w':{str(k):v for k,v in w.items()},'s':s,'d7':read_port("d7")}
            log(cell3=[bot.x,bot.y], w=M[key]['w'], s=s)
            print("cell",key,"s=",round(s,3) if s else s,flush=True)
            save(M,bot)
        if time.time()-last_ping>3:
            write_port("d8",f"E exploring {key}"); last_ping=time.time()
        w=M[key]['w']
        order=[bot.h,(bot.h+90)%360,(bot.h+270)%360,(bot.h+180)%360]
        nxt=None
        for D in order:
            if not w[str(D)] and fails.get((key,D),0)<2:
                dx,dy=DIRS[D]
                if (bot.x+dx,bot.y+dy) not in M: nxt=D; break
        if nxt is not None:
            bot.face(nxt)
            l=scan(2)
            if l and 0<l[0]<0.33:
                M[key]['w'][str(nxt)]=True; save(M,bot); continue
            if bot.step():
                path.append((nxt+180)%360)
            else:
                fails[(key,nxt)]=fails.get((key,nxt),0)+1
                w2,_=walls_here(bot.h)
                if w2: M[(bot.x,bot.y)]={'w':{str(k):v for k,v in w2.items()},'s':rd6()}
                save(M,bot); path.append((nxt+180)%360)
            continue
        if not path:
            print("explored all n=",len(M),flush=True)
            # jiggle: random moves to escape false walls
            for _ in range(6):
                w,_=walls_here(bot.h)
                if not w: break
                opts=[D for D in DIRS if not w[D]]
                if not opts: break
                D=random.choice(opts)
                bot.face(D)
                l=scan(2)
                if l and 0<l[0]<0.33: continue
                bot.step()
            # rescan current cell fresh & clear fails
            fails.clear()
            for kk in list(M.keys()):
                pass
            # force re-explore: forget walls of visited cells adjacent to unknowns? simpler: forget nothing, but clear map of cells with any fail
            continue
        D=path.pop(); bot.face(D); bot.step()
    stop(); save(M,bot)
    if FOUND[0]:
        print("STAYING PUT - broadcasting",flush=True)
        t1=time.time()
        while time.time()-t1<600:
            write_port("d8","G AT GOAL come here! follow signal to me")
            d9=read_port("d9"); log(event='CAMP', d9=d9, d7=read_port("d7"))
            time.sleep(2)
    print("end",bot.x,bot.y,flush=True)

if __name__=='__main__':
    main()
