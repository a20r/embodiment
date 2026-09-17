import time, math, json, statistics, random
from robot import Bot, angdiff
from explore import Nav, turn_to, drive
from collections import deque

CELL=0.2
TARGET=(-2.35,6.05)
def ck(x,y): return (round(x/CELL),round(y/CELL))

def load_grid():
    occ={}; free={}
    for fn in ("/memory/explore2.log","/memory/approach.log","/memory/goto.log"):
        try: f=open(fn)
        except: continue
        for l in f:
            if not l.startswith("{"): continue
            try: d=json.loads(l)
            except: continue
            if "r" not in d: continue
            add_scan(occ,free,d["x"],d["y"],d["h"],d["r"])
    return occ,free

def add_scan(occ,free,x,y,h,r):
    for i in range(16):
        ri=r[i]
        if ri is None or ri<0: continue
        a=math.radians((h+22.5*i)%360)
        steps=int(min(ri,2.0)/ (CELL*0.5))
        for s in range(steps):
            d0=s*CELL*0.5
            k=ck(x+d0*math.cos(a), y+d0*math.sin(a))
            free[k]=free.get(k,0)+1
        if ri<2.0:
            k=ck(x+ri*math.cos(a), y+ri*math.sin(a))
            occ[k]=occ.get(k,0)+1

def plan(occ,free,start,goal,recent=None):
    recent=recent or {}
    def blocked(k):
        if recent.get(k,0)>=2: return True
        o=occ.get(k,0); f=free.get(k,0)
        return o>=3 and o>f*0.8
    bad=set(k for k in occ if blocked(k))|set(k for k in recent if recent[k]>=2)
    sk=ck(*start); gk=ck(*goal)
    if sk in bad:
        bad.discard(sk)
    # dijkstra-lite (BFS with cost via deque levels) - use simple uniform BFS but prefer free
    import heapq
    pq=[(0,sk)]; came={sk:None}; cost={sk:0}
    lim=20000; n=0
    while pq and n<lim:
        n+=1
        c,k=heapq.heappop(pq)
        if k==gk: break
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            nk=(k[0]+dx,k[1]+dy)
            if nk in bad: continue
            if abs(nk[0]-sk[0])>60 or abs(nk[1]-sk[1])>60: continue
            step=1.4 if dx and dy else 1.0
            w=1.0 if free.get(nk,0)>0 else 2.5   # unknown cost higher
            nc=c+step*w
            if nk not in cost or nc<cost[nk]:
                cost[nk]=nc; came[nk]=k
                heapq.heappush(pq,(nc+0.5*(abs(nk[0]-gk[0])+abs(nk[1]-gk[1])),nk))
    if gk not in came:
        # nearest reached cell to goal
        gk=min(cost,key=lambda k:(k[0]-ck(*goal)[0])**2+(k[1]-ck(*goal)[1])**2)
    path=[]; k=gk
    while k: path.append(k); k=came.get(k)
    path.reverse()
    return [(k[0]*CELL,k[1]*CELL) for k in path]

def main():
    bot=Bot(); nav=Nav(bot)
    log=open("/memory/pilot.log","a",buffering=1)
    occ,free=load_grid()
    log.write(f"grid loaded occ={len(occ)} free={len(free)} pos=({nav.x:.2f},{nav.y:.2f})\n")
    t0=time.time(); lastplan=0; path=[]; wp_i=0; seq=0
    hist=[]; recent={}
    while time.time()-t0<2500:
        nav.update()
        st=bot.status()
        rx=[l for l in bot.rx.take() if l.strip()]
        if rx: log.write(f"rx {rx[-1]}\n")
        if st.get("here"):
            bot.stop()
            log.write(f"HERE=1!!! pos=({nav.x:.2f},{nav.y:.2f})\n")
            bot.tx("botB here=1 AT GOAL. botA come within 1 minute!")
            time.sleep(1.5); continue
        r=bot.ranges(); h=bot.heading()
        if r:
            add_scan(occ,free,nav.x,nav.y,h,r)
            for i in range(16):
                if 0<=r[i]<1.5:
                    a=math.radians((h+22.5*i)%360)
                    k=ck(nav.x+r[i]*math.cos(a),nav.y+r[i]*math.sin(a))
                    recent[k]=recent.get(k,0)+1
        vs=[float(x) for x in bot.d11.last(3)]
        v=statistics.median(vs) if vs else -1
        bot.tx(f"botB PING x={nav.x:.2f} y={nav.y:.2f} d11={v:.3f} seq={seq}"); seq+=1
        hist.append((nav.x,nav.y))
        if len(hist)>10: hist.pop(0)
        stuck = len(hist)==10 and max(abs(hist[-1][0]-p[0])+abs(hist[-1][1]-p[1]) for p in hist)<0.05
        if stuck:
            bot.wheels(-85,-85); e=time.time()+1.3
            while time.time()<e: time.sleep(0.1); nav.update()
            bot.stop(); hist=[]; lastplan=0
            log.write("unstick+replan\n")
        if time.time()-lastplan>12 or not path:
            lastplan=time.time()
            path=plan(occ,free,(nav.x,nav.y),TARGET,recent); wp_i=0
            log.write(f"plan len={len(path)} from ({nav.x:.2f},{nav.y:.2f}) end={path[-1] if path else None} v={v:.3f}\n")
        # advance waypoint
        while wp_i<len(path) and math.hypot(path[wp_i][0]-nav.x,path[wp_i][1]-nav.y)<0.25:
            wp_i+=1
        if wp_i>=len(path):
            log.write(f"path done at ({nav.x:.2f},{nav.y:.2f}) v={v:.3f} st={st}\n")
            bot.stop(); lastplan=0; path=[]
            time.sleep(1); continue
        wx,wy=path[wp_i]
        bearing=math.degrees(math.atan2(wy-nav.y,wx-nav.x))%360
        d=angdiff(bearing,h)
        if abs(d)>30:
            turn_to(bot,nav,bearing); continue
        # steer with obstacle check
        cand=[x for x in (r[0],r[1],r[15]) if x>=0]
        front=min(cand) if cand else 9
        if front<0.25:
            a=math.radians(h)
            fk=ck(nav.x+(front+0.1)*math.cos(a),nav.y+(front+0.1)*math.sin(a))
            recent[fk]=recent.get(fk,0)+2
            log.write(f"blocked front={front:.2f} pos=({nav.x:.2f},{nav.y:.2f}) wp=({wx:.1f},{wy:.1f}) brg={bearing:.0f} h={h:.0f}\n")
            bot.wheels(-70,-70); time.sleep(0.8); bot.stop(); lastplan=0; continue
        corr=max(-22,min(22,d*1.3))
        sp=75
        bot.wheels(int(sp-corr),int(sp+corr))
        time.sleep(0.15)
    bot.stop()

if __name__=="__main__":
    main()
