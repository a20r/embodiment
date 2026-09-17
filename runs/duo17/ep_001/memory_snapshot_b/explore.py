import ctl, rb, time, math, json, sys, collections, os
CELL=0.1; MAXR=2.3; INFL=2  # inflate obstacles by INFL cells
ctl.CPU=1920.0
grid={}  # (ix,iy)->1 free, 2 obstacle
bumps=set()
log=open('/bot/src/explore.log','a')
def L(*a):
    s=time.strftime('%H:%M:%S ')+' '.join(str(x) for x in a)
    log.write(s+'\n'); log.flush()
def cell(x,y): return (int(math.floor(x/CELL)),int(math.floor(y/CELL)))
def mark_scan(b,scan,h):
    if not scan: return
    for k,d in enumerate(scan):
        if d<=0: continue
        a=math.radians(h+22.5*k)
        hit = d<MAXR
        r=min(d,MAXR)
        n=int(r/CELL*2)+1
        for i in range(n):
            t=r*i/n
            c=cell(b.x+t*math.cos(a), b.y+t*math.sin(a))
            if grid.get(c)!=2: grid[c]=1
        if hit:
            c=cell(b.x+r*math.cos(a), b.y+r*math.sin(a)); grid[c]=2
def sweep(b,tag=''):
    """rotate ~30deg slowly while scanning, to fill angular gaps"""
    h0=b.h
    b.set(18,-18)
    t0=time.time()
    while time.time()-t0<3.0:
        b.update(); s=b.scan(); mark_scan(b,s,b.h)
        if (b.h-h0)%360>30 and (b.h-h0)%360<180: break
        time.sleep(0.02)
    b.stop(); mark_scan(b,b.scan(),b.h)
    b.record(tag)
def blocked(c):
    for dx in range(-INFL,INFL+1):
        for dy in range(-INFL,INFL+1):
            if grid.get((c[0]+dx,c[1]+dy))==2: return True
    return False
def frontier(c):
    if grid.get(c)!=1: return False
    for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
        if (c[0]+dx,c[1]+dy) not in grid: return True
    return False
def obst_cost(c):
    n=0
    for dx in range(-2,3):
        for dy in range(-2,3):
            if grid.get((c[0]+dx,c[1]+dy))==2:
                d=max(abs(dx),abs(dy))
                n+= 30 if d<=1 else 4
    return n
def plan(b):
    import heapq
    start=cell(b.x,b.y)
    dist={start:0}; prev={start:None}
    pq=[(0,start)]
    goal=None
    while pq:
        d,c=heapq.heappop(pq)
        if d>dist.get(c,1e9): continue
        if c!=start and frontier(c) and obst_cost(c)<30 and abs(c[0]-start[0])+abs(c[1]-start[1])>=3:
            goal=c; break
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            n=(c[0]+dx,c[1]+dy)
            if grid.get(n)!=1: continue
            nd=d+1+obst_cost(n)
            if nd<dist.get(n,1e9):
                dist[n]=nd; prev[n]=c; heapq.heappush(pq,(nd,n))
    if goal is None: return None
    path=[]; c=goal
    while c is not None: path.append(c); c=prev[c]
    return path[::-1]
def comm(b,step):
    msg=f"PING from ROBOT-A step {step} pos {b.x:.2f},{b.y:.2f} heading {b.h:.0f}. Reply with your position and what you know about the goal."
    rb.wr('d8',msg); time.sleep(0.3)
    st=rb.rd('d3',0.3)
    rx=rb.rd('d10',0.3)
    L('COMM',st,'RX:',rx)
    if rx.strip():
        open('/bot/src/msgs.txt','a').write(f'{time.time():.0f} RX: {rx}\n')
    return st,rx
def save_map():
    with open('/bot/src/grid.json','w') as f:
        json.dump([[k[0],k[1],v] for k,v in grid.items()],f)
def main():
    b=ctl.Bot()
    L('START pose',b.x,b.y,b.h)
    sweep(b,'init')
    step=0
    stuck=0
    while True:
        step+=1
        if step%3==1:
            st,rx=comm(b,step)
            if 'lost' not in st: L('*** TX NOT LOST ***',st)
        path=plan(b)
        if not path:
            L('no frontier; sweeping'); sweep(b,'nofrontier'); stuck+=1
            if stuck>3: L('giving up'); break
            continue
        # waypoint: point along path ~0.35 away (or end)
        idx=min(len(path)-1,4)
        tx=(path[idx][0]+0.5)*CELL; ty=(path[idx][1]+0.5)*CELL
        ang=math.degrees(math.atan2(ty-b.y,tx-b.x))%360
        dist=math.hypot(tx-b.x,ty-b.y)
        L(f'step {step} pos {b.x:.2f},{b.y:.2f} h {b.h:.0f} -> wp {tx:.2f},{ty:.2f} ang {ang:.0f} dist {dist:.2f} pathlen {len(path)} grid {len(grid)} d11 {rb.rd("d11",0.3)}')
        b.turn_to(ang,tol=5)
        mark_scan(b,b.scan(),b.h)
        reason,trav=b.forward(dist,speed=45,minfront=0.14)
        L('  fwd',reason,round(trav,2))
        if reason=='bump':
            c=cell(b.x+0.12*math.cos(math.radians(b.h)), b.y+0.12*math.sin(math.radians(b.h))); grid[c]=2
            b.set(-30,-30); time.sleep(0.6); b.stop(); b.update()
        elif reason.startswith('obstacle'):
            mark_scan(b,b.scan(),b.h)
        sweep(b,'step%d'%step)
        save_map()
        if os.path.exists('/bot/src/STOP'): L('STOP file'); break
    b.stop()
if __name__=='__main__':
    try: main()
    except Exception as e:
        L('EXC',repr(e)); import traceback; L(traceback.format_exc())
    finally:
        rb.wr('d1','0'); rb.wr('d7','0')
