import robot, time, math, json, heapq

CPM=1500.0; INF=9.9; R=0.125
def lid(): return [x if x and x>0 else INF for x in robot.lidar()]

class Nav:
    def __init__(self):
        self.x=0.;self.y=0.;self.e=robot.enc();self.h=robot.heading()
        try:
            p=json.load(open('/memory/pose.json'))
            if time.time()-p['t']<7200: self.x,self.y=p['x'],p['y']
        except: pass
    def update(self):
        e=robot.enc(); h=robot.heading()
        d=((e[0]-self.e[0])+(e[1]-self.e[1]))/2.0/CPM
        self.e=e; self.h=h
        r=math.radians(h)
        self.x+=d*math.sin(r); self.y+=d*math.cos(r)
        return h
    def save(self):
        json.dump({'t':time.time(),'x':self.x,'y':self.y},open('/memory/pose.json','w'))

nav=Nav()
trail=open('/memory/trail2.jsonl','a')
t0=time.time(); last_beacon=0; last_log=0

def housekeeping():
    global last_beacon,last_log
    now=time.time()
    st=robot.status() or ''
    if 'goal=1' in st:
        robot.motors(0,0)
        with open('/memory/GOAL_FOUND.txt','a') as f:
            f.write(json.dumps({'t':now,'x':nav.x,'y':nav.y,'st':st})+'\n')
        print('GOAL FOUND at %.2f %.2f'%(nav.x,nav.y),flush=True); nav.save()
        while True:
            robot.motors(0,0); robot.tx('GOALFOUND alpha waiting on goal')
            m=robot.readline('d4',0.1)
            if m:
                with open('/memory/radio_rx.log','a') as f: f.write('%f %s\n'%(time.time(),m))
                print('RX@goal:',m,flush=True)
            time.sleep(8)
    if now-last_beacon>4:
        robot.tx('PING from=alpha exploring'); last_beacon=now
    m=robot.readline('d4',0.03)
    if m:
        with open('/memory/radio_rx.log','a') as f: f.write('%f %s\n'%(now,m))
        print('RX:',m,flush=True)
    if now-last_log>0.8:
        trail.write(json.dumps({'t':round(now-t0,1),'x':round(nav.x,2),'y':round(nav.y,2),
          'h':round(nav.h,1),'L':[round(v,2) for v in lid()]})+'\n')
        trail.flush(); nav.save(); last_log=now

def build_map():
    import collections
    fv=collections.Counter(); wv=collections.Counter()
    for ln in open('/memory/trail2.jsonl'):
        if '"L"' not in ln: continue
        p=json.loads(ln)
        x,y,h=p['x'],p['y'],p['h']
        for i,d in enumerate(p['L']):
            if d>3.2: continue
            a=math.radians(h+22.5*i)
            sa,ca=math.sin(a),math.cos(a)
            for s2 in range(0,int(min(d,2.95)/R)):
                fv[(round((x+s2*R*sa)/R),round((y+s2*R*ca)/R))]+=1
            if d<2.95: wv[(round((x+d*sa)/R),round((y+d*ca)/R))]+=1
    WALL=set(c for c in wv if wv[c]>=2 and wv[c]>0.6*fv.get(c,0))
    FREE=set(c for c in fv if fv[c]>=2 and c not in WALL)
    return FREE,WALL

def plan(FREE,WALL,start):
    NEARW=set()
    for (a,b) in WALL:
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                NEARW.add((a+dx,b+dy))
    NAV=FREE
    # frontier: free cell adjacent to unknown (not free, not wall)
    ALL=FREE|WALL
    raw=set()
    for c in FREE:
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            if (c[0]+dx,c[1]+dy) not in ALL:
                raw.add(c); break
    front=set()
    for c in raw:
        for dx in (-2,-1,0,1,2):
            for dy in (-2,-1,0,1,2):
                n=(c[0]+dx,c[1]+dy)
                if n in NAV: front.add(n)
    if start not in NAV:
        # snap to nearest NAV cell
        best=None; bd=1e9
        for c in NAV:
            d=(c[0]-start[0])**2+(c[1]-start[1])**2
            if d<bd: bd=d; best=c
        if best is None or bd>64: return None,None
        start=best
    if not front: return None,start
    front-=plan.blacklist
    # Dijkstra to nearest frontier, penalize wall-adjacent cells
    pq=[(0,start)]; par={start:None}; dist={start:0}
    hit=None
    while pq:
        du,u=heapq.heappop(pq)
        if du>dist.get(u,1e9): continue
        if u in front: hit=u; break
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            v=(u[0]+dx,u[1]+dy)
            if v not in NAV: continue
            w=1+(4 if v in NEARW else 0)
            nd=du+w
            if nd<dist.get(v,1e9):
                dist[v]=nd; par[v]=u; heapq.heappush(pq,(nd,v))
    if not hit: return None,start
    path=[]
    c=hit
    while c is not None: path.append(c); c=par[c]
    path.reverse()
    return path,start

def follow(path):
    """follow list of grid cells; return 'done'|'blocked'"""
    wps=[(c[0]*R,c[1]*R) for c in path]
    i=0
    stuck_t=time.time(); last_d=1e9
    while i<len(wps):
        h=nav.update(); housekeeping()
        wx,wy=wps[i]
        dx=wx-nav.x; dy=wy-nav.y
        d=math.hypot(dx,dy)
        # skip waypoints we're near; look ahead
        j=i
        while j+1<len(wps) and math.hypot(wps[j+1][0]-nav.x,wps[j+1][1]-nav.y)<0.3:
            j+=1
        if j>i: i=j; continue
        if d<0.15:
            i+=1; continue
        tgt=math.degrees(math.atan2(dx,dy))%360
        err=robot.angdiff(tgt,h)
        L=lid()
        front=min(L[0],L[15],L[1])
        if abs(err)>40:
            s=max(8,min(40,abs(err)*0.8)); s=s if err>0 else -s
            robot.motors(s,-s)
        else:
            steer=max(-14,min(14,0.9*err))
            sp=70 if front>0.6 else (40 if front>0.3 else 25)
            if front<0.16:
                robot.motors(-30,-30); time.sleep(0.4); robot.motors(0,0)
                return 'blocked'
            robot.motors(sp+steer,sp-steer)
        time.sleep(0.05)
    robot.motors(0,0)
    return 'done'

plan.blacklist=set()
fails={}
while True:
    nav.update(); housekeeping()
    FREE,WALL=build_map()
    start=(round(nav.x/R),round(nav.y/R))
    path,snap=plan(FREE,WALL,start)
    if path is None:
        print('no frontier! wandering',flush=True)
        # rotate and drive to most open dir for fresh data
        L=lid(); h=nav.h
        i=max(range(16),key=lambda k:L[k])
        tgt=(h+22.5*i)%360
        # turn
        while abs(robot.angdiff(tgt,nav.update()))>5:
            e=robot.angdiff(tgt,nav.h); s=max(8,min(40,abs(e)*0.8)); s=s if e>0 else -s
            robot.motors(s,-s); time.sleep(0.05)
        e0=sum(robot.enc())/2.
        while sum(robot.enc())/2.-e0<0.5*CPM:
            L=lid(); housekeeping()
            if min(L[0],L[1],L[15])<0.25: break
            robot.motors(50,50); time.sleep(0.05)
        robot.motors(0,0)
        continue
    print('plan len %d from %s to %s pos=%.2f,%.2f'%(len(path),path[0],path[-1],nav.x,nav.y),flush=True)
    r=follow(path)
    print('follow:',r,flush=True)
    tgt=path[-1]
    if r=='blocked':
        fails[tgt]=fails.get(tgt,0)+1
        if fails[tgt]>=2:
            for dx in range(-2,3):
                for dy in range(-2,3):
                    plan.blacklist.add((tgt[0]+dx,tgt[1]+dy))
            print('blacklist',tgt,flush=True)
        # reposition: turn to most open dir, drive 0.3m
        L=lid(); h=nav.h
        i=max(range(16),key=lambda k:L[k])
        tg=(h+22.5*i)%360
        while abs(robot.angdiff(tg,nav.update()))>6:
            e=robot.angdiff(tg,nav.h); sp2=max(8,min(40,abs(e)*0.8)); sp2=sp2 if e>0 else -sp2
            robot.motors(sp2,-sp2); time.sleep(0.05)
        e0=sum(robot.enc())/2.
        while sum(robot.enc())/2.-e0<0.3*CPM:
            L=lid(); housekeeping()
            if min(L[0],L[1],L[15])<0.22: break
            robot.motors(45,45); time.sleep(0.05)
        robot.motors(0,0)
