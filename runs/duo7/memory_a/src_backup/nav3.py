import robot, time, math, json

CPM=1500.0; INF=9.9
SPEED_FAST=85; SPEED_SLOW=40
DIRS={'N':(0,1),'E':(1,0),'S':(0,-1),'W':(-1,0)}
ANG={'N':0,'E':90,'S':180,'W':270}

def lid():
    return [x if x and x>0 else INF for x in robot.lidar()]

class Nav:
    def __init__(self):
        self.x=0.;self.y=0.;self.e=robot.enc();self.h=robot.heading()
        try:
            p=json.load(open('/memory/pose.json'))
            if time.time()-p['t']<7200: self.x,self.y=p['x'],p['y']
        except: pass
        self.visits={}; self.adj={}
        try:
            d=json.load(open('/memory/graph.json'))
            for k,v in d['visits'].items():
                a,b=map(int,k.split(',')); self.visits[(a,b)]=v
            for k,v in d['adj'].items():
                a,b=map(int,k.split(','))
                self.adj[(a,b)]=set(tuple(t) for t in v)
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
        json.dump({'visits':{'%d,%d'%k:v for k,v in self.visits.items()},
                   'adj':{'%d,%d'%k:[list(t) for t in v] for k,v in self.adj.items()}},
                  open('/memory/graph.json','w'))
    def cell(self):
        return (round(self.x/0.5), round(self.y/0.5))
    def mark(self):
        c=self.cell(); self.visits[c]=self.visits.get(c,0)+1
    def edge(self,a,b):
        if a==b: return
        self.adj.setdefault(a,set()).add(b)
        self.adj.setdefault(b,set()).add(a)

nav=Nav()
trail=open('/memory/trail2.jsonl','a')
t0=time.time(); last_beacon=0; last_log=0
open_seen={}   # cell -> set of dirs seen open
try:
    for k,v in json.load(open('/memory/open_seen.json')).items():
        a,b=map(int,k.split(',')); open_seen[(a,b)]=set(v)
except: pass
_os_last=[0]
def save_open_seen():
    if time.time()-_os_last[0]>5:
        json.dump({'%d,%d'%k:sorted(v) for k,v in open_seen.items()},open('/memory/open_seen.json','w'))
        _os_last[0]=time.time()

def housekeeping():
    global last_beacon,last_log
    now=time.time()
    st=robot.status() or ''
    if 'goal=1' in st:
        robot.motors(0,0)
        with open('/memory/GOAL_FOUND.txt','a') as f:
            f.write(json.dumps({'t':now,'x':nav.x,'y':nav.y,'st':st})+'\n')
        print('GOAL FOUND at %.2f %.2f'%(nav.x,nav.y),flush=True)
        nav.save()
        while True:
            robot.motors(0,0)
            robot.tx('GOALFOUND alpha waiting on goal')
            m=robot.readline('d4',0.1)
            if m:
                with open('/memory/radio_rx.log','a') as f: f.write('%f %s\n'%(time.time(),m))
                print('RX@goal:',m,flush=True)
            time.sleep(8)
    if now-last_beacon>4:
        robot.tx('PING from=alpha exploring x=%.2f y=%.2f'%(nav.x,nav.y)); last_beacon=now
    m=robot.readline('d4',0.03)
    if m:
        with open('/memory/radio_rx.log','a') as f: f.write('%f %s\n'%(now,m))
        print('RX:',m,flush=True)
    if now-last_log>1.0:
        trail.write(json.dumps({'t':round(now-t0,1),'x':round(nav.x,2),'y':round(nav.y,2),
          'h':round(nav.h,1),'L':[round(v,2) for v in lid()]})+'\n')
        trail.flush(); nav.save(); last_log=now

def turn_to(target):
    while True:
        h=nav.update()
        d=robot.angdiff(target,h)
        if abs(d)<=4: robot.motors(0,0); return
        s=max(6,min(40,abs(d)*0.8)); s=s if d>0 else -s
        robot.motors(s,-s); time.sleep(0.06)

def card_open(L,h):
    out={}
    for name,ang in ANG.items():
        i=round(((ang-h)%360)/22.5)%16
        out[name]=min(L[i],L[(i+1)%16],L[(i-1)%16]); out[name+'0']=L[i]
    return out

def drive(H):
    dist=0.; e0=sum(robot.enc())/2.
    rc=False; lc=False; prev=nav.cell()
    while True:
        h=nav.update(); L=lid(); housekeeping()
        c=nav.cell()
        if c!=prev: nav.edge(prev,c); nav.mark(); prev=c
        dist=(sum(robot.enc())/2.-e0)/CPM
        front=L[0]
        if min(L[15],L[1])<0.17: front=min(front,0.2)
        right=min(L[3],L[4],L[5]); left=min(L[11],L[12],L[13])
        if front<0.30: robot.motors(0,0); return 'front',dist
        if L[4]<0.35: rc=True
        if L[12]<0.35: lc=True
        stop=None
        if rc and dist>0.15 and L[4]>0.55: stop='right'
        if lc and dist>0.15 and L[12]>0.55: stop='left'
        if stop:
            ee=sum(robot.enc())/2.
            robot.motors(28,28)
            while sum(robot.enc())/2.-ee<0.12*CPM: time.sleep(0.05); nav.update()
            robot.motors(0,0)
            c=nav.cell()
            if c!=prev: nav.edge(prev,c); nav.mark(); prev=c
            return stop,dist
        err=robot.angdiff(H,h)
        steer=max(-12,min(12,1.2*err))
        if right<0.5 and left<0.5: steer+=max(-6,min(6,25*(right-left)))
        elif right<0.30: steer-=4
        elif left<0.30: steer+=4
        sp=SPEED_FAST if front>0.65 else SPEED_SLOW
        robot.motors(sp+steer,sp-steer); time.sleep(0.05)

def bfs_next(start, targets):
    """return next cell step from start toward nearest target via adj"""
    from collections import deque
    if start in targets: return None
    q=deque([start]); par={start:None}
    while q:
        u=q.popleft()
        for v in nav.adj.get(u,()):
            if v not in par:
                par[v]=u; 
                if v in targets:
                    # walk back
                    while par[v]!=start: v=par[v]
                    return v
                q.append(v)
    return None

heading_card=None
while True:
    h=nav.update(); L=lid(); housekeeping(); nav.mark()
    c=card_open(L,h)
    cell=nav.cell()
    opens=[d for d in 'NESW' if c[d+'0']>0.45 and c[d]>0.18]
    open_seen.setdefault(cell,set()).update(opens)
    save_open_seen()
    # manual goto override
    goto=None
    try:
        g=json.load(open('/memory/goto.json')); goto=(g[0],g[1])
    except: pass
    if goto:
        if cell==goto:
            import os; os.remove('/memory/goto.json'); goto=None
            print('goto reached',flush=True)
        else:
            step=bfs_next(cell,{goto})
            if step:
                dx,dy=step[0]-cell[0],step[1]-cell[1]
                for d,vv in DIRS.items():
                    if vv==(dx,dy):
                        heading_card=d
                        print('goto step %s -> %s'%(cell,step),flush=True)
                        turn_to(ANG[d]); rr,dd=drive(ANG[d])
                        if rr=='front' and dd<0.06:
                            nav.adj.get(cell,set()).discard(step)
                            nav.adj.get(step,set()).discard(cell)
                            print('pruned edge',cell,step,flush=True)
                            robot.motors(-25,-25); time.sleep(0.5); robot.motors(0,0); nav.update()
                        break
                continue
            else:
                import os; os.remove('/memory/goto.json')
                print('goto unreachable',flush=True)
    # order pref: right-hand relative to heading_card
    if heading_card is None: order=['N','E','S','W']
    else:
        i='NESW'.index(heading_card)
        order=['NESW'[(i+1)%4],'NESW'[i],'NESW'[(i+3)%4],'NESW'[(i+2)%4]]
    best=None
    # 1) adjacent unvisited
    for d in order:
        if d in opens:
            nc=(cell[0]+DIRS[d][0],cell[1]+DIRS[d][1])
            if nav.visits.get(nc,0)==0: best=d; break
    # 1.5) long open corridor with unvisited cells along it
    if best is None:
        for d in order:
            if d in opens and c[d+'0']>1.0:
                steps=int(c[d+'0']/0.5)
                for k in range(2,steps+1):
                    nc=(cell[0]+DIRS[d][0]*k,cell[1]+DIRS[d][1]*k)
                    if nav.visits.get(nc,0)==0:
                        best=d; break
                if best: break
    # 2) frontier BFS
    if best is None:
        targets=set()
        for cc,ds in open_seen.items():
            for d in ds:
                nc=(cc[0]+DIRS[d][0],cc[1]+DIRS[d][1])
                if nav.visits.get(nc,0)==0: targets.add(cc)
        for _ in range(6):
            step=bfs_next(cell,targets) if targets else None
            if not step: break
            dx,dy=step[0]-cell[0],step[1]-cell[1]
            for d,(vx,vy) in DIRS.items():
                if (vx,vy)==(dx,dy):
                    if d in opens: best=d
                    break
            if best: break
            # bogus edge: prune and retry
            nav.adj.get(cell,set()).discard(step)
            nav.adj.get(step,set()).discard(cell)
    # 3) fallback: least visited neighbor, right-hand tiebreak
    if best is None:
        cand=opens or ['NESW'[('NESW'.index(heading_card)+2)%4] if heading_card else 'N']
        best=min(cand,key=lambda d:(nav.visits.get((cell[0]+DIRS[d][0],cell[1]+DIRS[d][1]),99),order.index(d) if d in order else 9))
    heading_card=best
    print('junction %s h=%.0f N=%.2f E=%.2f S=%.2f W=%.2f opens=%s best=%s pos=%.2f,%.2f'%(
        cell,h,c['N0'],c['E0'],c['S0'],c['W0'],opens,best,nav.x,nav.y),flush=True)
    turn_to(ANG[best])
    r,dist=drive(ANG[best])
    if r=='front' and dist<0.06:
        nc=(cell[0]+DIRS[best][0],cell[1]+DIRS[best][1])
        nav.visits[nc]=nav.visits.get(nc,0)+3
        robot.motors(-25,-25); time.sleep(0.5); robot.motors(0,0); nav.update()
