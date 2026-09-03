
import sys, time, math, pickle, os
from collections import deque
sys.path.insert(0,'/bot/src')
from robot import R
CELL=0.05; CCELL=0.10; MAPR=200
def wrap(a):
    while a>180: a-=360
    while a<-180: a+=360
    return a
class Nav:
    def __init__(self):
        self.r=R()
        self.x=0.0; self.y=0.0; self.mpt=0.000503; self.prev=None
        self.grid={}          # fine grid 0.05: 0 unk,1 free,2 occ
        self.logf=open('/memory/nav.log','a',buffering=1)
        self.rxlog=open('/memory/rx.log','a',buffering=1)
        self.last_ping=0
        self.scans=0
        self.goal_xy=None
        self.found=False
    def log(self,s): self.logf.write(f"[{time.time():.0f}] {s}\n")
    def fkey(self,x,y):
        return (int(round(x/CELL)),int(round(y/CELL)))
    def g(self,k): return self.grid.get(k,0) if k else 0
    def enc(self):
        for _ in range(4):
            e=self.r.enc()
            if e[0] is not None and e[1] is not None: return e
            time.sleep(0.03)
        return None
    def ping(self,msg=None):
        now=time.time()
        if msg: self.r.write(8,msg)
        elif now-self.last_ping>3.5:
            self.last_ping=now
            self.r.write(8,f"B2 PING x={self.x:.2f} y={self.y:.2f} t={now:.0f}")
        m=self.r.read(10,0.1)
        if m:
            self.rxlog.write(f"[{time.time():.0f}] RX: {m}\n")
            u=m.upper()
            if 'GOAL' in u or 'HERE' in u:
                self.log(f"INTERESTING RX: {m}")
                try:
                    parts=dict(p.split('=',1) for p in m.split() if '=' in p)
                    if 'x' in parts and 'y' in parts:
                        self.goal_xy=(float(parts['x']),float(parts['y']))
                        self.log(f"goal_xy set to {self.goal_xy}")
                except: pass
            return m
        return None
    def insert_scan(self,x,y,th,rg):
        for k,val in enumerate(rg):
            if val is None or val<0: continue
            ang=th+k*22.5
            ca=math.cos(math.radians(ang)); sa=math.sin(math.radians(ang))
            if val<2.0:
                steps=max(1,int((val-CELL)/CELL))
            else:
                steps=int(2.0/CELL)
            for s in range(1,steps+1):
                dd=s*CELL
                kk=self.fkey(x+ca*dd,y+sa*dd)
                if kk is None: break
                if val<2.0 and s==steps:
                    if self.grid.get(kk,0)!=1: self.grid[kk]=2
                else:
                    if self.grid.get(kk,0)==0: self.grid[kk]=1
    def match(self,x,y,th,rg):
        pts=[]
        for k,val in enumerate(rg):
            if val is None or val<0 or val>=2.4: continue
            a=math.radians(th+k*22.5)
            pts.append((x+val*math.cos(a), y+val*math.sin(a)))
        if len(pts)<6: return x,y
        best=(0,0);bests=-1e9
        for dx in [-0.2,-0.15,-0.1,-0.05,0,0.05,0.1,0.15,0.2]:
            for dy in [-0.2,-0.15,-0.1,-0.05,0,0.05,0.1,0.15,0.2]:
                sc=0
                for px,py in pts:
                    v=self.g(self.fkey(px+dx,py+dy))
                    sc += 2 if v==2 else (-1 if v==1 else 0)
                if sc>bests: bests=sc;best=(dx,dy)
        return x+best[0], y+best[1]
    def coarse(self):
        # build coarse grid: -1 blocked(occ or near), 0 unknown, 1 free
        cg={}
        for (i,j),v in self.grid.items():
            ci,cj=i//2,j//2   # int division works for negatives in py (floor)
            if v==2:
                for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
                    ck=(ci+di,cj+dj)
                    if cg.get(ck,0)!=-1: cg[ck]=-1
            elif v==1:
                if cg.get((ci,cj),0)==0: cg[(ci,cj)]=1
        return cg
    def plan(self):
        cg=self.coarse()
        sx,sy=int(round(self.x/CCELL)),int(round(self.y/CCELL))
        start=(sx,sy)
        if cg.get(start,0)==-1:  # standing in blocked cell (inflated) - allow anyway
            pass
        # BFS over free+unknown, find nearest unknown
        seen={start:None}
        q=deque([start])
        target=None; dist={start:0}
        while q:
            cur=q.popleft()
            v=cg.get(cur,0)
            if v==0 and cur!=start and dist[cur]>=8:
                target=cur; break
            if target is None and v==0 and cur!=start:
                target=cur
            ci,cj=cur
            for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
                nk=(ci+di,cj+dj)
                if nk in seen: continue
                nv=cg.get(nk,-1)
                if nv==-1: continue
                seen[nk]=cur; dist[nk]=dist[cur]+1
                q.append(nk)
        if target is None: return None,cg
        # reconstruct path
        path=[]
        cur=target
        while cur!=start:
            path.append(cur); cur=seen[cur]
        path.reverse()
        return path,cg

    def plan_to(self,tx,ty):
        cg=self.coarse()
        sx,sy=int(round(self.x/CCELL)),int(round(self.y/CCELL))
        gx,gy=int(round(tx/CCELL)),int(round(ty/CCELL))
        start=(sx,sy); goalc=(gx,gy)
        if cg.get(goalc,0)==-1:
            # find nearest non-blocked to goal
            best=None;bd=1e9
            for (ci,cj),v in cg.items():
                if v==-1: continue
                d=(ci-gx)**2+(cj-gy)**2
                if d<bd: bd=d;best=(ci,cj)
            if best: goalc=best
        seen={start:None}; q=deque([start]); ok=False
        while q:
            cur=q.popleft()
            if cur==goalc: ok=True; break
            ci,cj=cur
            for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
                nk=(ci+di,cj+dj)
                if nk in seen: continue
                if cg.get(nk,-1)==-1: continue
                seen[nk]=cur; q.append(nk)
        if not ok: return None,cg
        path=[]; cur=goalc
        while cur!=start:
            path.append(cur); cur=seen[cur]
        path.reverse(); return path,cg

    def turnto(self,target,tol=3.0):
        r=self.r
        for _ in range(80):
            h=r.heading()
            if h is None: time.sleep(0.05); continue
            e=wrap(target-h)
            if abs(e)<tol: r.motors(0,0); time.sleep(0.1); return True
            v=int(max(-90,min(90,2.2*e)))
            r.motors(v,-v); time.sleep(0.08)
        r.motors(0,0); return False
    def goto(self,tx,ty,maxt=8.0):
        r=self.r; t0=time.time()
        ang=math.degrees(math.atan2(ty-self.y, tx-self.x))
        self.turnto(ang,tol=4)
        res='ok'
        while time.time()-t0<maxt:
            st=r.status() or (0,0,0)
            if st[2]==1: res='here'; break
            rg=r.ranges()
            if rg is None: r.motors(0,0); time.sleep(0.06); continue
            f=[rg[i] for i in (0,1,15) if rg[i] is not None and rg[i]>=0]
            fmin=min(f) if f else 9
            if fmin<0.30:
                r.motors(-60,-60); time.sleep(0.5); r.motors(0,0); res='bump'; break
            if fmin<0.42:
                r.motors(0,0); res='wall'; break
            dist=math.hypot(tx-self.x,ty-self.y)
            if dist<0.12: res='ok'; break
            h=r.heading()
            e=self.enc()
            if e and self.prev and h is not None:
                d=((e[0]-self.prev[0])+(e[1]-self.prev[1]))/2.0*self.mpt
                self.x+=d*math.cos(math.radians(h)); self.y+=d*math.sin(math.radians(h))
            if e: self.prev=e
            # heading hold to target bearing
            ang=math.degrees(math.atan2(ty-self.y, tx-self.x))
            err=wrap(ang-h)
            a=int(max(-50,min(100, 70+1.6*err)))
            b=int(max(-50,min(100, 70-1.6*err)))
            r.motors(a,b); time.sleep(0.09)
        r.motors(0,0)
        return res
    def scan_cycle(self):
        r=self.r
        r.motors(0,0); time.sleep(0.25)
        rg=r.ranges(); h=r.heading()
        e=self.enc()
        if rg is None or h is None or e is None:
            self.log("scan read fail"); return None
        if self.prev:
            d=((e[0]-self.prev[0])+(e[1]-self.prev[1]))/2.0*self.mpt
            self.x+=d*math.cos(math.radians(h)); self.y+=d*math.sin(math.radians(h))
        self.prev=e
        nx,ny=self.match(self.x,self.y,h,rg)
        if (nx,ny)!=(self.x,self.y):
            self.log(f"adj {self.x:.2f},{self.y:.2f}->{nx:.2f},{ny:.2f}")
            self.x,self.y=nx,ny
        self.insert_scan(self.x,self.y,h,rg)
        self.scans+=1
        return rg
    def run(self):
        self.log("NAV v3 START")
        r=self.r; mode='explore'; idle=0
        while True:
            open('/memory/heartbeat','w').write(f"{time.time():.0f} {self.x:.2f} {self.y:.2f} m={mode}\n")
            if os.path.exists('/memory/STOP'):
                r.motors(0,0); self.log("STOP FILE"); break
            st=r.status() or (0,0,0)
            tick,goal,here=st
            if here==1:
                if not self.found:
                    self.found=True
                    self.log("*** HERE=1 AT GOAL ***")
                    pickle.dump({'x':self.x,'y':self.y,'h':r.heading()},open('/memory/goalfound.pkl','wb'))
                r.motors(0,0)
                self.ping(f"B2 ATGOAL x={self.x:.2f} y={self.y:.2f}")
                time.sleep(1.0)
                continue
            if goal==1: self.log(f"goal=1 at ({self.x:.2f},{self.y:.2f},{r.heading():.0f})")
            d0=r.read(0); d5=r.read(5)
            if (d0 and d0 not in ('0','-')) or (d5 and d5 not in ('0','-')):
                self.log(f"SENSOR d0={d0} d5={d5}")
            self.ping()
            if self.scans==0 or mode=='explore':
                rg=self.scan_cycle()
                if rg is None: time.sleep(0.2); continue
                if self.goal_xy:
                    path,cg=self.plan_to(self.goal_xy[0],self.goal_xy[1])
                else:
                    path,cg=self.plan()
                if path is None:
                    idle+=1
                    self.log(f"NO FRONTIER (idle={idle}) - spin 90")
                    self.turnto((r.heading() or 0)+90)
                    if idle>6:
                        self.log("NO FRONTIER repeated; broadcasting")
                        self.ping(f"B2 LOST x={self.x:.2f} y={self.y:.2f}")
                        idle=0
                    continue
                idle=0
                # take first few waypoints up to 0.6m total
                tx,ty=self.x,self.y; acc=0.0; idx=0
                for (ci,cj) in path[:8]:
                    wx,wy=ci*CCELL,cj*CCELL
                    acc=math.hypot(wx-tx,wy-ty)
                    tx,ty=wx,wy
                    if acc>0.5: break
                self.log(f"S#{self.scans} ({self.x:.2f},{self.y:.2f}) -> wp ({tx:.2f},{ty:.2f}) pathlen={len(path)} f={(rg or [9])[0]:.2f}")
                res=self.goto(tx,ty)
                self.log(f"goto res={res}")
                if res in ('wall','bump'):
                    # mark current front cell blocked to avoid repeat
                    h=r.heading()
                    bx=self.x+0.35*math.cos(math.radians(h)); by=self.y+0.35*math.sin(math.radians(h))
                    kk=self.fkey(bx,by)
                    if self.g(kk)!=2: self.grid[kk]=2
                    self.log(f"blocked cell {kk}")
                time.sleep(0.05)
if __name__=="__main__":
    n=Nav()
    try: n.run()
    except Exception as ex:
        n.r.motors(0,0); n.log(f"FATAL {ex!r}")
        import traceback; traceback.print_exc(file=n.logf)
        raise
