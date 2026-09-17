import sys, time, math, pickle, os
sys.path.insert(0,'/bot/src')
from robot import R
CELL=0.05; MAPR=200
def wrap(a):
    while a>180: a-=360
    while a<-180: a+=360
    return a
class Bot:
    def __init__(self):
        self.r=R()
        self.x=0.0; self.y=0.0
        self.mpt=0.000503
        self.grid={}
        self.prev_enc=None
        self.scans=0
        self.logf=open('/memory/explore.log','a',buffering=1)
        self.rxlog=open('/memory/rx.log','a',buffering=1)
        self.last_ping=0
        self.nogo={}      # (cellx,celly, dir_octant) -> fail count
        self.trail=[]
        self.cal_pts=[]
    def log(self,s):
        self.logf.write(f"[{time.time():.0f}] {s}\n")
    def key(self,x,y):
        i=int(round(x/CELL)); j=int(round(y/CELL))
        if abs(i)>MAPR or abs(j)>MAPR: return None
        return (i,j)
    def g(self,k): return self.grid.get(k,0) if k else 0
    def sense(self):
        e=self.r.enc()
        if e[0] is None or e[1] is None: return None
        th=self.r.heading()
        rg=self.r.ranges()
        st=self.r.status()
        if self.prev_enc is not None and th is not None:
            dl=e[0]-self.prev_enc[0]; dr=e[1]-self.prev_enc[1]
            d=(dl+dr)/2.0*self.mpt
            self.x+=d*math.cos(math.radians(th)); self.y+=d*math.sin(math.radians(th))
        self.prev_enc=e
        return (self.x,self.y,th,rg,st,e)
    def insert_scan(self,x,y,th,rg):
        for k,val in enumerate(rg):
            if val is None or val<0: continue
            ang=th+k*22.5
            ca=math.cos(math.radians(ang)); sa=math.sin(math.radians(ang))
            hit=val if val<2.45 else 2.45
            steps=int(hit/CELL)
            for s in range(1,steps+1):
                dd=s*CELL
                kk=self.key(x+ca*dd,y+sa*dd)
                if kk is None: break
                if val<2.45 and dd>=val-CELL:
                    # occupancy blob
                    for ox in (-1,0,1):
                        for oy in (-1,0,1):
                            self.grid[(kk[0]+ox,kk[1]+oy)]=2
                    break
                if self.grid.get(kk,0)==0: self.grid[kk]=1
    def match(self,x,y,th,rg):
        pts=[]
        for k,val in enumerate(rg):
            if val is None or val<0 or val>=2.4: continue
            a=math.radians(th+k*22.5)
            pts.append((x+val*math.cos(a), y+val*math.sin(a)))
        if len(pts)<6: return x,y
        best=None;bests=-1e9
        for dx in [-0.15,-0.1,-0.05,0,0.05,0.1,0.15]:
            for dy in [-0.15,-0.1,-0.05,0,0.05,0.1,0.15]:
                sc=0
                for px,py in pts:
                    v=self.g(self.key(px+dx,py+dy))
                    sc += 2 if v==2 else (-1 if v==1 else 0)
                if sc>bests: bests=sc;best=(dx,dy)
        return x+best[0], y+best[1]
    def ping(self):
        now=time.time()
        if now-self.last_ping>4:
            self.last_ping=now
            self.r.write(8,f"BOT PING x={self.x:.2f} y={self.y:.2f} t={now:.0f}")
        m=self.r.read(10,0.12)
        if m:
            self.rxlog.write(f"[{time.time():.0f}] RX: {m}\n")
            self.r.write(8,f"BOT HERE x={self.x:.2f} y={self.y:.2f}")
            return m
        return None
    def turnto(self,target,tol=3.0):
        r=self.r
        for _ in range(100):
            h=r.heading()
            if h is None: time.sleep(0.05); continue
            e=wrap(target-h)
            if abs(e)<tol: r.motors(0,0); time.sleep(0.12); return True
            v=int(max(-90,min(90,2.2*e)))
            r.motors(v,-v)
            time.sleep(0.08)
        r.motors(0,0); return False
    def drive(self,target,speed=80,maxt=6.0):
        # returns (cal_samples, traveled_ticks)
        r=self.r; t0=time.time()
        e0=None; cal=[]; last=None
        while time.time()-t0<maxt:
            rg=r.ranges()
            if rg is None: r.motors(0,0); time.sleep(0.08); continue
            f=[rg[i] for i in (0,15,1) if rg[i] is not None and rg[i]>=0]
            fmin=min(f) if f else 9
            if fmin<0.30:
                r.motors(-60,-60); time.sleep(0.55); r.motors(0,0)
                last='bump'; break
            if fmin<0.45:
                r.motors(0,0); last='wall'; break
            err=wrap(target-(r.heading() or target))
            a=int(speed+1.2*err); b=int(speed-1.2*err)
            r.motors(a,b); time.sleep(0.09)
            e=r.enc()
            if e[0] is not None and e[1] is not None:
                if e0 is None: e0=e
                tk=((e[0]-e0[0])+(e[1]-e0[1]))/2.0
                if fmin<1.8: cal.append((fmin,tk))
        r.motors(0,0)
        # opportunistic mpt calibration
        if len(cal)>12:
            xs=[c[1] for c in cal]; ys=[c[0] for c in cal]
            n=len(cal); mx=sum(xs)/n; my=sum(ys)/n
            num=sum((x-mx)*(y-my) for x,y in cal); den=sum((x-mx)**2 for x in xs)
            sl=num/den if den else 0
            if sl<-1e-4:
                m_new=-1.0/sl
                if 0.0003<m_new<0.0008:
                    self.mpt=0.9*self.mpt+0.1*m_new
                    self.log(f"CAL mpt={self.mpt:.6f} (inst {m_new:.6f}) n={n}")
        return last
    def choose_dir(self,x,y,th):
        rg=self.r.ranges()
        if not rg: return None
        cx,cy=int(round(x/0.5)),int(round(y/0.5))
        best=None;bestsc=-1e9
        for k in range(16):
            v=rg[k]
            if v is None or v<0 or v<0.5: continue
            ang=th+k*22.5
            ca=math.cos(math.radians(ang)); sa=math.sin(math.radians(ang))
            unk=0; blocked=False
            for s in range(1,int(1.6/CELL)):
                kk=self.key(x+ca*s*CELL,y+sa*s*CELL)
                if kk is None: break
                gv=self.g(kk)
                if gv==2: blocked=True; break
                if gv==0: unk+=1
            # nogo penalty
            oct_=int(round(((ang%360)/45)))%8
            ng=self.nogo.get((cx,cy,oct_),0)
            sc=v + unk*0.03 - (40 if blocked else 0) - ng*0.8
            if sc>bestsc: bestsc=sc; best=(k,ang,v,unk,blocked)
        return best
    def run(self):
        self.log("EXPLORER v2 START")
        stuck=0; lastx,lasty=0,0
        while True:
            open('/memory/heartbeat','w').write(f"{time.time():.0f} {self.x:.2f} {self.y:.2f}\n")
            if os.path.exists('/memory/STOP'):
                self.r.motors(0,0); self.log("STOP FILE"); break
            s=self.sense()
            if not s or s[2] is None or s[3] is None:
                self.r.motors(0,0); time.sleep(0.1); continue
            x,y,th,rg,st,e=s
            tick,goal,here=(st if st else (0,0,0))
            d0=self.r.read(0); d5=self.r.read(5)
            if here==1 or goal==1 or (d0 and d0 not in('0','-')) or (d5 and d5 not in('0','-')):
                self.log(f"FLAG! st={st} d0={d0} d5={d5}")
            self.ping()
            nx,ny=self.match(x,y,th,rg or [])
            if (nx,ny)!=(x,y):
                self.x,self.y=nx,ny; x,y=nx,ny
            self.insert_scan(x,y,th,rg or [])
            self.scans+=1
            self.trail.append((round(x,2),round(y,2)))
            self.log(f"S#{self.scans} ({x:.2f},{y:.2f},{th:.0f}) goal={goal} here={here} d0={d0} d5={d5} f={(rg or [9])[0]:.2f}")
            if self.scans%15==0:
                pickle.dump({'x':self.x,'y':self.y,'mpt':self.mpt,'grid':self.grid,'scans':self.scans}, open('/memory/map.pkl','wb'))
            # stuck detection
            if math.hypot(x-lastx,y-lasty)<0.25: stuck+=1
            else: stuck=0; lastx,lasty=x,y
            ch=self.choose_dir(x,y,th)
            if ch is None:
                self.log("NO DIR - spin 60")
                self.turnto(th+60); continue
            k,ang,v,unk,blocked=ch
            if blocked and unk==0:
                # dead end everywhere local: mark nogo, force spin
                cx,cy=int(round(x/0.5)),int(round(y/0.5))
                oct_=int(round(((ang%360)/45)))%8
                self.nogo[(cx,cy,oct_)]=self.nogo.get((cx,cy,oct_),0)+1
                self.log(f"NOGO at ({cx},{cy}) oct{oct_}")
            self.log(f"GO beam{k} abs={ang%360:.0f} clear={v:.2f} unk={unk} blocked={blocked}")
            self.turnto(ang,tol=3)
            res=self.drive(ang)
            if res=='bump':
                cx,cy=int(round(x/0.5)),int(round(y/0.5))
                oct_=int(round(((ang%360)/45)))%8
                self.nogo[(cx,cy,oct_)]=self.nogo.get((cx,cy,oct_),0)+2
                self.log(f"BUMP -> nogo oct{oct_} at ({cx},{cy})")
            if stuck>5:
                self.log("STUCK - big spin")
                self.turnto(th+120); stuck=0
            time.sleep(0.1)
if __name__=="__main__":
    b=Bot()
    try: b.run()
    except Exception as ex:
        b.r.motors(0,0); b.log(f"FATAL {ex!r}")
        import traceback; traceback.print_exc(file=b.logf)
        raise
