
import sys, time, math, pickle, os
sys.path.insert(0,'/bot/src')
from robot import R
def wrap(a):
    while a>180: a-=360
    while a<-180: a+=360
    return a
class WF:
    def __init__(self):
        self.r=R()
        self.x=0.0; self.y=0.0; self.mpt=0.000503; self.prev=None
        self.logf=open('/memory/wf.log','a',buffering=1)
        self.rxlog=open('/memory/rx.log','a',buffering=1)
        self.last_ping=0
    def log(self,s): self.logf.write(f"[{time.time():.0f}] {s}\n")
    def enc(self):
        for _ in range(4):
            e=self.r.enc()
            if e[0] is not None and e[1] is not None: return e
            time.sleep(0.03)
        return None
    def frontmin(self,rg):
        f=[rg[i] for i in (0,1,15) if rg[i] is not None and rg[i]>=0]
        return min(f) if f else None
    def ping(self,msg=None):
        now=time.time()
        if msg: self.r.write(8,msg)
        elif now-self.last_ping>3.5:
            self.last_ping=now
            self.r.write(8,f"B2 PING x={self.x:.2f} y={self.y:.2f} t={now:.0f}")
        m=self.r.read(10,0.1)
        if m:
            self.rxlog.write(f"[{time.time():.0f}] RX: {m}\n")
            if 'GOAL' in m.upper():
                self.log(f"GOAL MSG FROM R1: {m}")
                try:
                    parts=dict(p.split('=',1) for p in m.split() if '=' in p)
                    if 'x' in parts and 'y' in parts:
                        self.goal_xy=(float(parts['x']),float(parts['y']))
                except: pass
            return m
        return None
    def run(self):
        r=self.r
        self.log("WALLFOLLOW START (left-hand)")
        self.goal_xy=None
        left_open_since=0
        mode='run'
        while True:
            open('/memory/heartbeat','w').write(f"{time.time():.0f} {self.x:.2f} {self.y:.2f} m={mode}\n")
            if os.path.exists('/memory/STOP'):
                r.motors(0,0); self.log("STOP FILE"); break
            st=r.status() or (0,0,0)
            tick,goal,here=st
            d0=r.read(0); d5=r.read(5)
            if here==1:
                self.log("*** HERE=1 AT GOAL ***")
                r.motors(0,0)
                for i in range(3):
                    r.write(8,f"B2 ATGOAL x={self.x:.2f} y={self.y:.2f}")
                    time.sleep(0.3)
                    self.ping()
                pickle.dump({'x':self.x,'y':self.y,'h':r.heading()},open('/memory/goalfound.pkl','wb'))
                mode='atgoal'
            if mode=='atgoal':
                r.motors(0,0)
                self.ping(f"B2 ATGOAL x={self.x:.2f} y={self.y:.2f}")
                time.sleep(0.8)
                continue
            if goal==1: self.log(f"goal=1 at ({self.x:.2f},{self.y:.2f}) h={r.heading()}")
            if (d0 and d0 not in ('0','-')) or (d5 and d5 not in ('0','-')):
                self.log(f"SENSOR d0={d0} d5={d5} at ({self.x:.2f},{self.y:.2f})")
            self.ping()
            e=self.enc(); h=r.heading()
            if e and self.prev and h is not None:
                d=((e[0]-self.prev[0])+(e[1]-self.prev[1]))/2.0*self.mpt
                self.x+=d*math.cos(math.radians(h)); self.y+=d*math.sin(math.radians(h))
            if e: self.prev=e
            rg=r.ranges()
            if rg is None or h is None:
                r.motors(0,0); time.sleep(0.08); continue
            fmin=self.frontmin(rg)
            b3,b4,b5=rg[3],rg[4],rg[5]
            L=[]
            if b4 is not None and b4>=0: L.append(b4)
            if b3 is not None and b3>=0: L.append(b3*1.35)
            if b5 is not None and b5>=0: L.append(b5*1.35)
            dl=min(L) if L else None
            if fmin is not None and fmin<0.42:
                r.motors(0,0); time.sleep(0.1)
                self.log(f"WALL f={fmin:.2f} at ({self.x:.2f},{self.y:.2f},{h:.0f})")
                t0=time.time(); steps=0
                while time.time()-t0<4:
                    rg2=r.ranges()
                    if rg2 is not None:
                        f2=self.frontmin(rg2)
                        if f2 is not None and f2>0.75: break
                    r.motors(-75,75); time.sleep(0.09); steps+=1
                r.motors(0,0); time.sleep(0.1)
                self.log(f"turned right {steps} steps")
                continue
            target=0.30
            err=0.0 if dl is None else dl-target
            base=70 if (fmin or 9)>0.8 else 45
            K=140.0
            a=int(max(-40,min(110, base+K*err)))
            b=int(max(-40,min(110, base-K*err)))
            if dl is not None and dl>0.65: left_open_since+=1
            else: left_open_since=0
            if left_open_since>6 and (fmin or 9)>0.9:
                r.motors(0,0); time.sleep(0.05)
                self.log("LEFT OPEN - turn left")
                t0=time.time()
                while time.time()-t0<0.5:
                    r.motors(75,-75); time.sleep(0.06)
                r.motors(0,0); left_open_since=0
                continue
            r.motors(a,b)
            time.sleep(0.1)
if __name__=="__main__":
    w=WF()
    try: w.run()
    except Exception as ex:
        w.r.motors(0,0); w.log(f"FATAL {ex!r}")
        import traceback; traceback.print_exc(file=w.logf)
        raise
