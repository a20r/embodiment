import os, time, math, random, collections

class Port:
    def __init__(self, n, mode):
        p=f"/dev/robot/d{n}"
        if mode=='r':
            self.fd=os.open(p, os.O_RDONLY|os.O_NONBLOCK); self.buf=b""; self.last=None
        else:
            self.fd=os.open(p, os.O_WRONLY|os.O_NONBLOCK)
    def read(self):
        try:
            while True:
                d=os.read(self.fd,65536)
                if not d: break
                self.buf+=d
        except BlockingIOError: pass
        if b"\n" in self.buf:
            ls=self.buf.split(b"\n"); self.buf=ls[-1]; self.last=ls[-2].decode()
        return self.last
    def write(self,s):
        try: os.write(self.fd,(s+"\n").encode())
        except (BlockingIOError, BrokenPipeError): pass

lidar=Port(1,'r'); encA=Port(2,'r'); head=Port(3,'r'); stat=Port(4,'r')
bump=Port(5,'r'); p0=Port(0,'r'); encB=Port(8,'r')
mA=Port(6,'w'); mB=Port(7,'w')
_lastcmd=(None,None)
def drive(a,b):
    global _lastcmd
    a,b=int(a),int(b)
    if (a,b)!=_lastcmd:
        mA.write(str(a)); mB.write(str(b)); _lastcmd=(a,b)

log=open("/memory/run.log","a",buffering=1)
def L(*a): print(time.strftime("%H:%M:%S"),*a,file=log)

prev=[0.5]*16
def get_scan():
    global prev
    s=lidar.read()
    if s is None: return prev
    v=[float(x) for x in s.split(",")]
    prev=[prev[i] if x<0 else x for i,x in enumerate(v)]
    return prev

UPC=0.00044
x=y=0.0; ea=eb=None; H=0.0
def upd():
    global x,y,ea,eb,H
    h=head.read()
    if h is not None:
        try: H=math.radians(float(h))
        except: pass
    a=encA.read(); b=encB.read()
    try: na,nb=int(a),int(b)
    except: return
    global ea,eb
    if ea is not None:
        d=((na-ea)+(nb-eb))/2.0*UPC
        x+=d*math.cos(H); y+=d*math.sin(H)
    ea,eb=na,nb

def goal_check():
    st=stat.read() or ""
    if "goal=1" in st:
        drive(0,0); L("GOAL!",st,f"pos=({x:.2f},{y:.2f})"); print("GOAL",st); os._exit(0)
    return st

CELL=0.35
visits=collections.Counter()
def cell(px,py): return (round(px/CELL), round(py/CELL))
def wrap(a): return (a+180)%360-180

def turn_to(tdeg, timeout=10):
    t0=time.time()
    while time.time()-t0<timeout:
        upd(); goal_check(); get_scan()
        err=wrap(tdeg-math.degrees(H))
        if abs(err)<7: break
        sp=max(16,min(50,abs(err)*1.0))
        drive(sp if err>0 else -sp, -sp if err>0 else sp)
        time.sleep(0.04)
    drive(0,0)

def step_forward(dist=0.30, timeout=7):
    sx,sy=x,y; t0=time.time()
    res="ok"
    while time.time()-t0<timeout:
        upd(); goal_check()
        s=get_scan()
        if bump.read()=="1":
            drive(-70,-70); time.sleep(0.45); drive(0,0); res="bump"; break
        f=min(s[0],s[1],s[15])
        if f<0.17: res="blocked"; break
        if math.hypot(x-sx,y-sy)>=dist: break
        corr=0
        l=min(s[3],s[4]); r=min(s[12],s[13])
        if l<0.3 and r<0.3: corr=max(-14,min(14,int((l-r)*90)))
        elif l<0.14: corr=-12
        elif r<0.14: corr=12
        sp=90 if f>0.4 else 55
        drive(sp+corr, sp-corr)
        time.sleep(0.04)
    drive(0,0)
    return res

L("=== ctrl5 explorer start ===")
t0=time.time(); lastdir=0.0
dirpen=collections.defaultdict(float)  # abs sector 16 -> penalty
lastlog=0
while time.time()-t0<3300:
    upd(); st=goal_check()
    s=get_scan()
    visits[cell(x,y)]+=1
    hd=math.degrees(H)
    best=None; bestsc=-99
    for i in range(16):
        cone=min(s[(i-1)%16], s[i], s[(i+1)%16])
        if cone<0.30: continue
        absang=(hd+22.5*i)%360
        sect=int(absang//22.5)%16
        ang=math.radians(absang)
        reach=min(s[i]-0.18,0.45)
        v=visits[cell(x+reach*math.cos(ang), y+reach*math.sin(ang))]
        sc=min(cone,1.0)-0.55*v-dirpen[sect]
        if abs(wrap(22.5*i-180))<34: sc-=0.9
        if abs(wrap(absang-lastdir))<25: sc+=0.25
        sc+=random.uniform(0,0.05)
        if sc>bestsc: bestsc,best=sc,i
    for k in dirpen: dirpen[k]*=0.6
    if best is None:
        L(f"boxed pos=({x:.2f},{y:.2f})"); drive(-70,-70); time.sleep(0.9); drive(0,0)
        continue
    tdir=(hd+22.5*best)%360
    lastdir=tdir
    if time.time()-lastlog>1.5:
        lastlog=time.time()
        L(f"pos=({x:.2f},{y:.2f}) h={hd:.0f} ray{best} abs={tdir:.0f} sc={bestsc:.2f} v={visits[cell(x,y)]} d0={p0.read()} {st}")
    if best!=0: turn_to(tdir)
    r=step_forward()
    if r in ("bump","blocked"):
        sect=int(tdir//22.5)%16
        dirpen[sect]+=1.2
        ang=math.radians(tdir)
        visits[cell(x+0.25*math.cos(ang),y+0.25*math.sin(ang))]+=2
    time.sleep(0.05)
drive(0,0); L("ctrl5 exit")
