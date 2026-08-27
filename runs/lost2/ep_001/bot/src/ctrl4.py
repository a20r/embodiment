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
    def write(self,s): os.write(self.fd,(s+"\n").encode())

lidar=Port(1,'r'); encA=Port(2,'r'); head=Port(3,'r'); stat=Port(4,'r')
bump=Port(5,'r'); p0=Port(0,'r'); encB=Port(8,'r')
mA=Port(6,'w'); mB=Port(7,'w')
def drive(a,b): mA.write(str(int(a))); mB.write(str(int(b)))

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

def turn_to(tdeg, timeout=8):
    t0=time.time()
    while time.time()-t0<timeout:
        upd(); goal_check()
        err=wrap(tdeg-math.degrees(H))
        if abs(err)<7: break
        sp=max(16,min(55,abs(err)*1.1))
        drive(sp if err>0 else -sp, -sp if err>0 else sp)
        time.sleep(0.03)
    drive(0,0)

def step_forward(dist=0.30, timeout=8):
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
        # centering
        corr=0
        l=min(s[3],s[4]); r=min(s[12],s[13])
        if l<0.3 and r<0.3: corr=max(-14,min(14,int((l-r)*90)))
        elif l<0.14: corr=-12
        elif r<0.14: corr=12
        sp=90 if f>0.4 else 55
        drive(sp+corr, sp-corr)   # corr>0 steer toward +heading(left)
        time.sleep(0.03)
    drive(0,0)
    return res

L("=== ctrl4 explorer start ===")
t0=time.time(); lastdir=0.0
while time.time()-t0<3300:
    upd(); st=goal_check()
    s=get_scan()
    visits[cell(x,y)]+=1
    # score 16 directions
    best=None; bestsc=-9
    hd=math.degrees(H)
    for i in range(16):
        d=s[i]
        if d<0.30: continue
        ang=math.radians(hd+22.5*i)
        reach=min(d-0.18,0.45)
        tx,ty=x+reach*math.cos(ang), y+reach*math.sin(ang)
        v=visits[cell(tx,ty)]
        sc=min(d,1.2) - 0.55*v - (0.9 if abs(wrap(22.5*i-180))<34 else 0)
        sc += 0.25 if abs(wrap(hd+22.5*i-lastdir))<25 else 0
        sc += random.uniform(0,0.05)
        if sc>bestsc: bestsc, best = sc, i
    if best is None:
        L("boxed in; reverse"); drive(-70,-70); time.sleep(0.8); drive(0,0)
        continue
    tdir=(hd+22.5*best)%360
    lastdir=tdir
    L(f"pos=({x:.2f},{y:.2f}) h={hd:.0f} pick ray{best} abs={tdir:.0f} sc={bestsc:.2f} v={visits[cell(x,y)]} d0={p0.read()} {st}")
    if best!=0: turn_to(tdir)
    r=step_forward()
    if r=="bump":
        # mark ahead as visited-heavy to avoid
        ang=math.radians(tdir)
        visits[cell(x+0.25*math.cos(ang),y+0.25*math.sin(ang))]+=3
drive(0,0); L("ctrl4 exit")
