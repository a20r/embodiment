import os, time, math, json

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
_lc=(None,None)
def drive(a,b):
    global _lc
    a,b=int(a),int(b)
    if (a,b)!=_lc:
        mA.write(str(a)); mB.write(str(b)); _lc=(a,b)

STATE_FILE="/memory/pose.json"
try:
    _s=json.load(open(STATE_FILE))
    x,y=_s["x"],_s["y"]
except Exception:
    x=y=0.0
ea=eb=None; H=0.0
UPC=0.00044
prev=[0.5]*16
GOAL=False

def get_scan():
    global prev
    s=lidar.read()
    if s is None: return prev
    v=[float(t) for t in s.split(",")]
    prev=[prev[i] if t<0 else t for i,t in enumerate(v)]
    return prev

def upd():
    global x,y,ea,eb,H,GOAL
    h=head.read()
    if h is not None:
        try: H=math.radians(float(h))
        except: pass
    st=stat.read() or ""
    if "goal=1" in st:
        drive(0,0)
        open("/memory/GOAL.txt","a").write(time.strftime("%H:%M:%S")+" GOAL "+st+f" pos {x:.2f} {y:.2f}\n")
        GOAL=True
    a=encA.read(); b=encB.read()
    try: na,nb=int(a),int(b)
    except: return
    global ea,eb
    if ea is not None:
        d=((na-ea)+(nb-eb))/2.0*UPC
        x+=d*math.cos(H); y+=d*math.sin(H)
    ea,eb=na,nb

def save():
    json.dump({"x":x,"y":y}, open(STATE_FILE,"w"))

def wrap(a): return (a+180)%360-180

def turn_to(tdeg, tol=6, timeout=10):
    t0=time.time()
    while time.time()-t0<timeout:
        upd()
        if GOAL: return
        err=wrap(tdeg-math.degrees(H))
        if abs(err)<tol: break
        sp=max(15,min(50,abs(err)))
        drive(sp if err>0 else -sp, -sp if err>0 else sp)
        time.sleep(0.04)
    drive(0,0); time.sleep(0.15); upd()

_trace=open("/memory/trace.txt","a",buffering=1)
def step(dist=0.3, timeout=8, fstop=0.16):
    sx,sy=x,y; t0=time.time(); res="ok"
    n=0
    while time.time()-t0<timeout:
        upd()
        if GOAL: return "goal"
        s=get_scan()
        z=p0.read()
        if z not in (None,"0"):
            _trace.write(f"D0 {z} {x:.3f} {y:.3f}\n")
        n+=1
        if n%5==0:
            _trace.write(f"P {x:.3f} {y:.3f} {math.degrees(H):.1f} {' '.join('%.2f'%v for v in s)}\n")
        if bump.read()=="1":
            drive(-70,-70); time.sleep(0.4); drive(0,0); res="bump"; break
        f=min(s[0],s[1],s[15])
        if f<fstop: res="blocked"; break
        if math.hypot(x-sx,y-sy)>=dist: break
        corr=0
        l=min(s[3],s[4]); r=min(s[12],s[13])
        if l<0.3 and r<0.3: corr=max(-14,min(14,int((l-r)*90)))
        elif l<0.14: corr=-12
        elif r<0.14: corr=12
        sp=95 if f>0.4 else 55
        drive(sp+corr, sp-corr)
        time.sleep(0.04)
    drive(0,0); upd(); save()
    return res

def survey(fn="/memory/cloud.txt"):
    # rotate ~370deg, log wall points in world frame
    upd()
    out=open(fn,"a")
    h0=math.degrees(H); acc=0; lasth=h0
    drive(22,-22)
    t0=time.time()
    while acc<370 and time.time()-t0<40:
        upd()
        if GOAL: break
        s=get_scan()
        hd=math.degrees(H)
        d=wrap(hd-lasth); acc+=abs(d); lasth=hd
        for i,dist in enumerate(s):
            a=math.radians(hd+22.5*i)
            typ = "w" if dist<1.45 else "f"
            dd=min(dist,1.45)
            out.write(f"{x+dd*math.cos(a):.3f} {y+dd*math.sin(a):.3f} {typ}\n")
        time.sleep(0.05)
    drive(0,0); out.close(); save()

def goto(tx,ty,tol=0.18,timeout=60):
    t0=time.time()
    while time.time()-t0<timeout:
        upd()
        if GOAL: return "goal"
        dx,dy=tx-x,ty-y
        if math.hypot(dx,dy)<tol: return "ok"
        tdeg=math.degrees(math.atan2(dy,dx))%360
        if abs(wrap(tdeg-math.degrees(H)))>14: turn_to(tdeg)
        r=step(min(0.3,math.hypot(dx,dy)), fstop=0.16)
        if r in ("bump","blocked"):
            return r
    return "timeout"
