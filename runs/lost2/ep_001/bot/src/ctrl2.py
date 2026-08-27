import os, time, math

class Port:
    def __init__(self, n, mode):
        self.path=f"/dev/robot/d{n}"
        if mode=='r':
            self.fd=os.open(self.path, os.O_RDONLY|os.O_NONBLOCK)
            self.buf=b""; self.last=None
        else:
            self.fd=os.open(self.path, os.O_WRONLY|os.O_NONBLOCK)
    def read(self):
        try:
            while True:
                d=os.read(self.fd,65536)
                if not d: break
                self.buf+=d
        except BlockingIOError:
            pass
        if b"\n" in self.buf:
            lines=self.buf.split(b"\n")
            self.buf=lines[-1]
            self.last=lines[-2].decode()
        return self.last
    def write(self,s):
        os.write(self.fd,(s+"\n").encode())

lidar=Port(1,'r'); encA=Port(2,'r'); head=Port(3,'r'); stat=Port(4,'r')
bump=Port(5,'r'); d0=Port(0,'r'); encB=Port(8,'r')
mA=Port(6,'w'); mB=Port(7,'w')
def drive(a,b): mA.write(str(a)); mB.write(str(b))

log=open("/memory/run.log","a",buffering=1)
def L(*a): print(time.strftime("%H:%M:%S"),*a,file=log)

prev=[1.0]*16
def get_scan():
    global prev
    s=lidar.read()
    if s is None: return prev
    v=[float(x) for x in s.split(",")]
    prev=[prev[i] if x<0 else x for i,x in enumerate(v)]
    return prev

def iread(p, default=0):
    v=p.read()
    try: return int(v)
    except: return default

UPC=0.00044  # units per encoder count
SP=90
WALL=0.25
FRONT_STOP=0.22
FRONT_CLEAR=0.42

x=y=0.0
ea=eb=None
state="follow"
t0=time.time(); last_log=0
L("=== ctrl2 start ===")
while time.time()-t0<3300:
    s=get_scan()
    st=stat.read() or ""
    if "goal=1" in st:
        drive(0,0); L("GOAL!",st); print("GOAL",st); break
    h=head.read()
    try: hh=math.radians(float(h))
    except: hh=0.0
    na, nb = iread(encA), iread(encB)
    if ea is not None:
        d=((na-ea)+(nb-eb))/2.0*UPC
        x+=d*math.cos(hh); y+=d*math.sin(hh)
    ea,eb=na,nb
    b=bump.read()
    front=min(s[0],s[1],s[15])
    right=min(s[11],s[12],s[13])
    if b=="1":
        drive(-80,-80); time.sleep(0.5)
        drive(30,-30); time.sleep(0.5)
        drive(0,0); L("bump")
        state="follow"
        continue
    if state=="follow":
        if front<FRONT_STOP:
            state="turn"; drive(35,-35)
        elif right>0.55:
            # right opening: arc right
            drive(int(SP*0.35), SP)
        else:
            err=right-WALL
            corr=max(-22,min(22,int(err*160)))
            drive(SP-corr, SP+corr)
    else:  # turn left in place
        if front>FRONT_CLEAR:
            state="follow"
        else:
            drive(35,-35)
    now=time.time()
    if now-last_log>3:
        last_log=now
        L(f"{state} pos=({x:.2f},{y:.2f}) h={h} d0={d0.read()} f={front:.2f} r={right:.2f} {st} scan={','.join(f'{v:.2f}' for v in s)}")
    time.sleep(0.04)
drive(0,0); L("exit")
