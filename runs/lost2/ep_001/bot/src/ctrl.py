import os, time, sys, math

class Port:
    def __init__(self, n, mode):
        self.path=f"/dev/robot/d{n}"
        self.mode=mode
        if mode=='r':
            self.fd=os.open(self.path, os.O_RDONLY|os.O_NONBLOCK)
            self.buf=b""
            self.last=None
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

lidar=Port(1,'r'); encL=Port(2,'r'); head=Port(3,'r'); stat=Port(4,'r')
bump=Port(5,'r'); d0=Port(0,'r'); enc2=Port(8,'r')
mL=Port(6,'w'); mR=Port(7,'w')

def drive(l,r):
    mL.write(str(l)); mR.write(str(r))

log=open("/memory/run.log","a",buffering=1)
def L(*a):
    print(time.strftime("%H:%M:%S"),*a,file=log)

prev_scan=[1.0]*16
def get_scan():
    global prev_scan
    s=lidar.read()
    if s is None: return prev_scan
    v=[float(x) for x in s.split(",")]
    out=[]
    for i,x in enumerate(v):
        out.append(prev_scan[i] if x<0 else x)
    prev_scan=out
    return out

SP=60
WALL=0.25
FRONT=0.26

t0=time.time()
last_log=0
state="cruise"
while time.time()-t0 < 3300:
    s=get_scan()
    st=stat.read() or ""
    if "goal=1" in st:
        drive(0,0)
        L("GOAL REACHED!", st)
        print("GOAL", st)
        break
    b=bump.read()
    front=min(s[0],s[1],s[15])
    right=min(s[12],s[11])  # right side ~ -90deg = ray12
    rf=s[14]                # right-front diagonal
    if b=="1":
        # back off and turn left
        drive(-100,-100); time.sleep(0.6)
        drive(60,-60); time.sleep(0.7)
        drive(0,0)
        L("bump recover")
        continue
    if front < FRONT:
        # turn left in place until front clear
        drive(40,-40)
    else:
        # follow right wall: error>0 means too far from wall -> steer right (heading-)
        if right>0.9 and rf>0.9:
            # lost wall: arc right to find it
            drive(SP, int(SP*0.45))
        else:
            err = right - WALL
            corr = max(-40,min(40,int(err*250)))
            # steer right = slow right wheel? heading decreases with d6<d7? we know d6>d7 raises heading
            # to steer toward wall (heading-) need d7>d6
            drive(SP - corr, SP + corr)
    now=time.time()
    if now-last_log>2:
        last_log=now
        L(f"st={st} b={b} d0={d0.read()} h={head.read()} front={front:.2f} right={right:.2f} scan={','.join(f'{x:.2f}' for x in s)}")
    time.sleep(0.05)
drive(0,0)
L("controller exit")
