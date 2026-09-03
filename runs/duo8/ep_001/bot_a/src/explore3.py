import time, math, threading, json, sys

def rline(p):
    for _ in range(30):
        try:
            with open(f'/dev/robot/d{p}') as f:
                s=f.readline().strip()
            if s: return s
        except: pass
        time.sleep(0.01)
    return ''
def w(p,v):
    with open(f'/dev/robot/d{p}','w') as f: f.write(str(v)+'\n')
def motors(a,b): w(10,a); w(11,b)
def lidar():
    for _ in range(30):
        s=rline(3)
        try:
            l=[float(x) for x in s.split(',')]
            if len(l)==16: return l
        except: pass
    return None
def heading():
    try: return float(rline(1))
    except: return None

LOG=open('/memory/run3.log','a',buffering=1)
def log(*a): LOG.write(f"{time.time():.1f} "+" ".join(str(x) for x in a)+"\n")

def rx_loop():
    while True:
        try:
            with open('/dev/robot/d4') as f:
                for line in f:
                    line=line.strip()
                    if line: log("RX",line)
        except: time.sleep(0.5)
threading.Thread(target=rx_loop,daemon=True).start()

pose=[float(sys.argv[1]),float(sys.argv[2])] if len(sys.argv)>2 else [0.0,0.0]
enc=[None,None]
def upd_pose():
    try:
        L=float(rline(7)); R=float(rline(8)); h=heading()
    except: return
    if h is None: return
    if enc[0] is not None:
        ds=((L-enc[0])+(R-enc[1]))/2000.0
        th=math.radians(-h)
        pose[0]+=ds*math.cos(th); pose[1]+=ds*math.sin(th)
    enc[0],enc[1]=L,R

def san(l): return [x if x>0 else 3.0 for x in l]

visited={}
def cell(): return (round(pose[0]/0.3), round(pose[1]/0.3))
side=1
last_tx=0; last_stat=0; last_loopchk=time.time(); start=time.time()
log("START explore3 pose",pose)
try:
    while time.time()-start<4800:
        t=time.time()
        upd_pose()
        c=cell(); visited[c]=visited.get(c,0)+1
        l=lidar()
        if l is None: continue
        l=san(l)
        front=min(l[0],l[1],l[15])
        left=min(l[3],l[4],l[5]); right=min(l[11],l[12],l[13])
        wall = left if side==1 else right
        if front<0.30:
            motors(35*side,-35*side)
        elif wall>0.65:
            motors(60-35*side,60+35*side)
        else:
            base=95 if front>0.9 else 65
            err=(wall-0.25)*side*120
            steer=max(min(err,28),-28)
            motors(base-steer,base+steer)
        if t-last_loopchk>20:
            last_loopchk=t
            if visited.get(cell(),0)>60:
                side=-side; log("SWITCH side",side)
                t0=time.time()
                while time.time()-t0<1.0:
                    motors(40*side,-40*side); time.sleep(0.05)
        if t-last_tx>2:
            last_tx=t
            w(0,json.dumps({"id":"A","x":round(pose[0],2),"y":round(pose[1],2)}))
        if t-last_stat>3:
            last_stat=t
            st=rline(6); d5=rline(5)
            log("POS",round(pose[0],2),round(pose[1],2),"h",round(heading() or -1,1),"d5",d5,"st",st,"l",",".join(f"{x:.2f}" for x in l))
            if "here=1" in st or "goal=1" in st: log("HIT",st)
            json.dump([pose[0],pose[1]],open('/memory/lastpose.json','w'))
        time.sleep(0.04)
finally:
    motors(0,0); log("END",pose,"cells",len(visited))
