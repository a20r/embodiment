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

LOG=open('/memory/run1.log','a',buffering=1)
def log(*a):
    LOG.write(f"{time.time():.1f} "+" ".join(str(x) for x in a)+"\n")

# radio listener thread
rx_msgs=[]
def rx_loop():
    while True:
        try:
            with open('/dev/robot/d4') as f:
                for line in f:
                    line=line.strip()
                    if line:
                        rx_msgs.append((time.time(),line))
                        log("RX",line)
        except Exception as e:
            time.sleep(0.5)
threading.Thread(target=rx_loop,daemon=True).start()

# pose tracking
pose=[0.0,0.0]
enc=[None,None]
def upd_pose():
    global enc
    try:
        L=float(rline(7)); R=float(rline(8)); h=heading()
    except: return
    if h is None: return
    if enc[0] is not None:
        ds=((L-enc[0])+(R-enc[1]))/2.0/1000.0
        th=math.radians(-h)
        pose[0]+=ds*math.cos(th); pose[1]+=ds*math.sin(th)
    enc[0],enc[1]=L,R

def sanitize(l):
    return [x if x>0 else 3.0 for x in l]

last_tx=0; last_stat=0
start=time.time()
mode="follow"
log("START explore")
try:
    while True:
        t=time.time()
        if t-start>3300: break
        upd_pose()
        l=lidar()
        if l is None: continue
        l=sanitize(l)
        front=min(l[0],l[1],l[15])
        left=min(l[3],l[4],l[5])
        right=min(l[11],l[12],l[13])
        # left-hand wall follow
        target=0.25
        if front<0.28:
            # turn right in place (heading increases)
            motors(30,-30)
        elif left>0.6:
            # opening on left: arc left
            motors(25,60)
        else:
            err=left-target   # too far -> steer left(neg? ccw): left motor slower
            steer=max(min(err*120,25),-25)
            base=55
            motors(base+steer,base-steer)
        if t-last_tx>2:
            last_tx=t
            w(0,json.dumps({"id":"A","x":round(pose[0],2),"y":round(pose[1],2),"t":int(t)}))
        if t-last_stat>3:
            last_stat=t
            st=rline(6); d5=rline(5)
            log("POS",round(pose[0],2),round(pose[1],2),"h",heading(),"d5",d5,"st",st,"l",",".join(f"{x:.2f}" for x in l))
            if "here=1" in st or "goal=1" in st:
                log("STATUS-HIT",st)
        time.sleep(0.05)
finally:
    motors(0,0)
    log("END explore", pose)
