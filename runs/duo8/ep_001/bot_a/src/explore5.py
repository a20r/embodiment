import time, math, threading, json, random

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

LOG=open('/memory/run5.log','a',buffering=1)
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

pose=[0.0,0.0]; enc=[None,None]; slip=[False]
def upd_pose():
    try:
        L=float(rline(7)); R=float(rline(8)); h=heading()
    except: return
    if h is None: return
    if enc[0] is not None and not slip[0]:
        ds=((L-enc[0])+(R-enc[1]))/2000.0
        th=math.radians(-h)
        pose[0]+=ds*math.cos(th); pose[1]+=ds*math.sin(th)
    enc[0],enc[1]=L,R

def san(l): return [x if x>0 else 3.0 for x in l]

side=1   # 1 = follow LEFT wall (beams 11-13); -1 = follow RIGHT (beams 3-5)
start=time.time(); last_tx=0; last_stat=0
hist=[]; escapes=[]
log("START explore5")
def escape(l):
    log("ESCAPE",round(pose[0],2),round(pose[1],2))
    escapes.append(time.time())
    t0=time.time()
    while time.time()-t0<1.5:
        motors(-55,-55); time.sleep(0.05)
    motors(0,0); time.sleep(0.2)
    ll=lidar()
    if ll:
        ll=san(ll)
        # widest cone: score each beam by min of itself+neighbors
        best=max(range(16),key=lambda k: min(ll[(k-1)%16],ll[k],ll[(k+1)%16]))
        # beam k is at +22.5k clockwise; turn h by +22.5*best
        h0=heading()
        if h0 is not None:
            tgt=(h0+22.5*best)%360
            for _ in range(150):
                hh=heading()
                if hh is None: continue
                err=(tgt-hh+180)%360-180
                if abs(err)<4: break
                v=max(min(err*0.8,30),-30); v=math.copysign(max(abs(v),5),v)
                motors(v,-v); time.sleep(0.04)
    motors(0,0)
try:
  while time.time()-start<4800:
    t=time.time()
    upd_pose()
    l=lidar()
    if l is None: continue
    l=san(l)
    hist.append((t,list(l)))
    while hist and hist[0][0]<t-2.5: hist.pop(0)
    slip[0]=False
    if len(hist)>10 and t-hist[0][0]>2.0:
        old=hist[0][1]
        diff=sum(abs(a-b) for a,b in zip(old,l) if a<2.9 and b<2.9)/16
        if diff<0.02:
            slip[0]=True
            escape(l); hist.clear()
            if len([e for e in escapes if e>t-60])>=3:
                side=-side; log("SWITCH side",side); escapes.clear()
            continue
    frontpath = min(l[0], l[1]+0.1, l[15]+0.1)
    blocked = l[0]<0.32 or l[1]<0.22 or l[15]<0.22
    if side==1: wallb=(l[11],l[12],l[13]); 
    else: wallb=(l[3],l[4],l[5])
    wall=min(wallb)
    if blocked:
        motors(-35*side,35*side)   # side=1: turn right? motors(+,-)=clockwise=right; away from left wall
        # side=1 follow left -> turn right = motors(+,-) ; so:
        motors(35 if side==1 else -35, -35 if side==1 else 35)
    elif wall>0.7:
        # opening on followed side: arc into it
        if side==1: motors(30,65)
        else: motors(65,30)
    else:
        base=85 if frontpath>0.9 else 55
        err=(wall-0.25)*130
        steer=max(min(err,25),-25)   # positive: move toward wall side
        if side==1: motors(base-steer,base+steer)
        else: motors(base+steer,base-steer)
    if t-last_tx>2:
        last_tx=t
        w(0,json.dumps({"id":"A","x":round(pose[0],2),"y":round(pose[1],2)}))
    if t-last_stat>3:
        last_stat=t
        st=rline(6); d5=rline(5)
        log("POS",round(pose[0],2),round(pose[1],2),"h",round(heading() or -1,1),"d5",d5,"st",st,"l",",".join(f"{x:.2f}" for x in l))
        if "here=1" in st or "goal=1" in st: log("HIT",st)
    time.sleep(0.04)
finally:
    motors(0,0); log("END",pose)
