import os, time, math, collections, sys

DEV="/dev/robot/"
def rd(p):
    with open(DEV+p) as f: return f.readline().strip()
def wr(p,v):
    with open(DEV+p,"w") as f: f.write(str(v)+"\n")
rx_fd=os.open(DEV+"d3", os.O_RDONLY|os.O_NONBLOCK)
rx_buf=b""
def rx_poll():
    global rx_buf
    out=[]
    try:
        while True:
            c=os.read(rx_fd,4096)
            if not c: break
            rx_buf+=c
    except BlockingIOError: pass
    while b"\n" in rx_buf:
        l,rx_buf=rx_buf.split(b"\n",1)
        t=l.decode(errors="replace").strip()
        if t: out.append(t)
    return out
last=[3.0]*16
def scan():
    global last
    try:
        r=[float(v) for v in rd("d1").split(",")]
        for i,v in enumerate(r):
            if v>=0: last[i]=v
    except: pass
    return last[:]
def heading():
    for _ in range(5):
        try: return float(rd("d2"))
        except: pass
    return 0.0
def sig1():
    try:
        s=rd("d6"); return float(s) if s else None
    except: return None
def sample_s(n=15):
    vs=[]
    t0=time.time()
    while len(vs)<n and time.time()-t0<2:
        v=sig1()
        if v is not None: vs.append(v)
    vs.sort()
    return vs[len(vs)//2] if vs else None
log=open("/memory/run.log","a",buffering=1)
def L(*a): log.write(" ".join(str(v) for v in a)+"\n"); 
def ang_norm(a): return (a+180)%360-180
def setcmd(l,r): wr("d4",l); wr("d5",r)
def stop(): setcmd(0,0)

def rotate_to(target):
    h=heading()
    t0=time.time()
    while time.time()-t0<8:
        e=ang_norm(target-h)
        if abs(e)<6: break
        sp=max(6,min(30,abs(e)*0.6))
        if e>0: setcmd(-sp,sp)
        else: setcmd(sp,-sp)
        time.sleep(0.05)
        h=heading()
    stop()
    return h

K_V=0.085
def drive(dist, x, y):
    # drive straight-ish with side correction; returns new (x,y,traveled,reason)
    h0=heading()
    t_last=time.time(); trav=0.0
    setcmd(10,10)
    t0=time.time(); reason="dist"
    while True:
        now=time.time(); dt=now-t_last; t_last=now
        h=heading()
        r=scan()
        if rd("d0")=="1": reason="bump"; break
        front=min(r[0],r[15]*1.05,r[1]*1.05)
        if front<0.28: reason="wall"; break
        v=K_V*10
        trav+=v*dt
        x+=v*math.cos(math.radians(h))*dt; y+=v*math.sin(math.radians(h))*dt
        if trav>=dist: break
        if now-t0>dist/0.5+2: reason="timeout"; break
        # small centering correction
        steer=0.0
        if r[2]<0.25: steer-=4
        if r[14]<0.25: steer+=4
        if r[3]<0.18: steer-=4
        if r[13]<0.18: steer+=4
        # keep heading
        steer+=max(-6,min(6,0.4*ang_norm(h0-h)*-1))
        setcmd(round(10-steer,1), round(10+steer,1))
        time.sleep(0.05)
    stop()
    if reason=="bump":
        setcmd(-8,-8); time.sleep(0.5); stop()
        x-=0.35*math.cos(math.radians(heading())); y-=0.35*math.sin(math.radians(heading()))
    return x,y,trav,reason

x,y=float(sys.argv[1]),float(sys.argv[2])
recs=[]   # (x,y,s)
TX,TY=None,None
if len(sys.argv)>4: TX,TY=float(sys.argv[3]),float(sys.argv[4])
t0=time.time()
L(f"=== home start {time.time()} at ({x},{y}) ===")

def refit():
    global TX,TY
    if len(recs)<8: return
    pts=recs[-400:]
    best=None
    x0,y0=pts[-1][0],pts[-1][1]
    for X in range(int(x0)-40,int(x0)+41,2):
        for Y in range(int(y0)-40,int(y0)+41,2):
            for p in (1,2):
                logs=[(math.log(s), -p*math.log(math.hypot(a-X,b-Y)+0.5)) for a,b,s in pts]
                mC=sum(u-w for u,w in logs)/len(logs)
                res=sum((u-(w+mC))**2 for u,w in logs)/len(logs)
                if best is None or res<best[0]: best=(res,X,Y,p)
    TX,TY=best[1],best[2]
    L(f"FIT t={time.time()-t0:.0f} res={best[0]:.4f} T=({TX},{TY}) p={best[3]} n={len(pts)}")

step=0
while True:
    s=sample_s()
    if s is not None: recs.append((x,y,s))
    for m in rx_poll(): L(f"RX pos=({x:.1f},{y:.1f}) {m!r}")
    wr("d8","HELLO from A")
    st=rd("d9"); d7=rd("d7")
    if ("goal=" in st and "goal=0" not in st) or d7 not in ("","0"):
        L(f"!!! d9={st} d7={d7} pos=({x:.2f},{y:.2f})")
    L(f"H t={time.time()-t0:.0f} x={x:.2f} y={y:.2f} s={s} T=({TX},{TY})")
    if step%8==7: refit()
    r=scan(); h=heading()
    # candidate directions: beams with clearance
    if TX is not None:
        desired=math.degrees(math.atan2(TY-y,TX-x))
    else:
        desired=h  # keep going
    best=None
    for i in range(16):
        ang=h+i*22.5
        clr=min(r[i], r[(i+1)%16]*1.2, r[(i-1)%16]*1.2)
        if clr<0.55: continue
        cost=abs(ang_norm(ang-desired))
        # slight penalty for going backward relative to current heading
        if best is None or cost<best[0]: best=(cost,ang,clr)
    if best is None:
        # boxed in: rotate 90 and hope
        rotate_to(h+90); step+=1; continue
    rotate_to(best[1])
    x,y,trav,reason=drive(min(0.9,best[2]-0.15), x, y)
    L(f"M t={time.time()-t0:.0f} to h={best[1]:.0f} trav={trav:.2f} {reason} -> ({x:.2f},{y:.2f})")
    step+=1
