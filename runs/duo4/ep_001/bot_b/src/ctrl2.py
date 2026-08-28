import os, time, math, collections

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
        s=l.decode(errors="replace").strip()
        if s: out.append(s)
    return out

last=[3.0]*16
def scan():
    global last
    try:
        r=[float(x) for x in rd("d1").split(",")]
        for i,v in enumerate(r):
            if v>=0: last[i]=v
    except: pass
    return last[:]
def heading():
    try: return float(rd("d2"))
    except: return None
def sig():
    try:
        s=rd("d6")
        return float(s) if s else None
    except: return None

log=open("/memory/run.log","a",buffering=1)
def L(*a): log.write(" ".join(str(x) for x in a)+"\n")
def ang_norm(a): return (a+180)%360-180

CRUISE=10.0; K_V=0.085
x,y=0.0,0.0
h=heading() or 0.0
t_last=time.time(); cmdL=cmdR=0.0
def setcmd(l,r):
    global cmdL,cmdR
    if l!=cmdL: wr("d4",l); cmdL=l
    if r!=cmdR: wr("d5",r); cmdR=r

mode="seek"; mode_t=time.time(); turn_target=None
last_tx=0; last_pl=0; t0=time.time()
hist=collections.deque(maxlen=30)
sighist=collections.deque(maxlen=200)  # (t,x,y,s)
smooth=collections.deque(maxlen=6)
grad=(0.0,0.0); gmag=0.0

L(f"=== seek start {time.time()} ===")
try:
  while True:
    now=time.time(); dt=now-t_last; t_last=now
    r=scan(); hh=heading()
    if hh is not None: h=hh
    bump=(rd("d0")=="1")
    s=sig()
    v=K_V*(cmdL+cmdR)/2
    x+=v*math.cos(math.radians(h))*dt
    y+=v*math.sin(math.radians(h))*dt
    hist.append((now,r,h))
    if s is not None:
        smooth.append(s)
        sv=sorted(smooth)[len(smooth)//2]
        sighist.append((now,x,y,sv))

    for m in rx_poll(): L(f"RX t={now-t0:.1f} pos=({x:.1f},{y:.1f}) {m!r}")
    if now-last_tx>2:
        wr("d8", f"HELLO from A t={now-t0:.0f}"); last_tx=now
    st=rd("d9"); d7=rd("d7")
    if ("goal=" in st and "goal=0" not in st) or d7 not in ("","0"):
        L(f"!!! t={now-t0:.1f} d9={st} d7={d7} pos=({x:.2f},{y:.2f})")

    # gradient estimate over trailing window with >=1.0 path spread
    if len(sighist)>30:
        pts=list(sighist)[-120:]
        xs=[p[1] for p in pts]; ys=[p[2] for p in pts]; ss=[p[3] for p in pts]
        mx=sum(xs)/len(xs); my=sum(ys)/len(ys); ms=sum(ss)/len(ss)
        sxx=sum((a-mx)**2 for a in xs); syy=sum((a-my)**2 for a in ys)
        sxy=sum((a-mx)*(b-my) for a,b in zip(xs,ys))
        sxs=sum((a-mx)*(c-ms) for a,c in zip(xs,ss))
        sys_=sum((b-my)*(c-ms) for b,c in zip(ys,ss))
        det=sxx*syy-sxy*sxy
        if det>0.05:
            gx=(syy*sxs-sxy*sys_)/det; gy=(sxx*sys_-sxy*sxs)/det
            grad=(gx,gy); gmag=math.hypot(gx,gy)

    if now-last_pl>1:
        L(f"P t={now-t0:.1f} x={x:.2f} y={y:.2f} h={h:.1f} m={mode} c=({cmdL},{cmdR}) s={s} g=({grad[0]:.4f},{grad[1]:.4f}) r={','.join(f'{q:.2f}' for q in r)} {st} b={int(bump)}")
        last_pl=now

    stuck=False
    if len(hist)==30 and abs(cmdL)+abs(cmdR)>4 and hist[-1][0]-hist[0][0]>2.5:
        old=hist[0]
        if max(abs(a-b) for a,b in zip(old[1],r))<0.06 and abs(ang_norm(h-old[2]))<6:
            stuck=True

    front=min(r[0], r[15]*1.05, r[1]*1.05)
    right=r[12]; rightf=r[14]
    if mode=="seek":
        if bump or stuck:
            mode="backup"; mode_t=now; setcmd(-10,-10); hist.clear()
        elif front<0.35:
            # blocked: fall into wall-follow for a while
            mode="follow"; mode_t=now
        else:
            if gmag>1e-4:
                des=math.degrees(math.atan2(grad[1],grad[0]))
                err=ang_norm(des-h)
            else:
                err=0
            # avoid side walls
            if r[2]<0.18 or r[3]<0.15: err-=25
            if r[14]<0.18 or r[13]<0.15: err+=25
            steer=max(-14,min(14,0.25*err))
            sp=CRUISE if front>0.6 else 5.0
            setcmd(round(sp-steer,1), round(sp+steer,1))  # err>0 => turn CCW => right wheel faster
    elif mode=="follow":
        if bump or stuck:
            mode="backup"; mode_t=now; setcmd(-10,-10); hist.clear()
        elif now-mode_t>7.0:
            mode="seek"; mode_t=now
        elif front<0.3:
            mode="turnleft"; mode_t=now; turn_target=h+80; setcmd(-18,18)
        else:
            target=0.27
            err=(right-target) if right<2.5 else 0.45
            steer=30*err+20*((rightf-right) if rightf<2.5 else 0.2)
            steer=max(-10,min(10,steer))
            sp=CRUISE if front>0.6 else 5.0
            setcmd(round(sp+steer,1), round(sp-steer,1))
    elif mode=="backup":
        if now-mode_t>0.7:
            mode="turnleft"; mode_t=now; turn_target=h+75; setcmd(-18,18)
    elif mode=="turnleft":
        if abs(ang_norm(turn_target-h))<=8 or now-mode_t>4:
            mode="follow"; mode_t=now; setcmd(CRUISE,CRUISE); hist.clear()
    time.sleep(0.08)
except Exception as e:
    L(f"EXC {e!r}"); setcmd(0,0); raise
