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
        s=rd("d6"); return float(s) if s else None
    except: return None
log=open("/memory/run.log","a",buffering=1)
def L(*a): log.write(" ".join(str(x) for x in a)+"\n")
def ang_norm(a): return (a+180)%360-180

CRUISE=12.0; K_V=0.085
x,y=0.0,0.0; h=heading() or 0.0
t_last=time.time(); cmdL=cmdR=0.0
def setcmd(l,r):
    global cmdL,cmdR
    if l!=cmdL: wr("d4",l); cmdL=l
    if r!=cmdR: wr("d5",r); cmdR=r

side=1   # 1: follow right wall, -1: follow left wall
mode="follow"; mode_t=time.time(); turn_target=None
last_tx=0; last_pl=0; t0=time.time()
hist=collections.deque(maxlen=30)
swin=collections.deque()   # (t,s)
best_s=0.0; last_switch=time.time(); trend_lo=0
L(f"=== ctrl3 start {time.time()} side={side} ===")
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
        swin.append((now,s))
        while swin and now-swin[0][0]>24: swin.popleft()
    for m in rx_poll(): L(f"RX t={now-t0:.1f} pos=({x:.1f},{y:.1f}) {m!r}")
    if now-last_tx>2:
        wr("d8", f"HELLO from A t={now-t0:.0f}"); last_tx=now
    st=rd("d9"); d7=rd("d7")
    if ("goal=" in st and "goal=0" not in st) or d7 not in ("","0"):
        L(f"!!! t={now-t0:.1f} d9={st} d7={d7} pos=({x:.2f},{y:.2f})")

    # trend: compare median of first vs second half of 24s window
    trend=None
    if swin and swin[-1][0]-swin[0][0]>20:
        mid=(swin[0][0]+swin[-1][0])/2
        a=sorted(v for t_,v in swin if t_<=mid); b=sorted(v for t_,v in swin if t_>mid)
        if len(a)>10 and len(b)>10:
            ma=a[len(a)//2]; mb=b[len(b)//2]
            trend=mb-ma
            best_s=max(best_s, mb)
            if mb<best_s-0.015 and trend<-0.004 and now-last_switch>35 and mode=="follow":
                side=-side; last_switch=now; swin.clear()
                L(f"SWITCH t={now-t0:.1f} side={side} mb={mb:.3f} best={best_s:.3f} trend={trend}")
                mode="turnabout"; mode_t=now; turn_target=h+180; setcmd(-20*1,20*1)
    if now-last_pl>1:
        L(f"P t={now-t0:.1f} x={x:.2f} y={y:.2f} h={h:.1f} m={mode} sd={side} c=({cmdL},{cmdR}) s={s} tr={None if trend is None else round(trend,4)} bs={best_s:.3f} r={','.join(f'{q:.2f}' for q in r)} {st} b={int(bump)}")
        last_pl=now

    stuck=False
    if len(hist)==30 and abs(cmdL)+abs(cmdR)>4 and hist[-1][0]-hist[0][0]>2.5:
        old=hist[0]
        if max(abs(a-b) for a,b in zip(old[1],r))<0.06 and abs(ang_norm(h-old[2]))<6:
            stuck=True

    front=min(r[0], r[15]*1.05, r[1]*1.05)
    if side==1: wall=r[12]; wallf=r[14]
    else: wall=r[4]; wallf=r[2]
    if mode=="follow":
        if bump or stuck:
            mode="backup"; mode_t=now; setcmd(-10,-10); hist.clear()
        elif front<0.3:
            mode="turn"; mode_t=now; turn_target=h+80*side; setcmd(18*side*-1,18*side)
        else:
            target=0.27
            err=(wall-target) if wall<2.5 else 0.45
            steer=(30*err+20*((wallf-wall) if wallf<2.5 else 0.2))*side
            steer=max(-10,min(10,steer))
            sp=CRUISE if front>0.6 else 5.0
            setcmd(round(sp+steer,1), round(sp-steer,1))
    elif mode=="backup":
        if now-mode_t>0.7:
            mode="turn"; mode_t=now; turn_target=h+75*side; setcmd(-18*side,18*side)
    elif mode in ("turn","turnabout"):
        if abs(ang_norm(turn_target-h))<=8 or now-mode_t>(6 if mode=="turnabout" else 4):
            mode="follow"; mode_t=now; setcmd(CRUISE,CRUISE); hist.clear()
    time.sleep(0.08)
except Exception as e:
    L(f"EXC {e!r}"); setcmd(0,0); raise
