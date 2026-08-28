import os,time,math,collections,statistics
DEV="/dev/robot/"
def rd(p):
    with open(DEV+p) as f: return f.readline().strip()
def wr(p,v):
    with open(DEV+p,"w") as f: f.write(str(v)+"\n")
rx=os.open(DEV+"d3",os.O_RDONLY|os.O_NONBLOCK); buf=b""
log=open("/memory/run.log","a",buffering=1)
def L(*a): log.write(" ".join(map(str,a))+"\n")
def rxpoll():
    global buf
    out=[]
    try:
        while True:
            c=os.read(rx,4096)
            if not c: break
            buf+=c
    except BlockingIOError: pass
    while b"\n" in buf:
        l,buf=buf.split(b"\n",1)
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
def head():
    try: return float(rd("d2"))
    except: return None
def sig():
    try:
        v=rd("d6"); return float(v) if v else None
    except: return None
def ang(a): return (a+180)%360-180
cmdL=cmdR=None
def setcmd(l,r):
    global cmdL,cmdR
    if l!=cmdL: wr("d4",l); cmdL=l
    if r!=cmdR: wr("d5",r); cmdR=r

mode="follow"; mode_t=time.time(); turn_target=None; side=1
hist=collections.deque(maxlen=30)
swin=collections.deque(maxlen=60)
t0=time.time(); lasttx=0; lastpl=0
h=head() or 0
import math as _m
x=y=0.0
L(f"=== escort start {time.time()} ===")
while True:
    now=time.time()
    r=scan(); hh=head()
    if hh is not None: h=hh
    _v=0.085*((cmdL or 0)+(cmdR or 0))/2
    x+=_v*_m.cos(_m.radians(h))*0.085; y+=_v*_m.sin(_m.radians(h))*0.085
    bump=(rd("d0")=="1")
    s=sig()
    if s is not None: swin.append(s)
    sm=statistics.median(swin) if swin else 0
    hist.append((now,r,h))
    for m in rxpoll():
        if "hear you" not in m or int(now)%20<2: L(f"RXe t={now-t0:.0f} {m!r}")
        if "goal flag=1" in m or "flag=1" in m: L(f"!!!PARTNER AT GOAL {m!r}")
    if now-lasttx>3:
        wr("d8","BETA: exploring for goal, home on my beacon and follow. My goal flag=0.")
        lasttx=now
    st=rd("d9"); d7=rd("d7")
    if d7=="1": L(f"D7 t={now-t0:.0f} x={x:.1f} y={y:.1f} s={sm:.3f}")
    if ("goal=" in st and "goal=0" not in st):
        L(f"!!! t={now-t0:.0f} d9={st} d7={d7} ATGOAL-PARKING")
        setcmd(0,0)
        while True:
            wr("d8","BETA: I AM AT THE GOAL. HOME ON MY BEACON AND COME HERE.")
            st=rd("d9"); d7=rd("d7")
            L(f"G {time.time()-t0:.0f} d9={st} d7={d7} s={sig()}")
            for m in rxpoll(): L(f"RXg {m!r}")
            time.sleep(2)
    if now-lastpl>2:
        L(f"E t={now-t0:.0f} x={x:.1f} y={y:.1f} m={mode} s={sm:.3f} h={h:.0f} r0={r[0]:.2f} {st} d7={d7}")
        lastpl=now
    # if partner too far, pause and let it home onto us
    if False and mode=="follow" and now-mode_t>3:
        mode="waitp"; mode_t=now; setcmd(0,0)
        L(f"WAIT partner t={now-t0:.0f} s={sm:.3f}")
    stuck=False
    if len(hist)==30 and cmdL is not None and abs(cmdL)+abs(cmdR)>4 and hist[-1][0]-hist[0][0]>2.5:
        old=hist[0]
        if max(abs(a-b) for a,b in zip(old[1],r))<0.06 and abs(ang(h-old[2]))<6:
            stuck=True
    front=min(r[0],r[15]*1.05,r[1]*1.05)
    wall=r[12] if side==1 else r[4]
    wallf=r[14] if side==1 else r[2]
    if mode=="waitp":
        if sm>0.4 or now-mode_t>30:
            mode="follow"; mode_t=now; hist.clear()
    elif mode=="follow":
        if bump or stuck:
            mode="backup"; mode_t=now; setcmd(-10,-10); hist.clear()
        elif front<0.3:
            mode="turn"; mode_t=now; turn_target=h+80*side; setcmd(-18*side,18*side)
        else:
            err=(wall-0.27) if wall<2.5 else 0.45
            steer=(30*err+20*((wallf-wall) if wallf<2.5 else 0.2))*side
            steer=max(-10,min(10,steer))
            sp=14 if front>0.6 else 6
            setcmd(round(sp+steer,1), round(sp-steer,1))
    elif mode=="backup":
        if now-mode_t>0.7:
            mode="turn"; mode_t=now; turn_target=h+75*side; setcmd(-18*side,18*side)
    elif mode=="turn":
        if abs(ang(turn_target-h))<=8 or now-mode_t>4:
            mode="follow"; mode_t=now; setcmd(12,12); hist.clear()
    time.sleep(0.07)
