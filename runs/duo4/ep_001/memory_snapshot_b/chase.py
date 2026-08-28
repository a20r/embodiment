import os,time,math,random,collections,statistics
DEV="/dev/robot/"
def rd(p):
    with open(DEV+p) as f: return f.readline().strip()
def wr(p,v):
    with open(DEV+p,"w") as f: f.write(str(v)+"\n")
rx=os.open(DEV+"d3",os.O_RDONLY|os.O_NONBLOCK); buf=b""
log=open("/memory/run.log","a",buffering=1)
def L(*a): log.write(" ".join(map(str,a))+"\n")
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
def ang(a): return (a+180)%360-180
def setcmd(l,r): wr("d4",l); wr("d5",r)
def smed(dur=1.0):
    vs=[]; t0=time.time()
    while time.time()-t0<dur:
        try:
            v=rd("d6")
            if v: vs.append(float(v))
        except: pass
    return statistics.median(vs) if vs else None
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
def rotate(delta):
    h0=head() or 0; tgt=h0+delta; t0=time.time()
    while time.time()-t0<6:
        h=head() or 0
        e=ang(tgt-h)
        if abs(e)<7: break
        sp=max(8,min(30,abs(e)*0.6))
        setcmd(-sp if e>0 else sp, sp if e>0 else -sp)
        time.sleep(0.04)
    setcmd(0,0)
L(f"=== chase start {time.time()} ===")
t0=time.time(); lasttx=0
sprev=smed(0.8)
mode_fwd=True
while True:
    now=time.time()
    for m in rxpoll(): L(f"RX! t={now-t0:.1f} {m!r}")
    if now-lasttx>2: wr("d8","HELLO partner, chasing your beacon. Reply!"); lasttx=now
    st=rd("d9"); d7=rd("d7")
    if ("goal=" in st and "goal=0" not in st) or d7 not in ("","0"):
        L(f"!!! t={now-t0:.1f} d9={st} d7={d7}")
    # drive forward ~2s with avoid
    tseg=time.time(); bumped=False
    while time.time()-tseg<2.2:
        r=scan()
        front=min(r[0],r[15]*1.05,r[1]*1.05)
        if rd("d0")=="1":
            setcmd(-10,-10); time.sleep(0.5); setcmd(0,0); bumped=True; break
        if front<0.3: bumped=True; break
        steer=0
        if r[2]<0.22 or r[1]<0.3: steer-=5
        if r[14]<0.22 or r[15]<0.3: steer+=5
        setcmd(12-steer,12+steer)
        time.sleep(0.05)
    setcmd(0,0)
    snow=smed(0.7)
    L(f"C t={now-t0:.0f} s={snow} prev={sprev} bump={bumped} {st}")
    if snow is not None and sprev is not None:
        if snow<sprev-0.005 or bumped:
            # tumble: choose most open direction weighted toward keeping course
            r=scan()
            cands=[(i,min(r[i],r[(i+1)%16]*1.15,r[(i-1)%16]*1.15)) for i in range(16)]
            cands=[(i,c) for i,c in cands if c>0.6]
            if cands:
                i=random.choice(cands)[0]
                if bumped and i==0 and len(cands)>1: i=random.choice(cands[1:])[0]
                rotate(i*22.5)
            else:
                rotate(random.choice((90,-90,180)))
    if snow is not None: sprev=snow
