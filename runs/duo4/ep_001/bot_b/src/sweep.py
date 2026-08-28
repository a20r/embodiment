import time
DEV="/dev/robot/"
def rd(p):
    with open(DEV+p) as f: return f.readline().strip()
def wr(p,v):
    with open(DEV+p,"w") as f: f.write(str(v)+"\n")
log=open("/memory/run.log","a",buffering=1)
def L(*a): log.write(" ".join(map(str,a))+"\n"); print(*a)
def stop(): wr("d4",0); wr("d5",0)
last=[3.0]*16
def scan():
    global last
    try:
        r=[float(v) for v in rd("d1").split(",")]
        for i,v in enumerate(r):
            if v>=0: last[i]=v
    except: pass
    return last
def head():
    try: return float(rd("d2"))
    except: return None
def ang(a): return (a+180)%360-180
def rotate(delta):
    h0=head() or 0; tgt=h0+delta; t0=time.time()
    while time.time()-t0<7:
        h=head() or 0
        e=ang(tgt-h)
        if abs(e)<6: break
        sp=max(7,min(25,abs(e)*0.5))
        wr("d4",-sp if e>0 else sp); wr("d5",sp if e>0 else -sp)
        time.sleep(0.04)
    stop()
def leg(cmd,maxt,tag):
    wr("d4",cmd); wr("d5",cmd)
    t0=time.time(); hit=None
    while time.time()-t0<maxt:
        if rd("d7")=="1":
            hit=time.time()-t0; stop()
            L(f"SWEEP HIT {tag} after {hit:.2f}s"); break
        st=rd("d9")
        if "goal=" in st and "goal=0" not in st:
            stop(); L(f"SWEEP GOALFLAG {st}"); hit=-1; break
        if rd("d0")=="1":
            stop(); L(f"sweep bump {tag}"); break
        r=scan()
        if cmd>0 and min(r[0],r[1]*1.05,r[15]*1.05)<0.22:
            stop(); L(f"sweep wallstop {tag}"); break
    stop()
    return hit
L("=== sweep start ===")
# leg A: forward along corridor
h=leg(4,6,"fwd1")
if h is None:
    rotate(180)
    h=leg(4,12,"back-along")   # go back past start ~2u
if h is None:
    rotate(180)
    h=leg(4,6,"fwd2")
L(f"sweep done hit={h}")
# if hit: fine-tune with tiny steps to find stationary-positive spot
if h is not None and h>=0:
    for i in range(16):
        k=0;n=0;t0=time.time()
        while time.time()-t0<2:
            v=rd("d7"); n+=1; k+=(v=="1")
        L(f"tune {i} {k}/{n}")
        if k>0:
            L("PARKED ON GOAL")
            break
        # alternate tiny fwd/back around hit
        d=0.12*((-1)**i)*(i//2+1)
        cmd=3 if d>0 else -3
        wr("d4",cmd); wr("d5",cmd); time.sleep(abs(d)/0.26); stop(); time.sleep(0.2)
