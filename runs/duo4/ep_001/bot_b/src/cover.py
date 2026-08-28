import time,math,random,os
DEV="/dev/robot/"
def rd(p):
    with open(DEV+p) as f: return f.readline().strip()
def wr(p,v):
    with open(DEV+p,"w") as f: f.write(str(v)+"\n")
log=open("/memory/run.log","a",buffering=1)
def L(*a): log.write(" ".join(map(str,a))+"\n")
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
x,y=0.0,0.0   # anchor frame
h=head() or 0
K=0.085
def upd(cmd,dt,hh):
    global x,y
    v=K*cmd
    x+=v*math.cos(math.radians(hh))*dt
    y+=v*math.sin(math.radians(hh))*dt
def rotate(delta):
    global h
    h0=head() or 0; tgt=h0+delta; t0=time.time()
    while time.time()-t0<7:
        hh=head() or 0
        e=ang(tgt-hh)
        if abs(e)<6: break
        sp=max(7,min(25,abs(e)*0.5))
        wr("d4",-sp if e>0 else sp); wr("d5",sp if e>0 else -sp)
        if rd("d7")=="1":
            stop(); L(f"COVER HIT while rotating at ({x:.2f},{y:.2f})"); return "hit"
        time.sleep(0.03)
    stop(); h=head() or 0
    return None
L(f"=== cover start {time.time()} ===")
t0=time.time(); hit=False
lasttx=0
while time.time()-t0<280 and not hit:
    if time.time()-lasttx>3:
        wr("d8","BETA: goal is near me, searching exact spot. Stay close."); lasttx=time.time()
    # pick open direction, biased toward anchor if far
    r=scan(); hh=head() or 0
    cands=[]
    for i in range(16):
        clr=min(r[i],r[(i+1)%16]*1.15,r[(i-1)%16]*1.15)
        if clr>0.5: cands.append((i,clr))
    if not cands:
        if rotate(90)=="hit": hit=True; break
        continue
    dist=math.hypot(x,y)
    if dist>2.2:
        want=math.degrees(math.atan2(-y,-x))
        i=min(cands,key=lambda c:abs(ang(hh+c[0]*22.5-want)))[0]
    else:
        i=random.choice(cands)[0]
    if rotate(i*22.5)=="hit": hit=True; break
    # drive slowly with fast polling
    cmd=5; wr("d4",cmd); wr("d5",cmd)
    tl=time.time(); tstart=tl
    maxt=min(2.5,(dict(cands)[i]-0.25)/ (K*cmd))
    while time.time()-tstart<maxt:
        now=time.time(); dt=now-tl; tl=now
        hh=head() or hh
        upd(cmd,dt,hh)
        if rd("d7")=="1":
            stop(); L(f"COVER HIT driving at ({x:.2f},{y:.2f})"); hit=True; break
        if rd("d0")=="1":
            stop(); wr("d4",-5); wr("d5",-5); time.sleep(0.4); stop(); upd(-5,0.4,hh); break
        rr=scan()
        if min(rr[0],rr[1]*1.05,rr[15]*1.05)<0.22: stop(); break
    stop()
L(f"cover end hit={hit} pos=({x:.2f},{y:.2f})")
if hit:
    # confirm stationary rate
    k=0;n=0;t0=time.time()
    while time.time()-t0<3:
        v=rd("d7"); n+=1; k+=(v=="1")
    L(f"stationary d7 {k}/{n}")
    while True:
        wr("d8","BETA: I AM AT THE GOAL. HOME ON MY BEACON AND COME HERE.")
        st=rd("d9")
        L(f"G2 {time.time():.0f} {st} d7={rd('d7')}")
        time.sleep(2)
