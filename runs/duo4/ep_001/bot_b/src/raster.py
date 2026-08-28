import time,math
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
    for _ in range(3):
        try: return float(rd("d2"))
        except: pass
    return None
def ang(a): return (a+180)%360-180
x,y=0.0,0.0; K=0.085
hits=[]
def rotate(delta):
    h0=head() or 0; tgt=h0+delta; t0=time.time()
    while time.time()-t0<8:
        hh=head() or 0
        e=ang(tgt-hh)
        if abs(e)<5: break
        sp=max(6,min(22,abs(e)*0.5))
        wr("d4",-sp if e>0 else sp); wr("d5",sp if e>0 else -sp)
        if rd("d7")=="1":
            L(f"RHIT rot ({x:.2f},{y:.2f})"); hits.append((x,y))
        time.sleep(0.03)
    stop()
def drive(cmd,maxd,maxt):
    global x,y
    hh=head() or 0
    wr("d4",cmd); wr("d5",cmd)
    t0=time.time(); tl=t0; trav=0; i=0
    while time.time()-t0<maxt and abs(trav)<maxd:
        now=time.time(); dt=now-tl; tl=now
        v=K*cmd; trav+=abs(v)*dt
        x+=v*math.cos(math.radians(hh))*dt
        y+=v*math.sin(math.radians(hh))*dt
        if rd("d7")=="1":
            L(f"RHIT drv ({x:.2f},{y:.2f}) trav={trav:.2f}"); hits.append((x,y))
        i+=1
        if i%8==0:
            hh=head() or hh
            r=scan()
            if cmd>0 and min(r[0],r[1]*1.05,r[15]*1.05)<0.20: break
            if rd("d0")=="1":
                stop(); wr("d4",-4); wr("d5",-4); time.sleep(0.4); stop(); return "bump"
    stop()
    return None
L(f"=== raster start {time.time()} ===")
# pattern: go up and down the corridor 3 times at slow speed; slight zigzag via
# alternating small heading offsets to cover corridor width
for lap in range(3):
    off=(-14,0,14)[lap]
    rotate(off if lap else 0)
    drive(3,3.0,16)
    L(f"lap{lap}A pos=({x:.2f},{y:.2f}) hits={len(hits)}")
    rotate(180)
    drive(3,3.0,16)
    L(f"lap{lap}B pos=({x:.2f},{y:.2f}) hits={len(hits)}")
    rotate(180)
L(f"raster end hits={hits}")
if hits:
    hx=sum(p[0] for p in hits)/len(hits); hy=sum(p[1] for p in hits)/len(hits)
    L(f"going to hit centroid ({hx:.2f},{hy:.2f})")
    hh=head() or 0
    want=math.degrees(math.atan2(hy-y,hx-x))
    rotate(ang(want-hh))
    d=math.hypot(hx-x,hy-y)
    drive(3,d,int(d/0.25)+3)
    stop()
    k=0;n=0;t0=time.time()
    while time.time()-t0<4:
        v=rd("d7"); n+=1; k+=(v=="1")
    L(f"parked at ({x:.2f},{y:.2f}) stationary d7 {k}/{n} d9={rd('d9')}")
    while True:
        wr("d8","BETA: I AM AT THE GOAL AREA. COME TO MY BEACON.")
        L(f"G3 {rd('d9')} d7={rd('d7')}")
        time.sleep(3)
