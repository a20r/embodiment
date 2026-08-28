import lib, time, math, collections
prev=[0.5]*16
def clean(l):
    global prev
    out=[(prev[i] if v<0 else v) for i,v in enumerate(l)]
    prev=out; return out
def enc():
    try: return int(lib.read("d7")), int(lib.read("d8"))
    except: return (0,0)
last=None
for line in open("/memory/trail2.csv"): last=line
p=last.split(","); x,y=float(p[1]),float(p[2])
el=enc()
LOG=open("/memory/trail2.csv","a")
RLOG=open("/memory/radio.csv","a")
def odom():
    global x,y,el
    e=enc()
    d=((e[0]-el[0])+(e[1]-el[1]))/2.0/700.0
    h=math.radians(lib.heading())
    x+=d*math.cos(h); y+=d*math.sin(h); el=e
def log(l,gs):
    LOG.write(f"{time.time():.1f},{x:.2f},{y:.2f},{lib.heading():.1f},{','.join(f'{v:.2f}' for v in l)},{gs}\n"); LOG.flush()
samples=collections.deque(maxlen=40)
visits=collections.Counter()
def ping():
    try:
        lib.write("d3","ping")
    except: pass
    time.sleep(0.15)
    r=lib.read("d5").strip()
    if r:
        try:
            v=float(r)
            samples.append((x,y,v,time.time()))
            RLOG.write(f"{time.time():.1f},{x:.2f},{y:.2f},{v}\n"); RLOG.flush()
            print(f"RSSI {v} at {x:.2f},{y:.2f}",flush=True)
            return v
        except: print("RADIO?",r,flush=True)
    return None
def grad():
    if len(samples)<8: return None
    pts=[s for s in samples if time.time()-s[3]<120]
    if len(pts)<8: return None
    xs=[p[0] for p in pts]; ys=[p[1] for p in pts]; vs=[p[2] for p in pts]
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys); mv=sum(vs)/len(vs)
    sxx=sum((a-mx)**2 for a in xs)+1e-6; syy=sum((a-my)**2 for a in ys)+1e-6
    sxv=sum((a-mx)*(b-mv) for a,b in zip(xs,vs)); syv=sum((a-my)*(b-mv) for a,b in zip(ys,vs))
    gx=sxv/sxx; gy=syv/syy
    n=math.hypot(gx,gy)
    if n<0.5: return None
    return math.degrees(math.atan2(gy,gx))%360
t0=time.time()
while time.time()-t0<2800:
    l=clean(lib.lidar())
    g,gs=lib.goal()
    if g: lib.stop(); print("GOAL",gs,flush=True); break
    odom(); log(l,gs)
    ping()
    visits[(round(x/0.6),round(y/0.6))]+=1
    h0=lib.heading()
    gdir=grad()
    best=None
    for i in range(16):
        d=min(l[i], l[(i+1)%16]+0.35, l[(i-1)%16]+0.35)
        if d<0.55: continue
        absd=(h0+22.5*i)%360
        a=math.radians(absd)
        pen=0.0
        for s in (0.6,1.2,1.8):
            if s>d: break
            c=(round((x+s*math.cos(a))/0.6), round((y+s*math.sin(a))/0.6))
            pen+=min(visits[c],6)*0.3
        score=min(d,2.2)-pen
        if gdir is not None:
            rel=abs(((absd-gdir+180)%360)-180)
            score += (90-rel)/30.0
        if best is None or score>best[0]: best=(score,i)
    if best is None:
        lib.turn_by(160); continue
    j=best[1]
    delta=((22.5*j+180)%360)-180
    if abs(delta)>12: lib.turn_by(delta)
    lib.wheels(42,42)
    t1=time.time(); hold=lib.heading()
    while time.time()-t1<7:
        ll=clean(lib.lidar())
        g,gs=lib.goal()
        if g: lib.stop(); print("GOAL",gs,flush=True); exit()
        odom(); log(ll,gs)
        ping()
        f=min(ll[0],ll[1]*1.3,ll[15]*1.3)
        if f<0.45: break
        e2=((hold-lib.heading()+180)%360)-180
        c=max(-8,min(8,e2*0.5))
        lib.wheels(round(42-c,1),round(42+c,1))
        time.sleep(0.1)
    lib.stop()
lib.stop(); print("end",flush=True)
