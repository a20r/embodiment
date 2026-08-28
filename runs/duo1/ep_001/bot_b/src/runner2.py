import lib, time, math, collections
prev=[0.5]*16
def clean(l):
    global prev
    out=[(prev[i] if v<0 else v) for i,v in enumerate(l)]
    prev=out; return out
def enc():
    try: return int(lib.read("d7")), int(lib.read("d8"))
    except: return (0,0)
LOG=open("/memory/trail2.csv","a")
x,y=0.0,0.0; el=enc()
visits=collections.Counter()
def odom():
    global x,y,el
    e=enc()
    d=((e[0]-el[0])+(e[1]-el[1]))/2.0/700.0
    h=math.radians(lib.heading())
    x+=d*math.cos(h); y+=d*math.sin(h); el=e
    return x,y
def log(l,gs):
    LOG.write(f"{time.time():.1f},{x:.2f},{y:.2f},{lib.heading():.1f},{','.join(f'{v:.2f}' for v in l)},{gs}\n"); LOG.flush()
CELL=0.6
t0=time.time()
lastdir=None
while time.time()-t0<2800:
    l=clean(lib.lidar())
    g,gs=lib.goal()
    if g: lib.stop(); print("GOAL",gs,flush=True); break
    r=lib.read("d5")
    if r.strip(): print("RADIO:",r,flush=True)
    odom(); log(l,gs)
    visits[(round(x/CELL),round(y/CELL))]+=1
    h0=lib.heading()
    best=None
    for i in range(16):
        d=min(l[i], l[(i+1)%16]+0.4, l[(i-1)%16]+0.4)
        if d<0.5: continue
        a=math.radians(h0+22.5*i)
        score=min(d,2.5)
        # novelty: check cells along ray
        pen=0.0
        for s in (0.6,1.2,1.8,2.4):
            if s>d: break
            c=(round((x+s*math.cos(a))/CELL), round((y+s*math.sin(a))/CELL))
            pen+=min(visits[c],6)*0.35
        score-=pen
        if lastdir is not None:
            rel=abs(((h0+22.5*i-lastdir+180)%360)-180)
            if rel>135: score-=1.5
        if best is None or score>best[0]: best=(score,i)
    if best is None:
        lib.turn_by(180); lastdir=None; continue
    j=best[1]
    lastdir=(h0+22.5*j)%360
    delta=((22.5*j+180)%360)-180
    if abs(delta)>12: lib.turn_by(delta)
    lib.wheels(40,40)
    t1=time.time(); hold=lib.heading()
    while time.time()-t1<10:
        ll=clean(lib.lidar())
        g,gs=lib.goal()
        if g: lib.stop(); print("GOAL",gs,flush=True); exit()
        f=min(ll[0],ll[1]*1.3,ll[15]*1.3)
        odom(); log(ll,gs)
        visits[(round(x/CELL),round(y/CELL))]+=1
        if f<0.45: break
        e=((hold-lib.heading()+180)%360)-180
        c=max(-8,min(8,e*0.5))
        lib.wheels(round(40-c,1),round(40+c,1))
        time.sleep(0.15)
    lib.stop()
lib.stop(); print("done",flush=True)
