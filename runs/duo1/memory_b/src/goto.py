import lib, time, math, sys, collections
TX,TY=float(sys.argv[1]),float(sys.argv[2])
# resume odometry from last trail2 pose
last=None
for line in open("/memory/trail2.csv"):
    last=line
p=last.split(","); x,y=float(p[1]),float(p[2])
prev=[0.5]*16
def clean(l):
    global prev
    out=[(prev[i] if v<0 else v) for i,v in enumerate(l)]
    prev=out; return out
def enc():
    try: return int(lib.read("d7")), int(lib.read("d8"))
    except: return (0,0)
el=enc()
LOG=open("/memory/trail2.csv","a")
def odom():
    global x,y,el
    e=enc()
    d=((e[0]-el[0])+(e[1]-el[1]))/2.0/700.0
    h=math.radians(lib.heading())
    x+=d*math.cos(h); y+=d*math.sin(h); el=e
def log(l,gs):
    LOG.write(f"{time.time():.1f},{x:.2f},{y:.2f},{lib.heading():.1f},{','.join(f'{v:.2f}' for v in l)},{gs}\n"); LOG.flush()
visits=collections.Counter()
t0=time.time()
while time.time()-t0<600:
    l=clean(lib.lidar())
    g,gs=lib.goal()
    if g: lib.stop(); print("GOAL",gs,flush=True); break
    r=lib.read("d5")
    if r.strip(): print("RADIO:",r,flush=True)
    odom(); log(l,gs)
    if (x-TX)**2+(y-TY)**2<0.36:
        lib.stop(); print("ARRIVED",x,y,flush=True); break
    visits[(round(x/0.6),round(y/0.6))]+=1
    h0=lib.heading()
    tdir=math.degrees(math.atan2(TY-y,TX-x))%360
    best=None
    for i in range(16):
        d=min(l[i], l[(i+1)%16]+0.35, l[(i-1)%16]+0.35)
        if d<0.55: continue
        absd=(h0+22.5*i)%360
        rel=abs(((absd-tdir+180)%360)-180)
        a=math.radians(absd)
        pen=0.0
        for s in (0.6,1.2,1.8):
            if s>d: break
            c=(round((x+s*math.cos(a))/0.6), round((y+s*math.sin(a))/0.6))
            pen+=min(visits[c],5)*0.25
        score = -rel/45.0 + min(d,2.0) - pen
        if best is None or score>best[0]: best=(score,i)
    if best is None:
        lib.turn_by(160); continue
    j=best[1]
    delta=((22.5*j+180)%360)-180
    if abs(delta)>12: lib.turn_by(delta)
    lib.wheels(40,40)
    t1=time.time(); hold=lib.heading()
    while time.time()-t1<6:
        ll=clean(lib.lidar())
        g,gs=lib.goal()
        if g: lib.stop(); print("GOAL",gs,flush=True); exit()
        odom(); log(ll,gs)
        if (x-TX)**2+(y-TY)**2<0.36:
            lib.stop(); print("ARRIVED",x,y,flush=True); exit()
        f=min(ll[0],ll[1]*1.3,ll[15]*1.3)
        if f<0.45: break
        e2=((hold-lib.heading()+180)%360)-180
        c=max(-8,min(8,e2*0.5))
        lib.wheels(round(40-c,1),round(40+c,1))
        time.sleep(0.15)
    lib.stop()
lib.stop(); print("end",x,y,flush=True)
