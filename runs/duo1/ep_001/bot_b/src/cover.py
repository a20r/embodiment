import lib, time, math, random, collections
prev=[0.5]*16
def clean(l):
    global prev
    out=[(prev[i] if q<0 else q) for i,q in enumerate(l)]
    prev=out; return out
def enc():
    try: return int(lib.read("d7")),int(lib.read("d8"))
    except: return (0,0)
x,y=0.0,0.0; el=enc()
def odom():
    global x,y,el
    e=enc()
    d=((e[0]-el[0])+(e[1]-el[1]))/2.0/700.0
    h=math.radians(lib.heading())
    x+=d*math.cos(h); y+=d*math.sin(h); el=e
def checkgoal():
    g,gs=lib.goal()
    if g:
        lib.stop(); print("GOAL!!",gs,flush=True)
        open("/memory/GOAL.txt","a").write(gs+"\n"); exit()
hist=collections.deque(maxlen=10)
def pingv():
    lib.write("d3","p"); time.sleep(0.08)
    r=lib.read("d5").strip()
    try:
        v=float(r); hist.append((time.time(),v)); return v
    except: return None
def slope():
    pts=[p for p in hist if time.time()-p[0]<4]
    if len(pts)<4: return 0
    t0=pts[0][0]; xs=[p[0]-t0 for p in pts]; ys=[p[1] for p in pts]
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    return sum((a-mx)*(b-my) for a,b in zip(xs,ys))/(sum((a-mx)**2 for a in xs)+1e-9)
visits=collections.Counter()
TR=open("/memory/cover_trail.csv","a")
HOT=open("/memory/hot.csv","a")
targets=[90,45,0,315,270,225,180,135]  # N NE E SE S SW W NW
PHASE=150.0
t0=time.time()
laste=enc(); lastmove=time.time()
while time.time()-t0<3400:
    checkgoal()
    e=enc()
    if abs(e[0]-laste[0])+abs(e[1]-laste[1])>20: laste=e; lastmove=time.time()
    elif time.time()-lastmove>3:
        lib.wheels(-80,-80); time.sleep(0.9)
        s=random.choice([-1,1]); lib.wheels(30*s,-30*s); time.sleep(0.6)
        lib.stop(); laste=enc(); lastmove=time.time(); continue
    l=clean(lib.lidar())
    odom()
    v=pingv(); sl=slope()
    if v is not None:
        HOT.write(f"{time.time():.1f},{x:.2f},{y:.2f},{v}\n"); HOT.flush()
    TR.write(f"{time.time():.1f},{x:.2f},{y:.2f},{lib.heading():.1f},{','.join(f'{q:.2f}' for q in l)}\n"); TR.flush()
    visits[(round(x/0.6),round(y/0.6))]+=1
    h0=lib.heading()
    insig = bool(hist) and time.time()-hist[-1][0]<3
    tgt=targets[int((time.time()-t0)/PHASE)%8]
    best=None
    for i in range(16):
        d=min(l[i], l[(i+1)%16]+0.35, l[(i-1)%16]+0.35)
        if d<0.55: continue
        absd=(h0+22.5*i)%360
        a=math.radians(absd)
        w=min(d,2.2)+random.random()*0.4
        if insig:
            if i==0 and sl>-1: w+=6
            if i in (7,8,9) and sl<-3: w+=4
        else:
            pass
            pen=0.0
            for st in (0.6,1.2,1.8):
                if st>d: break
                c=(round((x+st*math.cos(a))/0.6),(round((y+st*math.sin(a))/0.6)))
                pen+=min(visits[c],6)*0.35
            w-=pen
        if best is None or w>best[0]: best=(w,i)
    if best is None:
        lib.wheels(-60,-60); time.sleep(0.6); lib.stop(); continue
    j=best[1]
    delta=((22.5*j+180)%360)-180
    if abs(delta)>15: lib.turn_by(delta,speed=28)
    hold=lib.heading(); t1=time.time()
    while time.time()-t1<5:
        checkgoal()
        ll=clean(lib.lidar())
        f=min(ll[0],ll[1]*1.25,ll[15]*1.25)
        if f<0.5: break
        v=pingv()
        odom()
        if v is not None:
            HOT.write(f"{time.time():.1f},{x:.2f},{y:.2f},{v}\n"); HOT.flush()
        TR.write(f"{time.time():.1f},{x:.2f},{y:.2f},{lib.heading():.1f},{','.join(f'{q:.2f}' for q in ll)}\n"); TR.flush()
        sl=slope()
        spd=120 if f>1.3 else 70
        e2=((hold-lib.heading()+180)%360)-180
        c=max(-12,min(12,e2*0.6))
        lib.wheels(round(spd-c,1),round(spd+c,1))
        if v is not None and sl<-8: break
    lib.stop()
print("cover done",flush=True)
