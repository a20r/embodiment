import lib, time, math, random, collections
prev=[0.5]*16
def clean(l):
    global prev
    out=[(prev[i] if q<0 else q) for i,q in enumerate(l)]
    prev=out; return out
hist=collections.deque(maxlen=10)
def pingv():
    lib.write("d3","p"); time.sleep(0.1)
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
def checkgoal():
    g,gs=lib.goal()
    if g:
        lib.stop(); print("GOAL!!",gs,flush=True)
        open("/memory/GOAL.txt","a").write(gs+"\n"); exit()
def enc():
    try: return int(lib.read("d7")),int(lib.read("d8"))
    except: return (0,0)
mode="west"
t0=time.time(); patrol_dir=90
laste=enc(); lastmove=time.time()
westfail=0
while time.time()-t0<2000:
    checkgoal()
    e=enc()
    if abs(e[0]-laste[0])+abs(e[1]-laste[1])>20: laste=e; lastmove=time.time()
    elif time.time()-lastmove>3:
        lib.wheels(-70,-70); time.sleep(1.0)
        s=random.choice([-1,1]); lib.wheels(30*s,-30*s); time.sleep(0.6)
        lib.stop(); laste=enc(); lastmove=time.time(); continue
    l=clean(lib.lidar())
    v=pingv(); s=slope()
    h0=lib.heading()
    insig = v is not None and (time.time()-hist[-1][0]<3 if hist else False)
    if insig:
        tgt=None  # chase mode: keep going if warming, else turn around
        print(f"contact v={v} slope={s:.1f}",flush=True)
        best=None
        for i in range(16):
            d=min(l[i], l[(i+1)%16]+0.35, l[(i-1)%16]+0.35)
            if d<0.55: continue
            w=min(d,2.0)+random.random()*0.3
            if i==0 and s>-1: w+=6
            if i in (7,8,9) and s<-3: w+=4
            if best is None or w>best[0]: best=(w,i)
    else:
        tgtdir = 180 if mode=="west" else patrol_dir
        best=None
        for i in range(16):
            d=min(l[i], l[(i+1)%16]+0.35, l[(i-1)%16]+0.35)
            if d<0.55: continue
            absd=(h0+22.5*i)%360
            rel=abs(((absd-tgtdir+180)%360)-180)
            w=min(d,2.0)-rel/40.0+random.random()*0.3
            if best is None or w>best[0]: best=(w,i)
        if best is not None:
            absd=(h0+22.5*best[1])%360
            rel=abs(((absd-180+180)%360)-180)
            if mode=="west":
                if rel>100: westfail+=1
                else: westfail=0
                if westfail>4:
                    mode="patrol"; print("switch to patrol",flush=True)
    if best is None:
        lib.wheels(-60,-60); time.sleep(0.6); lib.stop(); continue
    j=best[1]
    delta=((22.5*j+180)%360)-180
    if abs(delta)>15: lib.turn_by(delta,speed=25)
    hold=lib.heading(); t1=time.time()
    while time.time()-t1<5:
        checkgoal()
        ll=clean(lib.lidar())
        f=min(ll[0],ll[1]*1.25,ll[15]*1.25)
        if f<0.5: break
        v=pingv(); s=slope()
        spd=110 if f>1.2 else 65
        e2=((hold-lib.heading()+180)%360)-180
        c=max(-12,min(12,e2*0.6))
        lib.wheels(round(spd-c,1),round(spd+c,1))
        if v is not None and s<-8: break
    lib.stop()
    if mode=="patrol" and not insig:
        # bounce patrol N/S
        if min(l[0],0.5+0)<0.6 or random.random()<0.15:
            patrol_dir = 270 if patrol_dir==90 else 90
