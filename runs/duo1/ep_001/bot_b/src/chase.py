import lib, time, math, random, collections
prev=[0.5]*16
def clean(l):
    global prev
    out=[(prev[i] if v<0 else v) for i,v in enumerate(l)]
    prev=out; return out
hist=collections.deque(maxlen=8)
def pingv():
    lib.write("d3","p"); time.sleep(0.12)
    r=lib.read("d5").strip()
    try:
        v=float(r); hist.append((time.time(),v)); return v
    except: return None
def slope():
    if len(hist)<4: return 0
    pts=list(hist)
    t0=pts[0][0]
    xs=[p[0]-t0 for p in pts]; ys=[p[1] for p in pts]
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    num=sum((a-mx)*(b-my) for a,b in zip(xs,ys)); den=sum((a-mx)**2 for a in xs)+1e-9
    return num/den
def checkgoal():
    g,gs=lib.goal()
    if g:
        lib.stop(); print("GOAL!!",gs,flush=True); exit()
t0=time.time()
lastv=None
while time.time()-t0<2700:
    checkgoal()
    l=clean(lib.lidar())
    v=pingv()
    cur=[h[1] for h in list(hist)[-3:]]
    vv=max(cur) if cur else -99
    s=slope()
    print(f"v={v} vv={vv:.1f} slope={s:.1f}",flush=True)
    # choose direction: open rays; prefer keep-current(0) if slope>0
    cands=[]
    for i in range(16):
        d=min(l[i], l[(i+1)%16]+0.35, l[(i-1)%16]+0.35)
        if d<0.6: continue
        w=min(d,2.5)
        if i==0 and s>1: w+=6
        if i in (7,8,9) and s>1: w-=3
        if i==0: w+=1.0
        cands.append((w+random.random()*0.8,i))
    if not cands:
        lib.wheels(-40,-40); time.sleep(0.6); lib.stop(); continue
    cands.sort(reverse=True)
    j=cands[0][1]
    delta=((22.5*j+180)%360)-180
    if abs(delta)>15: lib.turn_by(delta, speed=25)
    hold=lib.heading()
    t1=time.time()
    while time.time()-t1<4:
        checkgoal()
        ll=clean(lib.lidar())
        f=min(ll[0],ll[1]*1.25,ll[15]*1.25)
        if f<0.5: break
        v=pingv()
        s=slope()
        spd = 60 if (v is not None and v>38) else (120 if f>1.2 else 70)
        e2=((hold-lib.heading()+180)%360)-180
        c=max(-12,min(12,e2*0.6))
        lib.wheels(round(spd-c,1),round(spd+c,1))
        if s<-6 and len(hist)>=6: break   # getting colder fast
    lib.stop()
