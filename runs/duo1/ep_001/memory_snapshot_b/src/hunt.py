import lib, time, math, random, collections
prev=[0.5]*16
def clean(l):
    global prev
    out=[(prev[i] if v<0 else v) for i,v in enumerate(l)]
    prev=out; return out
def enc():
    try: return int(lib.read("d7")), int(lib.read("d8"))
    except: return (0,0)
x,y=0.0,0.0; el=enc()
def odom():
    global x,y,el
    e=enc()
    d=((e[0]-el[0])+(e[1]-el[1]))/2.0/700.0
    h=math.radians(lib.heading())
    x+=d*math.cos(h); y+=d*math.sin(h); el=e
hist=collections.deque(maxlen=8)
_msgs=["stop","halt","freeze","come here","0","p"]
_mi=[0]
def pingv():
    m="p"
    if hist and time.time()-hist[-1][0]<3:
        m=_msgs[_mi[0]%len(_msgs)]; _mi[0]+=1
    lib.write("d3",m); time.sleep(0.1)
    r=lib.read("d5").strip()
    try:
        v=float(r); hist.append((time.time(),v)); return v
    except: return None
def slope():
    if len(hist)<4: return 0
    pts=[p for p in hist if time.time()-p[0]<4]
    if len(pts)<4: return 0
    t0=pts[0][0]
    xs=[p[0]-t0 for p in pts]; ys=[p[1] for p in pts]
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    num=sum((a-mx)*(b-my) for a,b in zip(xs,ys)); den=sum((a-mx)**2 for a in xs)+1e-9
    return num/den
def checkgoal():
    g,gs=lib.goal()
    if g:
        lib.stop(); print("GOAL!!",gs,flush=True)
        open("/memory/GOAL.txt","a").write(gs+"\n")
        exit()
visits=collections.Counter()
HOT=open("/memory/hot.csv","a")
TR=open("/memory/hunt_trail.csv","a")
t0=time.time()
laste=enc(); lastmove=time.time()
while time.time()-t0<2700:
    checkgoal()
    e=enc()
    if abs(e[0]-laste[0])+abs(e[1]-laste[1])>20:
        laste=e; lastmove=time.time()
    elif time.time()-lastmove>3:
        print("STUCK recover",flush=True)
        lib.wheels(-70,-70); time.sleep(1.0)
        import random as _r
        s=_r.choice([-1,1]); lib.wheels(25*s,-25*s); time.sleep(0.8)
        lib.stop(); laste=enc(); lastmove=time.time()
        continue
    l=clean(lib.lidar())
    odom()
    v=pingv()
    s=slope()
    if v is not None:
        HOT.write(f"{time.time():.1f},{x:.2f},{y:.2f},{v}\n"); HOT.flush()
    visits[(round(x/0.6),round(y/0.6))]+=1
    TR.write(f"{time.time():.1f},{x:.2f},{y:.2f},{lib.heading():.1f},{','.join(f'{q:.2f}' for q in l)}\n"); TR.flush()
    insig = v is not None
    h0=lib.heading()
    best=None
    for i in range(16):
        d=min(l[i], l[(i+1)%16]+0.35, l[(i-1)%16]+0.35)
        if d<0.6: continue
        a=math.radians(h0+22.5*i)
        w=min(d,2.4)+random.random()*0.5
        if not insig:
            pen=0.0
            for st in (0.6,1.2,1.8):
                if st>d: break
                c=(round((x+st*math.cos(a))/0.6), round((y+st*math.sin(a))/0.6))
                pen+=min(visits[c],6)*0.3
            w-=pen
        else:
            if i==0 and s>0: w+=5
            if i in (6,7,8,9,10) and s>0: w-=2
        if best is None or w>best[0]: best=(w,i)
    if best is None:
        lib.wheels(-50,-50); time.sleep(0.5); lib.stop(); continue
    j=best[1]
    delta=((22.5*j+180)%360)-180
    if abs(delta)>15: lib.turn_by(delta,speed=25)
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
        s=slope()
        spd = 65 if (v is not None and v>35) else (110 if f>1.3 else 65)
        e2=((hold-lib.heading()+180)%360)-180
        c=max(-12,min(12,e2*0.6))
        lib.wheels(round(spd-c,1),round(spd+c,1))
        if v is not None and s<-8: break
    lib.stop()
    if v is not None and v>45:
        print(f"very close v={v} at {x:.2f},{y:.2f}",flush=True)
