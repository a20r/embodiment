import lib, time, math
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
def odom():
    global x,y,el
    e=enc()
    d=((e[0]-el[0])+(e[1]-el[1]))/2.0/700.0
    h=math.radians(lib.heading())
    x+=d*math.cos(h); y+=d*math.sin(h); el=e
    return x,y
t0=time.time()
lastdir=None
while time.time()-t0<3000:
    l=clean(lib.lidar())
    g,gs=lib.goal()
    if g: lib.stop(); print("GOAL",gs,flush=True); break
    r=lib.read("d5")
    if r.strip(): print("RADIO:",r,flush=True)
    ox,oy=odom()
    LOG.write(f"{time.time():.1f},{ox:.2f},{oy:.2f},{lib.heading():.1f},{','.join(f'{v:.2f}' for v in l)},{gs}\n"); LOG.flush()
    # choose direction: maximize distance, penalize going back the way we came
    best=None
    for i in range(16):
        d=l[i]
        score=d
        if lastdir is not None:
            rel=abs(((22.5*i - lastdir +180)%360)-180)
            if rel>120: score-=1.0   # penalty for reversing
        if best is None or score>best[0]: best=(score,i)
    j=best[1]
    absdir=(lib.heading()+22.5*j)%360
    lastdir=(lib.heading()+22.5*j)%360
    delta=((22.5*j+180)%360)-180
    if abs(delta)>12:
        lib.turn_by(delta)
        l=clean(lib.lidar())
    # drive forward until front shrinks
    dist=l[0]
    lib.wheels(40,40)
    t1=time.time(); hold=(lib.heading())
    while time.time()-t1<12:
        ll=clean(lib.lidar())
        g,gs=lib.goal()
        if g: lib.stop(); print("GOAL",gs,flush=True); exit()
        f=min(ll[0],ll[1]*1.3,ll[15]*1.3)
        if f<0.45: break
        # heading hold
        e=((hold-lib.heading()+180)%360)-180
        c=max(-8,min(8,e*0.5))
        lib.wheels(round(40-c,1),round(40+c,1))
        ox,oy=odom()
        LOG.write(f"{time.time():.1f},{ox:.2f},{oy:.2f},{lib.heading():.1f},{','.join(f'{v:.2f}' for v in ll)},{gs}\n"); LOG.flush()
        time.sleep(0.15)
    lib.stop()
lib.stop(); print("done",flush=True)
