import rob, walker, time, random
def ad(x): return (x+180)%360-180
bng=walker.bng
t0=time.time()
lasttx=[0]
def chat():
    now=time.time()
    if now-lasttx[0]>0.8:
        rob.wr(6,"hello"); lasttx[0]=now
    s=rob.rd(5,tries=1)
    return s
def dwell_and_talk():
    print("=== CONTACT ZONE b=%.0f t=%.0f"%(bng(), time.time()-t0), flush=True)
    print("lidar:", [f"{x:.2f}" for x in rob.lidar()], flush=True)
    for msg in ("hello","ping","pong","marco","who","help","goal","open","status"):
        rob.wr(6,msg)
        buf=''; te=time.time()
        while time.time()-te<2.5:
            c=rob.rd(5,tries=1)
            if c: buf+=c+'|'; te=time.time()
            time.sleep(0.05)
        print(f"  {msg} -> {buf!r}", flush=True)
while time.time()-t0<3300:
    if rob.goal(): print("GOAL!!!", flush=True); rob.motors(0,0); break
    s=chat()
    if s:
        rob.motors(0,0)
        print(f"RX {s!r}", flush=True)
        dwell_and_talk()
    walker.align()
    F,R,Lt,B=walker.look()
    b=bng()
    # bias toward beacon-adjacent pockets: prefer dirs pointing goalward-ish
    opts=[]
    for dd,dist in ((0,F),(90,R),(-90,Lt)):
        if dist<0.38: continue
        nb=abs(ad(b+dd))
        opts.append((dd, 0.6+(180-nb)/180*1.2+dist*0.15))
    if not opts:
        walker.turn(180); continue
    dd=random.choices([o[0] for o in opts],weights=[o[1] for o in opts])[0]
    if dd: walker.turn(dd)
    o0=rob.odo()
    while rob.odo()-o0<78:
        L=rob.lidar()
        if 0<L[0]<0.20: break
        s=chat()
        if s:
            rob.motors(0,0)
            print(f"RX-mid {s!r}", flush=True)
            dwell_and_talk()
            break
        r_,l_=L[4],L[12]
        c=0.0
        if 0<r_<0.45 and 0<l_<0.45: c=(r_-l_)*35
        elif 0<r_<0.28: c=(r_-0.18)*45
        elif 0<l_<0.28: c=-(l_-0.18)*45
        c=max(-7,min(7,c))
        rob.motors(18+c,18-c)
        time.sleep(0.05)
    rob.motors(0,0)
print("end", flush=True)
