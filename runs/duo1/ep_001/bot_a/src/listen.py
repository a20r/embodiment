import rob, walker, time, random
t0=time.time()
def rx():
    s=rob.rd(5,tries=1)
    return s
hits=0
while time.time()-t0<3300:
    if rob.goal(): print("GOAL!!!", flush=True); rob.motors(0,0); break
    s=rx()
    if s:
        hits+=1
        b=walker.bng()
        print(f"RX {s!r} b={b:.0f} t={time.time()-t0:.0f}", flush=True)
        rob.motors(0,0)
        # dwell: collect
        buf=''
        te=time.time()
        while time.time()-te<3:
            c=rx()
            if c: buf+=c; te=time.time()
            time.sleep(0.05)
        print("DWELL:", repr(buf), flush=True)
    walker.align()
    F,R,Lt,B=walker.look()
    opts=[(dd,(1.6 if dd==0 else 1.0)+d*0.2) for dd,d in ((0,F),(90,R),(-90,Lt)) if d>0.38]
    if not opts:
        walker.turn(180); continue
    dd=random.choices([o[0] for o in opts],weights=[o[1] for o in opts])[0]
    if dd: walker.turn(dd)
    # step with listening
    o0=rob.odo()
    while rob.odo()-o0<78:
        L=rob.lidar()
        if 0<L[0]<0.20: break
        s=rx()
        if s:
            rob.motors(0,0)
            print(f"RX-mid {s!r} b={walker.bng():.0f} t={time.time()-t0:.0f}", flush=True)
            buf=''; te=time.time()
            while time.time()-te<3:
                c=rx()
                if c: buf+=c; te=time.time()
                time.sleep(0.05)
            print("DWELL:", repr(buf), flush=True)
        r_,l_=L[4],L[12]
        c=0.0
        if 0<r_<0.45 and 0<l_<0.45: c=(r_-l_)*35
        elif 0<r_<0.28: c=(r_-0.18)*45
        elif 0<l_<0.28: c=-(l_-0.18)*45
        c=max(-7,min(7,c))
        rob.motors(20+c,20-c)
        time.sleep(0.05)
    rob.motors(0,0)
print("end hits=%d"%hits, flush=True)
