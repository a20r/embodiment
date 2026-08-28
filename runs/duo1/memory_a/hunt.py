import rob, walker, time
def ad(x): return (x+180)%360-180
t0=time.time()
rxlog=[]
while time.time()-t0<280:
    if rob.goal(): print("GOAL!!!", flush=True); break
    b=walker.bng()
    s=rob.rd(5,tries=1)
    if s:
        print(f"RX '{s}' b={b:.0f} t={time.time()-t0:.0f}", flush=True)
        rob.motors(0,0)
        # sit and listen
        buf=''
        te=time.time()
        while time.time()-te<6:
            c=rob.rd(5,tries=1)
            if c: buf+=c; te=time.time()
            time.sleep(0.1)
        print("BUF:", repr(buf), flush=True)
        if buf:
            rob.wr(6,"hello")
            time.sleep(0.5)
            buf2=''
            te=time.time()
            while time.time()-te<5:
                c=rob.rd(5,tries=1)
                if c: buf2+=c; te=time.time()
                time.sleep(0.1)
            print("REPLY:", repr(buf2), flush=True)
    # keep orbiting ring: maintain b~90 like orbit
    L=rob.lidar()
    e=ad(b-90)
    c=max(-12,min(12,0.5*e))
    if 0<L[0]<0.25:
        walker.turn(-e)  # re-point tangential
    else:
        rob.motors(14+c,14-c)
    time.sleep(0.08)
rob.motors(0,0)
print("end", flush=True)
