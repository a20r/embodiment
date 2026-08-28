import rob, walker, time, random
t0=time.time()
def probe():
    rob.wr(6,"hello")
    time.sleep(0.15)
    out=''
    for _ in range(40):
        s=rob.rd(5,tries=1)
        if s: out+=s
    return out
while time.time()-t0<2000:
    if rob.goal(): print("GOAL!!!", flush=True); rob.motors(0,0); break
    r=probe()
    if r:
        b=walker.bng(); L=rob.lidar()
        print(f"HIT '{r}' b={b:.0f} t={time.time()-t0:.0f} F={L[0]:.2f}", flush=True)
        # stay and interrogate
        for msg in ("hello","ping","help","open","who","goal","where"):
            rob.wr(6,msg); time.sleep(0.3)
            out=''
            for _ in range(60):
                s=rob.rd(5,tries=1)
                if s: out+=s
            print(f"  {msg} -> {out!r}", flush=True)
    walker.align()
    F,R,Lt,B=walker.look()
    opts=[(dd,(1.8 if dd==0 else 1.0)+d*0.25) for dd,d in ((0,F),(90,R),(-90,Lt)) if d>0.38]
    if not opts:
        walker.turn(180); continue
    dd=random.choices([o[0] for o in opts],weights=[o[1] for o in opts])[0]
    if dd: walker.turn(dd)
    walker.step(spd=26)
print("end", flush=True)
