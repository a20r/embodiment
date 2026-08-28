import rob, walker, time, random
t0=time.time()
def snifftx(n=[0]):
    n[0]+=1
    if n[0]%8==0:
        try:
            rob.wr(6,"hello")
            s=rob.rd(5,tries=1); s9=rob.rd(9,tries=1)
            if s or (s9 not in ("0","")):
                print(f"RADIO d5='{s}' d9='{s9}' b={walker.bng():.0f}", flush=True)
        except Exception: pass
while time.time()-t0<3000:
    if rob.goal(): print("GOAL!!!", flush=True); rob.motors(0,0); break
    snifftx()
    walker.align()
    F,R,Lt,B=walker.look()
    b=walker.bng()
    opts=[]
    for dd,dist in ((0,F),(90,R),(-90,Lt)):
        if dist>0.38: opts.append((dd, (1.8 if dd==0 else 1.0)+dist*0.25))
    if not opts:
        walker.turn(180); continue
    dd=random.choices([o[0] for o in opts],weights=[o[1] for o in opts])[0]
    if dd: walker.turn(dd)
    r=walker.step(spd=26)
    if random.random()<0.15:
        print(f"t={time.time()-t0:.0f} b={b:.0f} F={F:.2f} R={R:.2f} L={Lt:.2f} {dd} {r}", flush=True)
print("end", flush=True)
