import rob, walker, time, random
def ad(x): return (x+180)%360-180
t0=time.time()
def probe():
    rob.wr(6,"hello")
    out=''
    for _ in range(25):
        s=rob.rd(5,tries=1)
        if s: out+=s
    return out
while time.time()-t0<2400:
    if rob.goal(): print("GOAL!!!", flush=True); rob.motors(0,0); break
    r=probe()
    if r: print(f"HIT {r!r} b={walker.bng():.0f} t={time.time()-t0:.0f}", flush=True)
    walker.align()
    F,R,Lt,B=walker.look()
    b=walker.bng()
    opts=[]
    for dd,dist in ((0,F),(90,R),(-90,Lt)):
        if dist<0.38: continue
        aw=abs(ad(b+dd))  # 180 = away from beacon
        opts.append((dd, 0.5+ aw/180*2.0 + dist*0.2))
    if not opts:
        walker.turn(180); continue
    dd=random.choices([o[0] for o in opts],weights=[o[1] for o in opts])[0]
    if dd: walker.turn(dd)
    walker.step(spd=26)
print("end", flush=True)
