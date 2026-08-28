import rob, walker, time
def ad(x): return (x+180)%360-180
bng=walker.bng
t0=time.time()
def listen(dur=3):
    buf=''
    te=time.time()
    while time.time()-te<dur:
        c=rob.rd(5,tries=1)
        if c: buf+=c; te=time.time()
        time.sleep(0.08)
    return buf
# 1) ride the ring until the corner signature: F<0.35 while b in [10,40]
found=False
while time.time()-t0<200 and not found:
    if rob.goal(): print("GOAL!!!", flush=True); break
    b=bng(); L=rob.lidar()
    if 0<L[0]<0.32 and 5<=b<=45:
        rob.motors(0,0); found=True; break
    e=ad(b-90)
    c=max(-12,min(12,0.5*e))
    if 0<L[0]<0.25:
        walker.turn(-e)
    else:
        rob.motors(16+c,16-c)
    time.sleep(0.07)
rob.motors(0,0)
print("corner found b=%.0f F=%.2f"%(bng(), rob.lidar()[0]), flush=True)
# 2) turn right (=turn +90) and step through the marginal opening
walker.turn(90)
print("faced opening F=%.2f b=%.0f"%(rob.lidar()[0], bng()), flush=True)
r=walker.step(spd=14)
print("step1", r, "b=%.0f"%bng(), "RX:", repr(listen(2)), flush=True)
# 3) continue: try steps forward, listening; prefer moving while RX active
for k in range(12):
    if rob.goal(): print("GOAL!!!", flush=True); break
    walker.align()
    F,R,Lt,B=walker.look()
    b=bng()
    print(f"k={k} b={b:.0f} F={F:.2f} R={R:.2f} L={Lt:.2f}", flush=True)
    rx=listen(1.5)
    if rx: print("RX:", repr(rx), flush=True)
    # choose: prefer goalward-ish open dir
    best=None
    for dd,dist in ((0,F),(90,R),(-90,Lt)):
        if dist<0.38: continue
        nb=abs(ad(b+dd))
        sc=nb
        if best is None or sc<best[0]: best=(sc,dd)
    if best is None:
        walker.turn(180); walker.step(spd=14); continue
    if best[1]: walker.turn(best[1])
    walker.step(spd=14)
print("end b=%.0f"%bng(), flush=True)
