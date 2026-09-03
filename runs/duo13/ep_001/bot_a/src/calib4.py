import sys, time, math; sys.path.insert(0,'/bot/src')
from drive import Driver, wrap
d=Driver(); r=d.r
def enc2():
    best=None
    for _ in range(6):
        e=r.enc()
        if e[0] is not None and e[1] is not None: return e
        time.sleep(0.04)
    return None
r.motors(0,0); time.sleep(0.3)
d.turnto(263, tol=3); time.sleep(0.2)
pts=[]; e0=enc2()
t0=time.time()
while time.time()-t0<14:
    r.motors(0,0); time.sleep(0.12)
    rg=r.ranges(); f=rg[0] if rg else None
    e=enc2()
    if f is None or e is None: continue
    if f<0.6: break
    ticks=((e[0]-e0[0])+(e[1]-e0[1]))/2.0
    pts.append((f,ticks))
    r.motors(70,70); time.sleep(0.25)
r.motors(0,0)
print("final front=",(r.ranges() or [9])[0])
if len(pts)>4:
    xs=[p[1] for p in pts]; ys=[p[0] for p in pts]
    n=len(pts); mx=sum(xs)/n; my=sum(ys)/n
    num=sum((x-mx)*(y-my) for x,y in pts); den=sum((x-mx)**2 for x in xs)
    k=-num/den if den else 0
    print(f"meters_per_tick={k:.6f} (={1/k if k else 0:.1f} ticks/m) n={n} span_ticks={xs[-1]-xs[0]:.0f} span_m={ys[0]-ys[-1]:.3f}")
    print("pts:", [(round(x),round(t,3)) for t,x in pts[::3]])
else:
    print("too few pts", pts)
