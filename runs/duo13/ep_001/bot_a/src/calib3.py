import sys, time, math; sys.path.insert(0,'/bot/src')
from drive import Driver, wrap
d=Driver(); r=d.r
def enc2():
    for _ in range(5):
        e=r.enc()
        if e[0] is not None and e[1] is not None: return e
        time.sleep(0.05)
    return None
r.motors(0,0); time.sleep(0.3)
d.turnto(95, tol=3); time.sleep(0.2)
print("h=",r.heading())
pts=[]; t0=time.time(); e0=enc2()
while time.time()-t0<9:
    rg=r.ranges(); f=rg[0] if rg else None
    if f is not None and 0<f<0.55: break
    err=wrap(95-(r.heading() or 95))
    a=int(60+1.2*err); b=int(60-1.2*err)
    r.motors(a,b); time.sleep(0.08)
    e=enc2()
    if f is not None and f>0 and e: pts.append((f,((e[0]-e0[0])+(e[1]-e0[1]))/2.0))
r.motors(0,0)
print("final front=", (r.ranges() or [None])[0], "enc=", enc2())
if len(pts)>4:
    xs=[p[1] for p in pts]; ys=[p[0] for p in pts]
    n=len(pts); mx=sum(xs)/n; my=sum(ys)/n
    num=sum((x-mx)*(y-my) for x,y in pts); den=sum((x-mx)**2 for x in xs)
    k=-num/den if den else 0
    print(f"meters_per_tick={k:.5f} n={n} ticks_span={xs[-1]-xs[0]:.0f} range_span={ys[0]-ys[-1]:.3f}")
    print("pts:", [(round(t,2),round(x)) for t,x in pts[::4]])
