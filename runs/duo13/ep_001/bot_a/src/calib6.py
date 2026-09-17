import sys, time, math; sys.path.insert(0,'/bot/src')
from drive import Driver, wrap
from robot import R
r=R(); r.motors(0,0); time.sleep(0.3)
rg=r.ranges() or []
cand=[(v,k) for k,v in enumerate(rg) if v and 0.75<=v<=2.2]
if not cand:
    r.motors(-60,-60); time.sleep(0.7); r.motors(0,0); time.sleep(0.3)
    rg=r.ranges() or []
    cand=[(v,k) for k,v in enumerate(rg) if v and 0.75<=v<=2.2]
cand.sort(reverse=True)
print("cand:",cand[:4], flush=True)
k=cand[0][1]; target=(r.heading() or 0)+k*22.5
d=Driver()
ok=d.turnto(target, tol=2.5)
print("turned:",ok, r.heading(), flush=True)
def enc2():
    for _ in range(6):
        e=r.enc()
        if e[0] is not None and e[1] is not None: return e
        time.sleep(0.04)
e0=enc2(); pts=[]; t0=time.time()
while time.time()-t0<15:
    r.motors(0,0); time.sleep(0.15)
    rg=r.ranges(); f=rg[0] if rg else None
    e=enc2()
    if f is None or e is None:
        r.motors(0,0); time.sleep(0.05); continue
    if f<0.35: break
    tk=((e[0]-e0[0])+(e[1]-e0[1]))/2.0
    pts.append((f,tk))
    if f<0.55: break
    err=wrap(target-(r.heading() or target))
    r.motors(int(60+1.0*err), int(60-1.0*err)); time.sleep(0.28)
r.motors(0,0)
print("n=",len(pts), flush=True)
xs=[p[1] for p in pts]; ys=[p[0] for p in pts]
n=len(pts)
if n>5:
    mx=sum(xs)/n; my=sum(ys)/n
    num=sum((x-mx)*(y-my) for x,y in pts); den=sum((x-mx)**2 for x in xs)
    kfit=-num/den if den else 0
    print(f"MPT={kfit:.6f} = {1/kfit if kfit else 0:.0f} ticks/m span_ticks={xs[-1]-xs[0]:.0f} m {ys[0]:.2f}->{ys[-1]:.2f}", flush=True)
    print("first/last:",pts[0],pts[-1], flush=True)
else: print("pts:",pts, flush=True)
