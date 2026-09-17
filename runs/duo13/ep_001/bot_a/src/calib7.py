import sys, time, math; sys.path.insert(0,'/bot/src')
from drive import Driver, wrap
from robot import R
r=R(); r.motors(0,0); time.sleep(0.3)
rg=r.ranges() or []
cand=[(v,k) for k,v in enumerate(rg) if v and 1.2<=v<=2.4]
if not cand:
    r.motors(-60,-60); time.sleep(0.7); r.motors(0,0); time.sleep(0.3)
    rg=r.ranges() or []
    cand=[(v,k) for k,v in enumerate(rg) if v and 1.2<=v<=2.4]
print("cand:",cand[:3], flush=True)
k=cand[0][1]; target=(r.heading() or 0)+k*22.5
d=Driver(); d.turnto(target,tol=2.5); time.sleep(0.3)
print("aiming abs",target,"h=",r.heading(),"f0=",(r.ranges() or [9])[0], flush=True)
def enc2():
    for _ in range(6):
        e=r.enc()
        if e[0] is not None and e[1] is not None: return e
        time.sleep(0.04)
e0=enc2(); pts=[]; t0=time.time()
while time.time()-t0<18:
    r.motors(0,0); time.sleep(0.18)
    rg=r.ranges(); f=rg[0] if rg else None
    e=enc2()
    if f is None or e is None:
        continue
    if f<0.4: break
    tk=((e[0]-e0[0])+(e[1]-e0[1]))/2.0
    pts.append((round(f,3),int(tk)))
    err=wrap(target-(r.heading() or target))
    r.motors(int(55+1.0*err), int(55-1.0*err)); time.sleep(0.25)
r.motors(0,0)
print("pts:",pts, flush=True)
n=len(pts)
xs=[p[1] for p in pts]; ys=[p[0] for p in pts]
mx=sum(xs)/n; my=sum(ys)/n
num=sum((x-mx)*(y-my) for x,y in pts); den=sum((x-mx)**2 for x in xs)
slope=num/den
print(f"slope={slope:.6f} m/tick  (1m = {abs(1/slope):.0f} ticks)", flush=True)
