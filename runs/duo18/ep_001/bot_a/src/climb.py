#!/usr/bin/env python3
# Discrete hill-climb on d11 (other robot parked): sample, step in best open direction, repeat.
import json, time, math, sys, random
sys.path.insert(0,'/bot/src')
from ctl import st, scan, hd, angdiff, load_pose, save_pose, stop, turn_to, forward
LOG='/tmp/climb.log'
def log(m):
    with open(LOG,'a') as f: f.write(f"{time.time():.1f} {m}\n")
def sample(sec=1.5):
    vals=[]; t0=time.time()
    while time.time()-t0<sec:
        try: vals.append(float(st()['d11']))
        except: pass
        time.sleep(0.05)
    vals.sort(); return vals[len(vals)//2]
def grad(samples):
    # least squares plane fit z = a + bx x + by y over samples [(x,y,z)]
    if len(samples)<3: return None
    n=len(samples); mx=sum(s[0] for s in samples)/n; my=sum(s[1] for s in samples)/n; mz=sum(s[2] for s in samples)/n
    sxx=sum((s[0]-mx)**2 for s in samples); syy=sum((s[1]-my)**2 for s in samples); sxy=sum((s[0]-mx)*(s[1]-my) for s in samples)
    sxz=sum((s[0]-mx)*(s[2]-mz) for s in samples); syz=sum((s[1]-my)*(s[2]-mz) for s in samples)
    det=sxx*syy-sxy*sxy
    if det<1e-3: return None
    bx=(sxz*syy-syz*sxy)/det; by=(syz*sxx-sxz*sxy)/det
    return bx,by
def run(steps=12, step=0.3):
    samples=[]; visited=[]
    best=(-1,None)
    for k in range(steps):
        s=st(); p=load_pose(); z=sample(1.5); h=hd(s)
        samples.append((p['x'],p['y'],z)); visited.append((p['x'],p['y']))
        if z>best[0]: best=(z,(p['x'],p['y']))
        if 'goal=1' in s.get('d3','') or 'here=1' in s.get('d3',''): log(f"STATUS {s['d3']}"); break
        g=grad(samples[-8:])
        want=None
        if g and math.hypot(*g)>0.03: want=math.degrees(math.atan2(g[0],g[1]))%360
        sc=[v if v>=0 else 3.0 for v in scan(s)]
        cands=[]
        for i in range(16):
            ang=(h+22.5*i)%360
            clr=min(sc[i], max(sc[(i-1)%16],0.01)/0.7, max(sc[(i+1)%16],0.01)/0.7)
            if clr<0.55: continue
            nx=p['x']+step*math.sin(math.radians(ang)); ny=p['y']+step*math.cos(math.radians(ang))
            nov=min([math.hypot(nx-vx,ny-vy) for vx,vy in visited]+[9])
            score=min(clr,1.5)*0.3 + 1.0*min(nov,0.6)
            if want is not None: score+=1.2*math.cos(math.radians(angdiff(ang,want)))
            cands.append((score,ang,clr))
        if not cands:
            log(f"step{k}: no candidates, z={z:.3f}"); turn_to((h+90)%360); continue
        cands.sort(reverse=True); score,ang,clr=cands[0]
        log(f"step{k}: pose=({p['x']:.2f},{p['y']:.2f}) z={z:.3f} want={None if want is None else round(want)} go={ang:.0f} clr={clr:.2f}")
        turn_to(ang, tol=4, timeout=12)
        r=forward(min(step,clr-0.3), speed=130, front_stop=0.28, timeout=15)
        if r[0]=='bump': log("bump")
    p=load_pose(); z=sample(1.5); log(f"end pose=({p['x']:.2f},{p['y']:.2f}) z={z:.3f} best={best}")
    stop()
if __name__=="__main__":
    run(int(sys.argv[1]) if len(sys.argv)>1 else 12)
