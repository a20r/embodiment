import sys; sys.path.insert(0,'/bot/src')
from manual import *
import os, time

def d11(n=5):
    vs=[]
    for _ in range(n):
        v=last_of('d11',0.1)
        try: vs.append(float(v))
        except: pass
    return sum(vs)/len(vs) if vs else None

def tx(msg):
    try:
        fd=os.open('/dev/robot/d8',os.O_WRONLY|os.O_NONBLOCK)
        os.write(fd,(msg+"\n").encode()); os.close(fd)
    except Exception: pass

def flags():
    s=last_of('d3',0.2)
    return ('goal=1' in s), ('here=1' in s)

t_end=time.time()+600
v=d11(6)
best=v
print(f"start d11={v:.3f}", flush=True)
rnd=0
while time.time()<t_end:
    rnd+=1
    g_,h_=flags()
    if g_ or h_:
        print(f"FLAG goal={g_} here={h_} !!!", flush=True)
        stop(); break
    v0=d11(5)
    results=[]
    for deg in (0,90,180,-90):
        if deg: rot(deg,60)
        fwd(0.8,70)
        vv=d11(3)
        results.append((deg,vv))
        g_,h_=flags()
        if g_ or h_: print(f"FLAG! goal={g_} here={h_}",flush=True); stop(); sys.exit()
    # commit to best direction
    valid=[(v_,d_) for d_,v_ in results if v_ is not None]
    if not valid: continue
    bv,bd=min(valid)
    # rotate so that net heading = direction of bd: current heading is after +0+90+180-90 = +180 net
    # simpler: best dir relative to current: (bd - 180)
    rel=(bd-180)%360
    if rel>180: rel-=360
    if abs(rel)>3: rot(rel,60)
    # drive 2 more steps in committed direction
    for _ in range(2):
        fwd(1.0,70)
        vv=d11(3)
        if vv is not None and vv<bv-0.005: bv=vv
        g_,h_=flags()
        if g_ or h_: print(f"FLAG! goal={g_} here={h_}",flush=True); stop(); sys.exit()
    if bv<best: best=bv
    tx(f"PING A{rnd}")
    print(f"round{rnd} v0={v0:.3f} probes={[round(v,3) if v else None for d_,v in results]} commit rel={rel:.0f} best={best:.3f} h={last_of('d4',0.15)}", flush=True)
print(f"done best={best:.3f}")
