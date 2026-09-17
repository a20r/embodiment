#!/usr/bin/env python3
import json, time, math, sys, os
sys.path.insert(0,'/bot/src')
from rio import write
TPU = 0.00053   # units per encoder tick (calibrated vs lidar)
POSE='/tmp/pose.json'
def st():
    for _ in range(50):
        try: return json.load(open('/tmp/state.json'))
        except Exception: time.sleep(0.01)
    return {}
def scan(s):
    try: return [float(x) for x in s['d2'].split(',')]
    except: return [-1]*16
def hd(s):
    try: return float(s['d4'])
    except: return None
def angdiff(a,b):  # a-b wrapped to [-180,180]
    return (a-b+180)%360-180
def load_pose():
    try: return json.load(open(POSE))
    except: return {'x':0.0,'y':0.0}
def save_pose(p): json.dump(p, open(POSE,'w'))
def stop(): write('d1','0'); write('d7','0')
def med(vals):
    v=sorted(vals); return v[len(v)//2]
class HeadingFilter:
    def __init__(s): s.h=[]
    def upd(s,v):
        if v is None: return s.get()
        s.h.append(v); s.h=s.h[-3:]
        return s.get()
    def get(s):
        if not s.h: return None
        # circular median approx: unwrap relative to last
        ref=s.h[-1]; return (ref+med([angdiff(x,ref) for x in s.h]))%360
def turn_to(target, tol=3, timeout=20):
    hf=HeadingFilter(); t0=time.time(); ok=0
    while time.time()-t0<timeout:
        s=st(); h=hf.upd(hd(s))
        if h is None: time.sleep(0.05); continue
        err=angdiff(target,h)
        if abs(err)<tol:
            ok+=1
            if ok>=3: break
            stop(); time.sleep(0.05); continue
        ok=0
        sp=max(8,min(60,abs(err)*1.5))
        if err>0: write('d1',str(sp)); write('d7',str(-sp))
        else: write('d1',str(-sp)); write('d7',str(sp))
        time.sleep(0.05)
    stop(); time.sleep(0.3)
    return hd(st())
def forward(dist, speed=150, front_stop=0.30, timeout=60, target_hd=None):
    """drive forward up to dist units, hold heading, stop if front beam < front_stop. Updates pose."""
    hf=HeadingFilter(); s=st(); pose=load_pose()
    h0=hd(s); target=target_hd if target_hd is not None else h0
    l0=int(s['d9']); r0=int(s['d6']); lp,rp=l0,r0
    t0=time.time(); reason='dist'; travelled=0.0
    while time.time()-t0<timeout:
        s=st(); h=hf.upd(hd(s))
        try: l=int(s['d9']); r=int(s['d6'])
        except: time.sleep(0.02); continue
        d=((l-lp)+(r-rp))/2*TPU; lp,rp=l,r; travelled+=d
        if h is not None:
            pose['x']+=d*math.sin(math.radians(h)); pose['y']+=d*math.cos(math.radians(h))
        if travelled>=dist: reason='dist'; break
        sc=scan(s)
        f=[v for v in (sc[0],sc[15],sc[1]) if v>=0]
        if f and min(sc[0] if sc[0]>=0 else 9, min(v/math.cos(math.radians(22.5)) for v in (sc[1],sc[15]) if v>=0) if any(v>=0 for v in (sc[1],sc[15])) else 9) < front_stop:
            reason='blocked'; break
        if s.get('d0')!='0' or s.get('d5')!='0': reason='bump'; break
        err=angdiff(target,h) if h is not None else 0
        corr=max(-40,min(40,err*1.5))
        write('d1',str(speed+corr)); write('d7',str(speed-corr))
        time.sleep(0.05)
    else: reason='timeout'
    stop(); time.sleep(0.3); save_pose(pose)
    s=st()
    return reason, travelled, pose, hd(s), scan(s)
def report(tag=''):
    s=st(); sc=scan(s); p=load_pose()
    print(f"{tag} hd={hd(s)} pose=({p['x']:.2f},{p['y']:.2f}) d11={s.get('d11')} {s.get('d3')} d0={s.get('d0')} d5={s.get('d5')}")
    print("  scan:", " ".join(f"{v:5.2f}" for v in sc))
if __name__=="__main__":
    cmd=sys.argv[1]
    if cmd=='turn': print("turned, hd=",turn_to(float(sys.argv[2]))); report('after turn')
    elif cmd=='fwd':
        dist=float(sys.argv[2]); sp=float(sys.argv[3]) if len(sys.argv)>3 else 150
        r=forward(dist,speed=sp); print(f"fwd: reason={r[0]} travelled={r[1]:.2f}"); report('after fwd')
    elif cmd=='stop': stop()
    elif cmd=='report': report()
    elif cmd=='setpose': save_pose({'x':float(sys.argv[2]),'y':float(sys.argv[3])})
