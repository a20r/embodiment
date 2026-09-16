import sys,time,math
sys.path.insert(0,'/bot/src')
from ctl import *
SAT=2.65   # readings above this = saturated/far
def medscan(n=3):
    """median of n scans, ignoring -1 dropouts"""
    ss=[ranges() for _ in range(n)]; ss=[s for s in ss if s]
    out=[]
    for i in range(16):
        v=sorted(x[i] for x in ss if x[i]>=0)
        out.append(v[len(v)//2] if v else -1.0)
    return out
def beam_for(bearing, h):
    return int(round(((bearing-h)%360)/22.5))%16
def cardinal_view(h=None, sc=None):
    """distances toward N,E,S,W (bearings 0,90,180,270) from current heading"""
    if h is None: h=hd(3)
    if sc is None: sc=medscan(3)
    return {b: sc[beam_for(b,h)] for b in (0,90,180,270)}, h, sc
def steer(bearing,h,sc,spd):
    r,l=sc[beam_for(bearing+90,h)],sc[beam_for(bearing-90,h)]; tgt=bearing
    if 0<r<0.6 and 0<l<0.6:
        tgt=bearing+max(-20,min(20,70*(r-l)))   # + => more room right => aim right (CW)
    d=angdiff(tgt,h); corr=d*abs(spd)/30
    return max(-abs(spd)/2,min(abs(spd)/2,corr))
def drive_front(target, bearing, maxt=40, tol=0.03):
    """servo forward/back until beam0 distance == target (m). Assumes facing bearing."""
    t0=time.time(); ok=False; hist=[]
    while time.time()-t0<maxt:
        sc=medscan(1); h=hd(2)
        if sc is None or h is None: continue
        f=sc[beam_for(bearing,h)]
        if f<0: continue
        err=f-target
        if abs(err)<tol:
            hist.append(err)
            if len(hist)>=2: ok=True; break
        else: hist=[]
        spd=max(18,min(60,abs(err)*120))
        if err<0: spd=-max(18,min(30,abs(err)*120))
        corr=steer(bearing,h,sc,spd)
        set_speeds(spd+corr, spd-corr)
        time.sleep(0.08)
    stop(); time.sleep(0.4)
    return ok, medscan(3)[beam_for(bearing,hd(3))]
def drive_blind(bearing, secs, spd=40):
    t0=time.time()
    while time.time()-t0<secs:
        sc=medscan(1); h=hd(2)
        if sc is None or h is None: continue
        if 0<sc[beam_for(bearing,h)]<0.30: break
        corr=steer(bearing,h,sc,spd)
        set_speeds(spd+corr,spd-corr); time.sleep(0.08)
    stop(); time.sleep(0.4)
if __name__=='__main__':
    if sys.argv[1]=='view':
        v,h,sc=cardinal_view(); print(f"h={h:.1f}", {k:round(x,2) for k,x in v.items()}); print(' '.join(f"{x:.2f}" for x in sc))
    elif sys.argv[1]=='front': print(drive_front(float(sys.argv[2]), float(sys.argv[3])))
