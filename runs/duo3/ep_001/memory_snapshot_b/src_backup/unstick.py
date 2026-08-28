from ctl import Ctl, angdiff
import time

def spin_to(b, target):
    t0=time.time()
    while time.time()-t0<12:
        h=b.heading()
        if h is None: continue
        e=angdiff(target,h)
        if abs(e)<5:
            b.wr(4,'0'); b.wr(5,'0'); return True
        w=max(min(e*1.5,60),-60)
        if abs(w)<22: w=22*(1 if w>0 else -1)
        l=-w/1.78; r=w/1.78
        b.wr(4,f'{l:.1f}'); b.wr(5,f'{r:.1f}')
        time.sleep(0.1)
    b.wr(4,'0'); b.wr(5,'0'); return False

def sc(b):
    return [x if x>0 else 0.05 for x in b.scan()]

def unstick(b, min_clear=0.17, max_it=8):
    for it in range(max_it):
        s=sc(b); mn=min(s); mi=s.index(mn)
        if mn>=min_clear: return True
        h=b.heading()
        away=(h + ((mi+8)%16)*22.5)%360
        spin_to(b, away)
        b.wr(4,'-22'); b.wr(5,'-22'); time.sleep(0.45)
        b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.15)
    return min(sc(b))>=min_clear
