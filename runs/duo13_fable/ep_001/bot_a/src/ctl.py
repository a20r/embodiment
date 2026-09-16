import sys,time,math
sys.path.insert(0,'/bot/src')
from rio import *
def stop():
    writeline(1,"0"); writeline(7,"0")
def set_speeds(l,r):
    writeline(1,str(l)); writeline(7,str(r))
def hd(n=3):
    """averaged heading (circular mean of n samples)"""
    xs=[];ys=[]
    for _ in range(n):
        h=heading()
        if h is None: continue
        xs.append(math.cos(math.radians(h))); ys.append(math.sin(math.radians(h)))
    if not xs: return None
    return math.degrees(math.atan2(sum(ys),sum(xs)))%360
def angdiff(a,b):
    """signed a-b in [-180,180)"""
    return (a-b+180)%360-180
def enc():
    a=readline(6); b=readline(9)
    return (int(a) if a else None, int(b) if b else None)
def turn_to(target, tol=4, maxt=40):
    t0=time.time()
    while time.time()-t0<maxt:
        h=hd(3)
        if h is None: continue
        d=angdiff(target,h)  # + => need to turn clockwise => left wheel (d1) forward
        if abs(d)<=tol: break
        s=max(2,min(6,abs(d)/8))
        if d>0: set_speeds(s,-s)
        else: set_speeds(-s,s)
        time.sleep(0.15)
    stop(); time.sleep(0.3)
    return hd(5)
def forward(counts, bearing, speed=6, front_stop=0.18, maxt=60):
    """drive forward `counts` encoder counts holding compass `bearing`."""
    e0=enc(); t0=time.time(); last=None
    while time.time()-t0<maxt:
        e=enc(); r=ranges(); h=hd(2)
        if None in e or r is None or h is None: continue
        trav=((e[0]-e0[0])+(e[1]-e0[1]))/2
        if trav>=counts: break
        if 0<r[0]<front_stop: print("front obstacle",r[0]); break
        d=angdiff(bearing,h)  # + => turn clockwise => left faster
        corr=max(-2,min(2,d/10))
        set_speeds(speed+corr, speed-corr)
        last=(trav,r[0],h)
        time.sleep(0.1)
    stop(); time.sleep(0.3)
    e=enc(); return ((e[0]-e0[0])+(e[1]-e0[1]))/2
if __name__=='__main__':
    cmd=sys.argv[1]
    if cmd=='turn': print(turn_to(float(sys.argv[2])))
    elif cmd=='fwd': print(forward(float(sys.argv[2]), float(sys.argv[3])))
    elif cmd=='stop': stop()
    elif cmd=='hd': print(hd(5))
    elif cmd=='scan':
        r=ranges(); h=hd(3); print("h=",h); print(' '.join(f"{x:.2f}" for x in r))
