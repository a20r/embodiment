import time, math
from rob import *
from nav import rot_to, havg, d5val
def fb():
    r=ranges(); return r[2],r[10]
def athere(st): return 'here=1' in st or 'goal=1' in st

def push(nom, tmax=35, dist_stop=0.30, rev=False):
    sgn=-1 if rev else 1
    PATTERNS=[('solid7',7,1.2,0),('pulseD',7,0.5,0.06),('solid4',4,1.0,0),('pulseA',7,0.18,0.08),('solid10',10,1.0,0),('kick',-7,0.35,0)]
    face=(nom+180)%360 if rev else nom
    def FB():
        f,b=fb()
        return (b,f) if rev else (f,b)
    t0=time.time(); moved=0.0; pi=0
    f0,b0=FB()
    while time.time()-t0<tmax:
        name,v,on,off=PATTERNS[pi%len(PATTERNS)]; pi+=1
        v*=sgn
        te=time.time()+(3.0 if off>0 else on)
        while time.time()<te:
            motors(v,v); time.sleep(on)
            if off: motors(0,0); time.sleep(off)
        st=rd('d6')
        if athere(st): stop(); return 'HERE',moved
        f1,b1=FB()
        if 0<f1<dist_stop and v*sgn>0: stop(); return 'wall',moved
        d=0
        if f0>0 and f1>0 and f0<2.9: d=f0-f1
        if b0>0 and b1>0 and b1<2.9: d=max(d,b1-b0)
        if v*sgn>0 and d>0.15:
            moved+=d; fmin=f1 if f1>0 else 3.0; last=time.time()
            while time.time()-t0<tmax+20:
                motors(v,v); time.sleep(on)
                if off: motors(0,0); time.sleep(off)
                st=rd('d6')
                if athere(st): stop(); return 'HERE',moved
                fc,bc=FB()
                if fc<0: continue
                if fc<dist_stop: stop(); return 'wall',moved+max(0,fmin-fc)
                if fc<fmin-0.05: moved+=fmin-fc; fmin=fc; last=time.time()
                if fc>fmin+0.4: fmin=fc; last=time.time()
                if time.time()-last>2.0: break
            stop()
            pi-=1
            f0,b0=FB(); continue
        f0,b0=f1,b1
        h=havg(2)
        if abs((face-h+180)%360-180)>7: rot_to(face,tol=4)
    stop(); return 'end',moved
