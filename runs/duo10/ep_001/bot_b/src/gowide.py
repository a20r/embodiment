import time
from rob import *
from nav import rot_to
def fb():
    r=ranges(); return r[2], r[10]
def try_move(secs=0.9):
    f0,b0=fb(); t0=time.time()
    while time.time()-t0<secs: motors(7,7); time.sleep(0.12)
    f1,b1=fb()
    d=0
    if f0>0 and f1>0 and f0<2.9: d=max(d,f0-f1)
    if b0>0 and b1>0 and b1<2.9: d=max(d,b1-b0)
    if f0>=2.9 and f1>=2.9 and b0>=2.9 and b1>=2.9: d=0.5  # assume moving in long corridor
    return d, f1
def go_open(tmax=30, dist_stop=0.32, nom=None):
    r=ranges(); h=heading()
    if nom is None:
        best=max(range(16), key=lambda i:r[i])
        nom=(h+22.5*(best-2))%360
    lock=None
    for off in [0,4,-4,8,-8,12,-12]:
        rot_to(nom+off, tol=3)
        d,f=try_move()
        if f>0 and f<dist_stop: return 'wall',0,heading()
        if d>0.12: lock=off; break
    if lock is None: return 'nolock',0,heading()
    nomL=nom+lock; moved=d; fmin=f if f>0 else 3.0; last=time.time(); wig=0; t0=time.time()
    while time.time()-t0<tmax:
        motors(7,7); time.sleep(0.13)
        st=rd('d6')
        if 'here=1' in st or 'goal=1' in st: stop(); return 'HERE',moved,heading()
        fc,bc=fb()
        if fc<0: continue
        if fc<dist_stop: stop(); return 'wall',moved+max(0,fmin-fc),heading()
        if fc<fmin-0.05: moved+=fmin-fc; fmin=fc; last=time.time()
        if fc>fmin+0.4: fmin=fc; last=time.time()
        if fc>=2.9: last=time.time()  # can't judge; assume ok
        if time.time()-last>1.3:
            wig+=1
            if wig>5: stop(); return 'stuck',moved,heading()
            motors(-7,-7); time.sleep(0.35); stop()
            rot_to(nomL+[3,-3,6,-6,9][(wig-1)%5], tol=2.5)
            d,f=try_move()
            if d>0.12: moved+=d; fmin=f if f>0 else 3.0; last=time.time()
    stop(); return 'time',moved,heading()
