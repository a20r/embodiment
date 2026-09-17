import time, math
from rob import *
from nav import rot_to, havg, d5val
from man import fsweep
LOG=open('/tmp/nav.log','a')
def log(*a):
    LOG.write('F2 '+' '.join(str(x) for x in a)+'\n'); LOG.flush()
def fb():
    r=ranges(); return r[2],r[10]
def athere(st): return 'here=1' in st or 'goal=1' in st

PATTERNS=[('solid7',7,1.2,0),('pulseD',7,0.5,0.06),('solid4',4,1.0,0),('pulseA',7,0.18,0.08),('solid10',10,1.0,0),('kick',-7,0.35,0)]
def push(nom, tmax=35, dist_stop=0.30):
    t0=time.time(); moved=0.0; pi=0
    f0,b0=fb()
    while time.time()-t0<tmax:
        name,v,on,off=PATTERNS[pi%len(PATTERNS)]; pi+=1
        te=time.time()+ (3.0 if off>0 else on)
        while time.time()<te:
            motors(v,v); time.sleep(on)
            if off: motors(0,0); time.sleep(off)
        st=rd('d6')
        if athere(st): stop(); return 'HERE',moved
        f1,b1=fb()
        if 0<f1<dist_stop and v>0: stop(); return 'wall',moved
        d=0
        if f0>0 and f1>0 and f0<2.9: d=f0-f1
        if b0>0 and b1>0 and b1<2.9: d=max(d,b1-b0)
        if v>0 and d>0.15:
            moved+=d; fmin=f1 if f1>0 else 3.0; last=time.time()
            while time.time()-t0<tmax+20:
                motors(v,v); time.sleep(on)
                if off: motors(0,0); time.sleep(off)
                st=rd('d6')
                if athere(st): stop(); return 'HERE',moved
                fc,bc=fb()
                if fc<0: continue
                if fc<dist_stop: stop(); return 'wall',moved+max(0,fmin-fc)
                if fc<fmin-0.05: moved+=fmin-fc; fmin=fc; last=time.time()
                if fc>fmin+0.4: fmin=fc; last=time.time()
                if time.time()-last>2.0: break
            stop()
            pi-=1  # retry same pattern
            f0,b0=fb(); continue
        f0,b0=f1,b1
        # heading maintenance
        h=havg(2)
        if abs((nom-h+180)%360-180)>7: rot_to(nom,tol=4)
    stop(); return 'end',moved
cur=228.0
lastbeacon=0
for it in range(500):
    st=status()
    if athere(st):
        stop(); log('ATGOAL',st)
        while True:
            tx('botB FOUND GOAL. staying. climb d5 to me.'); time.sleep(2); log(status())
    if time.time()-lastbeacon>15:
        tx('botB d5=%.3f %s'%(d5val(2),st)); lastbeacon=time.time()
    b,m=fsweep(cur, span=30, dur=6)
    if m<0.55:
        # look other directions: left, right, back
        found=False
        for dd in [-90,90,180]:
            b2,m2=fsweep(cur+dd, span=30, dur=6)
            if m2>0.55: b,m=b2,m2; found=True; break
        if not found:
            log('boxed'); motors(12,-12); time.sleep(2); stop(); continue
    rot_to(b, tol=3.5)
    why,mv=push(b)
    log('it',it,'tgt',round(b,1),why,'moved',round(mv,2),'d5',round(d5val(2),3),'h',round(havg(),1),status())
    if why=='HERE': continue
    if mv>0.2 or why=='wall':
        cur=b if mv>0.2 else cur+180
stop()
