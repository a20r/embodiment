import time, math, json, sys
from rob import *
from nav import rot_to, d5val, front

LOG=open('/tmp/nav.log','a')
def log(*a):
    LOG.write(' '.join(str(x) for x in a)+'\n'); LOG.flush()

def drive7(tgt_h, dist_stop=0.34, tmax=6):
    t0=time.time(); fmin=front(); lastprog=t0; f0=fmin
    while time.time()-t0<tmax:
        h=heading(); err=(tgt_h-h+180)%360-180
        # gentle heading correction around 7
        corr=max(-2,min(2,0.3*err))
        motors(round(7+corr,1), round(7-corr,1))
        time.sleep(0.12)
        f=front()
        if f<0: continue
        if f<dist_stop: stop(); return f0-f,'wall'
        if f<fmin-0.04: fmin=f; lastprog=time.time()
        if f>fmin+0.5: fmin=f; lastprog=time.time()
        if time.time()-lastprog>1.8: stop(); return f0-f,'stuck'
    stop(); return f0-front(),'time'

def step(prefer_h=None):
    r=ranges()
    h=heading()
    # candidate directions: beams with range>0.8
    cands=[]
    for i in range(16):
        if r[i]>0.8:
            bh=(h+22.5*(i-2))%360
            cands.append((r[i],bh,i))
    if not cands: return None
    if prefer_h is not None:
        cands.sort(key=lambda c:-(c[0]) - 2.0*math.cos(math.radians(c[1]-prefer_h)))
        cands.sort(key=lambda c: abs((c[1]-prefer_h+180)%360-180))
    else:
        cands.sort(key=lambda c:-c[0])
    return cands[0]

def main():
    d5=d5val(12)
    log('START d5',round(d5,3),status())
    prefer=None
    for it in range(40):
        c=step(prefer)
        if c is None:
            log('no candidates, rotating'); motors(10,-10); time.sleep(1); stop(); continue
        rng,tgt,beam=c
        rot_to(tgt)
        m,why=drive7(tgt)
        d5n=d5val(12)
        st=status()
        log('it',it,'beam',beam,'rng',round(rng,2),'tgt',round(tgt,1),'moved',round(m,2),why,'d5',round(d5n,3),st)
        if 'here=1' in st or 'goal=1' in st:
            log('ARRIVED?',st); stop(); return
        if m>0.3:
            prefer = tgt if d5n>d5+0.005 else (tgt+180)%360
        if why=='stuck':
            motors(-7,-7); time.sleep(0.7); stop()
        d5=d5n
        tx('botB d5=%.3f'%d5n)
    stop()
main()
