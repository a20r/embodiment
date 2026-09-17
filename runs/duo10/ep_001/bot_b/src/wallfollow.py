import time, math
from rob import *
from nav import rot_to, d5val, front
LOG=open('/tmp/nav.log','a')
def log(*a):
    LOG.write('WF '+' '.join(str(x) for x in a)+'\n'); LOG.flush()

def drive7(tgt_h, dist_stop=0.34, tmax=6):
    t0=time.time(); fmin=front(); lastprog=t0; f0=fmin
    while time.time()-t0<tmax:
        h=heading(); err=(tgt_h-h+180)%360-180
        corr=max(-2,min(2,0.3*err))
        motors(round(7+corr,1), round(7-corr,1))
        time.sleep(0.12)
        st=rd('d6')
        if 'here=1' in st or 'goal=1' in st:
            stop(); return 0,'HERE'
        f=front()
        if f<0: continue
        if f<dist_stop: stop(); return f0-f,'wall'
        if f<fmin-0.04: fmin=f; lastprog=time.time()
        if f>fmin+0.5: fmin=f; lastprog=time.time()
        if time.time()-lastprog>1.8: stop(); return f0-f,'stuck'
    stop(); return f0-front(),'time'

def pick_left():
    r=ranges(); h=heading()
    # left-hand rule: left(14), fl(0), straight(2), fr(4), right(6), back(10)
    for i in [14,0,2,4,6,10,12,8]:
        if r[i]>0.75:
            return (h+22.5*(i-2))%360, i, r[i]
    return None

lastbeacon=0
stuckcount=0
for it in range(300):
    st=status()
    if 'here=1' in st or 'goal=1' in st:
        stop(); log('AT GOAL', st)
        while True:
            tx('botB FOUND GOAL here=1! climb d5 to me.'); time.sleep(2)
            log(status())
    if time.time()-lastbeacon>8:
        tx('botB d5=%.3f %s'%(d5val(4), st)); lastbeacon=time.time()
    c=pick_left()
    if c is None:
        motors(10,-10); time.sleep(0.8); stop(); log('spin, no exit'); continue
    tgt,beam,rng=c
    rot_to(tgt)
    m,why=drive7(tgt)
    log('it',it,'beam',beam,'rng',round(rng,2),'moved',round(m,2),why,'d5',round(d5val(3),3))
    if why=='HERE':
        continue
    if why=='stuck' and m<0.3:
        stuckcount+=1
        motors(-7,-7); time.sleep(0.6); stop()
        if stuckcount>=3:
            motors(12,-12); time.sleep(0.8); stop(); stuckcount=0
    else: stuckcount=0
stop()
