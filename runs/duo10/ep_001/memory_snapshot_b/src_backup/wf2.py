import time, math
from rob import *
from nav import rot_to, d5val
LOG=open('/tmp/nav.log','a')
def log(*a):
    LOG.write('WF2 '+' '.join(str(x) for x in a)+'\n'); LOG.flush()
def front(): return ranges()[2]

def athere(st):
    return 'here=1' in st or 'goal=1' in st

def drive(tgt_h, dist_stop=0.33, tmax=7):
    t0=time.time(); fmin=front(); f0=fmin; lastprog=t0
    motors(7,7)
    while time.time()-t0<tmax:
        motors(7,7)
        time.sleep(0.15)
        st=rd('d6')
        if athere(st): stop(); return 0,'HERE'
        f=front()
        if f<0: continue
        if f<dist_stop: stop(); return f0-f,'wall'
        if f<fmin-0.05: fmin=f; lastprog=time.time()
        if f>fmin+0.5: fmin=f; lastprog=time.time()
        h=heading(); err=(tgt_h-h+180)%360-180
        if abs(err)>14:
            stop(); rot_to(tgt_h); motors(7,7)
        if time.time()-lastprog>1.6:
            stop(); return f0-f,'stuck'
    stop(); return f0-front(),'time'

def pick_left():
    r=ranges(); h=heading()
    for i in [14,0,2,4,6,10,12,8]:
        if r[i]>0.75:
            return (h+22.5*(i-2))%360, i, r[i]
    r2=max(range(16), key=lambda i:r[i])
    if r[r2]>0.5: return (h+22.5*(r2-2))%360, r2, r[r2]
    return None

lastbeacon=0; stuckcount=0
for it in range(500):
    st=status()
    if athere(st):
        stop(); log('AT GOAL',st)
        while True:
            tx('botB FOUND GOAL here=1! stay-put=me. climb d5 to me.'); time.sleep(2); log(status())
    if time.time()-lastbeacon>8:
        tx('botB d5=%.3f %s'%(d5val(4),st)); lastbeacon=time.time()
    c=pick_left()
    if c is None:
        motors(10,-10); time.sleep(0.7); stop(); log('spin'); continue
    tgt,beam,rng=c
    rot_to(tgt)
    m,why=drive(tgt)
    log('it',it,'beam',beam,'rng',round(rng,2),'moved',round(m,2),why,'d5',round(d5val(3),3),status())
    if why=='stuck' and m<0.25:
        stuckcount+=1
        motors(-7,-7); time.sleep(0.6); stop()
        if stuckcount>=3:
            motors(11,-11); time.sleep(0.5); stop(); stuckcount=0
    else: stuckcount=0
stop()
