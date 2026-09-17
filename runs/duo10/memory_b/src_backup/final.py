import time, math
from rob import *
from nav import rot_to, d5val, havg
LOG=open('/tmp/nav.log','a')
def log(*a):
    LOG.write('FIN '+' '.join(str(x) for x in a)+'\n'); LOG.flush()
AXES={0:33.0,1:123.0,2:213.0,3:303.0}
def fb():
    r=ranges(); return r[2],r[10]
def athere(st): return 'here=1' in st or 'goal=1' in st

def push_catch(nom, tmax=25, dist_stop=0.30):
    """try to get moving along heading nom. returns (state, moved)"""
    t0=time.time(); moved=0.0
    variants=[('f',7,1.2),('k',-7,0.35),('f',7,1.2),('f',4,1.0),('w',3,0),('f',7,1.2),('k',-7,0.35),('f',4,1.0),('w',-3,0),('f',7,1.2),('f',10,1.0)]
    vi=0
    f0,b0=fb()
    while time.time()-t0<tmax:
        typ,v,dur=variants[vi%len(variants)]; vi+=1
        if typ=='w':
            rot_to(nom+v,tol=3); continue
        motors(v,v); time.sleep(dur)
        st=rd('d6')
        if athere(st): stop(); return 'HERE',moved
        f1,b1=fb()
        d=0
        if 0<f1<dist_stop and v>0: stop(); return 'wall',moved
        if f0>0 and f1>0 and f0<2.9: d=f0-f1
        if b0>0 and b1>0 and b1<2.9: d=max(d,b1-b0)
        if v>0 and d>0.15:
            # caught! keep same speed
            moved+=d; fmin=f1 if f1>0 else 3.0; last=time.time()
            while time.time()-t0<tmax:
                motors(v,v); time.sleep(0.13)
                st=rd('d6')
                if athere(st): stop(); return 'HERE',moved
                fc,bc=fb()
                if fc<0: continue
                if fc<dist_stop: stop(); return 'wall',moved+max(0,fmin-fc)
                if fc<fmin-0.05: moved+=fmin-fc; fmin=fc; last=time.time()
                if fc>fmin+0.4: fmin=fc; last=time.time()
                if time.time()-last>1.5: break
            stop()
            if time.time()-t0>=tmax: return 'time',moved
            f0,b0=fb(); continue
        f0,b0=f1,b1
    stop(); return 'end',moved

def beam_center(tgt):
    h=havg()
    i=round(2+((tgt-h)%360)/22.5)%16
    v=ranges()[i]
    if v<0: v=ranges()[i]
    return v

cur=2; lastbeacon=0
for it in range(2000):
    st=status()
    if athere(st):
        stop(); log('AT GOAL',st)
        while True:
            tx('botB FOUND GOAL. staying. climb d5 to me.'); time.sleep(2)
            s2=status(); log(s2)
    if time.time()-lastbeacon>12:
        tx('botB d5=%.3f %s'%(d5val(3),st)); lastbeacon=time.time()
    order=[(cur-1)%4,cur,(cur+1)%4,(cur+2)%4]
    choice=None
    for k in order:
        if beam_center(AXES[k])>0.6: choice=k; break
    if choice is None:
        log('boxed; spin'); motors(11,-11); time.sleep(0.35); stop(); continue
    tgt=AXES[choice]
    rot_to(tgt, tol=3)
    why,m=push_catch(tgt)
    log('it',it,'dir',choice,why,'moved',round(m,2),'d5',round(d5val(2),3),'h',round(havg(),1),status())
    if m>0.25: cur=choice
    elif why in ('end','time'):
        motors(-7,-7); time.sleep(0.4); stop()
stop()
