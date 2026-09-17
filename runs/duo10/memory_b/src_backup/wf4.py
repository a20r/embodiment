import time, math
from rob import *
from nav import rot_to, d5val
from drv2 import hunt_drive, AXES, front

LOG=open('/tmp/nav.log','a')
def log(*a):
    LOG.write('WF4 '+' '.join(str(x) for x in a)+'\n'); LOG.flush()

def beam_center(tgt):
    h=heading()
    i=round(2+((tgt-h)%360)/22.5)%16
    r=ranges()
    v=r[i]
    if v<0:
        time.sleep(0.15); v=ranges()[i]
    return v

cur=2; lastbeacon=0; pos=[0.0,0.0]; blocked=set()
for it in range(1000):
    st=status()
    if 'here=1' in st or 'goal=1' in st:
        stop(); log('AT GOAL',st)
        while True:
            tx('botB FOUND GOAL. staying put. climb d5 to me.'); time.sleep(2); log(status())
    if time.time()-lastbeacon>10:
        tx('botB d5=%.3f %s'%(d5val(3),st)); lastbeacon=time.time()
    order=[(cur-1)%4,cur,(cur+1)%4,(cur+2)%4]
    choice=None
    for k in order:
        if k in blocked: continue
        if beam_center(AXES[k])>0.6: choice=k; break
    if choice is None:
        blocked=set(); log('boxed/all blocked; spin'); motors(11,-11); time.sleep(0.4); stop(); continue
    why,m=hunt_drive(choice, tmax=25)
    d5=d5val(3)
    log('it',it,'dir',choice,'why',why,'moved',round(m,2),'d5',round(d5,3),'h',round(heading(),1),'ax',round(AXES[choice],1),status())
    if why=='HERE': continue
    if m>0.2:
        a=math.radians(AXES[choice])
        pos[0]+=m*math.sin(a); pos[1]+=m*math.cos(a)
        log('pos',round(pos[0],2),round(pos[1],2))
        cur=choice; blocked=set()
    else:
        blocked.add(choice)
        if why in ('nolock','stuck'):
            motors(-7,-7); time.sleep(0.5); stop()
stop()
