import time, math, json
from rob import *
from nav import rot_to, d5val
import drv2
from drv2 import hunt_drive, AXES, front

LOG=open('/tmp/nav.log','a')
def log(*a):
    LOG.write('WF3 '+' '.join(str(x) for x in a)+'\n'); LOG.flush()

def beam_for(tgt):
    h=heading()
    i=round(2+((tgt-h)%360)/22.5)%16
    return i

def open_dir(axkey):
    tgt=AXES[axkey]
    r=ranges()
    i=beam_for(tgt)
    vals=[r[i]]+[r[(i+1)%16]]+[r[(i-1)%16]]
    vals=[v for v in vals if v>0]
    return max(vals) if vals else 0

cur=2  # current axis key guess (heading ~215)
lastbeacon=0
pos=[0.0,0.0]  # dead reckoning in axis frame
for it in range(400):
    st=status()
    if 'here=1' in st or 'goal=1' in st:
        stop(); log('AT GOAL',st)
        while True:
            tx('botB FOUND GOAL. I am staying. climb d5 to me.'); time.sleep(2); log(status())
    if time.time()-lastbeacon>10:
        tx('botB d5=%.3f %s'%(d5val(3),st)); lastbeacon=time.time()
    # left-hand rule: left, straight, right, back  (left = CCW = key-1)
    order=[(cur-1)%4,(cur)%4,(cur+1)%4,(cur+2)%4]
    choice=None
    for k in order:
        o=open_dir(k)
        if o>0.72: choice=k; break
    if choice is None:
        log('boxed in; spin'); motors(11,-11); time.sleep(0.6); stop(); continue
    why,m=hunt_drive(choice, tmax=14)
    d5=d5val(3)
    log('it',it,'dir',choice,'why',why,'moved',round(m,2),'d5',round(d5,3),'h',round(heading(),1),status())
    if why=='HERE':
        continue
    if why!='nolock' and m>0.2:
        a=math.radians(AXES[choice])
        pos[0]+=m*math.sin(a); pos[1]+=m*math.cos(a)
        log('pos',round(pos[0],2),round(pos[1],2))
        cur=choice
    elif why in ('nolock','stuck'):
        motors(-7,-7); time.sleep(0.5); stop()
stop()
