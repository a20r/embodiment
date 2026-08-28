import time, math
from robolib2 import Bot, angdiff

b=Bot()
log=open('/memory/telemetry.log','a',buffering=1)
def L(*a): log.write(' '.join(str(x) for x in a)+'\n')
L('=== explore2 start',time.time())

MT=0.00176  # m per odo tick
x,y=0.0,0.0
lasto=b.odo()
t0=time.time()
lastlog=0; lasttx=0
stuck_o=lasto; stuck_t=t0

def upd_pose():
    global x,y,lasto
    o=b.odo(); h=b.heading()
    if h is None: return
    d=(o-lasto)*MT; lasto=o
    x+=d*math.cos(math.radians(h)); y+=d*math.sin(math.radians(h))

try:
  while True:
    t,g=b.status()
    if g:
        b.stop(); L('GOAL! t=%.1f pos %.2f %.2f'%(time.time()-t0,x,y)); print('GOAL'); break
    l=b.lidar()
    upd_pose()
    now=time.time()
    front=l[0]
    fr=l[15]; fl=l[1]
    right=min(l[12], l[11]*0.924, l[13]*0.924)
    left=min(l[4], l[3]*0.924, l[5]*0.924)
    if now-lastlog>1.0:
        L('t=%.0f pos %.2f,%.2f h=%.0f f=%.2f r=%.2f l=%.2f odo=%d lid=%s'%(now-t0,x,y,b.heading() or -1,front,right,left,lasto,','.join('%.2f'%v for v in l)))
        lastlog=now
    if now-lasttx>10:
        b.tx('hello'); lasttx=now
        m=b.rx()
        if m: L('RX:',m)
    # stuck?
    if abs(lasto-stuck_o)>80: stuck_o=lasto; stuck_t=now
    elif now-stuck_t>5:
        L('stuck recover')
        b.wheels(-45,-45); time.sleep(0.7)
        b.wheels(-35,35); time.sleep(0.6)
        b.stop(); stuck_o=b.odo(); stuck_t=time.time(); lasto=stuck_o
        continue
    # control
    if front<0.32 or (front<0.45 and fr<0.25 and fl<0.25):
        # turn left in place until clear
        b.wheels(-35,35)
        time.sleep(0.12)
        continue
    if right>0.55:
        # opening to the right: arc right
        b.wheels(48,20)
        time.sleep(0.12)
        continue
    err=(0.26-right)
    if left<0.18: err+=(0.18-left)*1.5
    err=max(-0.3,min(0.3,err))
    turn=err*140
    sp=50 if front>0.8 else 30
    b.wheels(sp-turn, sp+turn)
    time.sleep(0.08)
finally:
  b.stop()
