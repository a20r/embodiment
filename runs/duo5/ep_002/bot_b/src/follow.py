import time, math, sys, json
sys.path.insert(0,'/bot/src')
from robot import Robot

r=Robot()
log=open('/memory/run.log','a')
def L(*a):
    s=' '.join(str(x) for x in a)
    log.write('%.1f %s\n'%(time.time(),s)); log.flush()

L('=== follow start ===')
BASE=60.0
TARGET=0.35   # desired side wall distance (ray 12 side = heading-90 = follow LEFTish? we'll follow ray4 side)
last_tx=0; last_log=0
state='follow'
t_state=time.time()
try:
  while True:
    r.update()
    now=time.time()
    if r.msgs:
        for m in r.msgs: L('RX:',m)
        r.msgs=[]
    for e in r.events:
        L('EV:',e)
        if 'goal=1' in e or ('goal=' in e and 'goal=0' not in e):
            L('GOAL FLAG!', e)
    r.events=[]
    if now-last_tx>2:
        r.tx.write(json.dumps({'id':'A','x':round(r.x,2),'y':round(r.y,2),'h':r.h,'g':r.goal,'t':round(now)}))
        last_tx=now
    if not r.rays:
        time.sleep(0.05); continue
    front=r.rmin([15,0,1])
    side=r.rmin([3,4,5])     # follow wall on ray4 side
    diag=r.rmin([2,3])
    if now-last_log>1.5:
        L('pose %.2f %.2f h=%s f=%.2f s=%.2f goal=%s d5=%s d2=%s d9=%s'%(r.x,r.y,r.h,front,side,r.goal,r.d5.last,r.d2.last,r.d9.last))
        last_log=now
    # simple right(ray4)-wall follower
    if front<0.30:
        # turn away from side wall (toward ray12 side): heading decrease => left wheel slower
        r.wheels(-35,35)
    elif side>1.2:
        # lost wall: arc toward ray4 side
        r.wheels(45,15)
    else:
        err=side-TARGET
        corr=max(-25,min(25, err*80))
        # positive err: too far from wall -> steer toward ray4 side => heading increase => left faster
        r.wheels(BASE+corr, BASE-corr)
    time.sleep(0.08)
except KeyboardInterrupt:
    pass
finally:
    r.wheels(0,0); L('follow exit')
