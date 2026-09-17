import sys,time
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== sweep south start ===')
goal_seen=False; last_tx=0
MSG='B: NEW PLAN. WALK SOUTH to the far end of your 2.6m corridor. PARK AND SPIN there. REPORT clearances N E S W and d5 from there. I sweep my parallel corridor south checking every side opening.'
def poll():
    global goal_seen,last_tx
    r.update()
    for m in r.msgs: L('RX:',m)
    r.msgs[:]=[]
    for e in r.events:
        if 'goal=1' in e: goal_seen=True; L('EV:',e)
    r.events[:]=[]
    if time.time()-last_tx>3:
        r.tx.write(MSG); last_tx=time.time()
def d5s(t=0.8):
    vals=[]; end=time.time()+t
    while time.time()<end:
        poll()
        if r.d5.last:
            try: vals.append(float(r.d5.last))
            except: pass
        time.sleep(0.03)
    return sum(vals)/max(1,len(vals)) if vals else 0.0
def axclr(ax):
    r.update()
    rel=((ax-r.h)%360)/22.5
    k0=int(rel)%16; k1=(k0+1)%16
    vals=[v for v in (r.ray(k0),r.ray(k1)) if v is not None and v>=0]
    return min(vals) if vals else 0.0
def hold_goal():
    global last_tx
    L('GOAL HOLD')
    while True:
        r.wheels(0,0); poll()
        if time.time()-last_tx>1: r.tx.write('A at_goal 1 GOAL STAY PUT'); last_tx=time.time()
        time.sleep(0.1)
while r.h is None or r.rays is None: r.update(); time.sleep(0.05)
d.turn_to(275)
for i in range(14):
    poll()
    if goal_seen: hold_goal()
    v=d5s(0.8)
    e=axclr(5); w=axclr(185); s=axclr(275); n=axclr(95)
    L('sweep cell %d d5=%.2f E=%.2f W=%.2f S=%.2f N=%.2f'%(i,v,e,w,s,n))
    if v>1.1:
        L('TRIPWIRE %.2f'%v); r.wheels(0,0)
        t0=time.time()
        while time.time()-t0<70:
            poll()
            if goal_seen: hold_goal()
            time.sleep(0.05)
    if s<0.30:
        L('sweep: south blocked, stopping here'); break
    tr,_=d.forward(0.5,target_h=275,front_stop=0.20,speed=28)
    if tr<0.2: L('sweep: truncated tr=%.2f'%tr); break
# final report
v=d5s(1.0)
L('sweep end d5=%.2f rays=%s h=%s'%(v,r.lidar.last,r.h))
while True:
    poll()
    if goal_seen: hold_goal()
    r.wheels(0,0); time.sleep(0.1)
