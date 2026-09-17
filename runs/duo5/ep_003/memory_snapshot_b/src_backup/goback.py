import sys,time
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== goback south to hotspot ===')
AXES=[5,95,185,275]
goal_seen=False; last_tx=0
MSG=('B: DO THIS: walk SOUTH down your 2.6m corridor ONE CELL AT A TIME. At EACH cell check East and West clearance. '
     'If any side >0.45m, TAKE that side opening, then PARK+SPIN and report. If you reach the south end wall with no openings, '
     'park there, SPIN NONSTOP, and report all clearances. The wall at your corridor south end has ME ~1m behind it.')
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
def d5s(t=0.7):
    vals=[]; end=time.time()+t
    while time.time()<end:
        poll()
        if r.d5.last:
            try: vals.append(float(r.d5.last))
            except: pass
        time.sleep(0.03)
    return sum(vals)/max(1,len(vals)) if vals else 0.0
def hold():
    global last_tx
    while True:
        r.wheels(0,0); poll()
        if goal_seen:
            L('GOAL HOLD')
            while True:
                poll(); r.wheels(0,0)
                if time.time()-last_tx>1: r.tx.write('A at_goal 1 GOAL STAY PUT'); last_tx=time.time()
                time.sleep(0.1)
        time.sleep(0.05)
while r.h is None or r.rays is None: r.update(); time.sleep(0.05)
d.turn_to(275)
prev=d5s(0.6); best=prev; bestidx=0
for i in range(60):
    poll()
    if goal_seen: hold()
    tr,_=d.forward(0.5,target_h=275,front_stop=0.20,speed=30)
    v=d5s(0.6)
    L('gb%d tr=%.2f d5=%.2f'%(i,tr,v))
    if v>best: best=v; bestidx=i
    if tr<0.2:
        L('gb blocked'); break
    if v>0.80 and v<prev-0.03 and best>0.85:
        L('gb passed peak, backing 1')
        d.turn_to(95); d.forward(0.5,target_h=95,front_stop=0.20,speed=28)
        break
    prev=v
v=d5s(0.8)
L('gb hold at d5=%.2f rays=%s h=%s'%(v,r.lidar.last,r.h))
hold()
