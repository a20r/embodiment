import sys,time
sys.path.insert(0,'/bot/src')
from robot import Robot
r=Robot()
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== spin3 start: A parks+spins, B homes ===')
goal_seen=False; last_tx=0; lastpos=None
MSG='B: GO TO YOUR COORD 9 -2 (your frame, where you were when d5 peaked 0.92). PARK AND SPIN NONSTOP there. A is parked spinning adjacent. Climb d5 the last cells. FREEZE when d5>1.1.'
t_hi=None
def poll():
    global goal_seen,last_tx,lastpos
    r.update()
    for m in r.msgs:
        L('RX:',m)
        if ' pos ' in m: lastpos=m
    r.msgs[:]=[]
    for e in r.events:
        if 'goal=1' in e: goal_seen=True; L('EV:',e)
    r.events[:]=[]
    if time.time()-last_tx>2:
        r.tx.write(MSG); last_tx=time.time()
def d5v():
    try: return float(r.d5.last)
    except: return 0.0
mode='SPIN'
t0=time.time(); lastlog=0
while True:
    poll()
    v=d5v()
    if goal_seen:
        r.wheels(0,0)
        L('GOAL HOLD')
        while True:
            poll(); r.wheels(0,0)
            if time.time()-last_tx>1: r.tx.write('A at_goal 1 GOAL STAY PUT'); last_tx=time.time()
            time.sleep(0.1)
    if mode=='SPIN':
        r.wheels(45,-45)
        if v>1.05:
            if t_hi is None: t_hi=time.time()
            elif time.time()-t_hi>2.0:
                mode='FREEZE'; tf=time.time(); L('FREEZE d5=%.2f'%v)
        else: t_hi=None
    else: # FREEZE: quiet, test goal co-location
        r.wheels(0,0)
        if time.time()-last_tx>1.5:
            r.tx.write('A STOP TEST freeze, d5 %.2f, testing goal'%v); last_tx=time.time()
        if time.time()-tf>75:
            L('freeze no goal, resume spin (d5=%.2f)'%v)
            mode='SPIN'; t_hi=None
    if time.time()-lastlog>6:
        L('spin3 mode=%s d5=%.2f lastB=%s'%(mode,v,lastpos)); lastlog=time.time()
    time.sleep(0.05)
