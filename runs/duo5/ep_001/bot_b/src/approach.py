import sys,time,math,statistics
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== approach start ===')
while r.h is None or r.rays is None: r.update(); time.sleep(0.05)
goal_seen=False; last_tx=0
def poll():
    global goal_seen,last_tx
    r.update()
    for m in r.msgs: L('RX:',m)
    r.msgs[:]=[]
    for e in r.events:
        if 'goal=0' not in e: L('EV:',e)
        if 'goal=1' in e: goal_seen=True
    r.events[:]=[]
    if time.time()-last_tx>2:
        r.tx.write('A approaching goal %d'%(1 if goal_seen else 0)); last_tx=time.time()

def scan_motion(t=6.0):
    # collect frames, return per-ray min,max,var
    frames=[]
    end=time.time()+t
    lastline=None
    while time.time()<end:
        poll()
        if r.lidar.last!=lastline and r.rays:
            frames.append(list(r.rays)); lastline=r.lidar.last
        time.sleep(0.05)
    stats=[]
    for i in range(16):
        vals=[f[i] for f in frames if f[i]>=0]
        if len(vals)<3: stats.append((9,0,0)); continue
        stats.append((min(vals),max(vals),max(vals)-min(vals)))
    return stats

def d5v():
    try: return float(r.d5.last)
    except: return 0.0

while True:
    poll()
    if goal_seen:
        L('GOAL SEEN'); r.wheels(0,0)
        while True:
            poll(); time.sleep(0.2)
            if time.time()-last_tx>1: r.tx.write('A at_goal 1'); last_tx=time.time()
    st=scan_motion(6.0)
    moving=[(i,s) for i,s in enumerate(st) if s[2]>0.15 and s[0]<2.0]
    L('motion rays: '+', '.join('r%d min%.2f max%.2f d%.2f'%(i,s[0],s[1],s[2]) for i,s in moving)+' d5=%.2f h=%.1f'%(d5v(),r.h))
    if not moving:
        L('no motion seen; full rays='+','.join('%.2f'%(x if x else -1) for x in r.rays))
        time.sleep(2); continue
    # target: moving ray with smallest min distance
    i,s=min(moving,key=lambda t:t[1][0])
    bearing=(r.h+22.5*i)%360
    L('target blob ray %d bearing %.0f dist %.2f'%(i,bearing,s[0]))
    d.turn_to(bearing,tol=8)
    tr,reason=d.forward(max(0.05,s[0]-0.25),target_h=bearing,front_stop=0.20,speed=60)
    L('advanced %.2f toward blob d5=%.2f'%(tr,d5v()))
