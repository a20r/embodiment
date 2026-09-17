import sys,time,math,json
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff

r=Robot(); d=Drive(r)
log=open('/memory/run.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== wallrun start (right-hand) ===')

AXES=[5,95,185,275]
DIRV={5:(1,0),95:(0,1),185:(-1,0),275:(0,-1)}
while r.h is None or r.rays is None:
    r.update(); time.sleep(0.05)

def clearance():
    best={ax:0.0 for ax in AXES}
    for s in range(3):
        r.update()
        for ax in AXES:
            rel=((ax-r.h)%360)/22.5
            k0=int(rel)%16; k1=(k0+1)%16
            vals=[v for v in (r.ray(k0),r.ray(k1)) if v is not None]
            if vals: best[ax]=max(best[ax],min(vals))
        time.sleep(0.07)
    return best

def nrm(a):
    return min(AXES,key=lambda x:abs(angdiff(x,a)))

last_tx=0; goal_seen=False; step=0
cur=5
cx,cy=0.0,0.0
while True:
    r.update()
    now=time.time()
    for m in r.msgs: L('RX:',m)
    r.msgs=[]
    for e in r.events:
        if 'goal=0' not in e: L('EV:',e)
        if 'goal=1' in e: goal_seen=True
    r.events=[]
    if now-last_tx>2:
        r.tx.write('A pos %.2f %.2f goal %d'%(cx,cy,1 if goal_seen else 0))
        if int(now)%20<2:
            r.tx.write('A proto: agreed. if you find goal: park+spin+broadcast GOAL FOUND. I do same. Also try: maybe goal needs both of us co-located; if your d5>1.5 park and spin, I will come.')
        last_tx=now
    if goal_seen:
        L('AT GOAL, holding. steps=%d pos=%.0f,%.0f'%(step,cx,cy))
        r.wheels(0,0)
        while True:
            r.update()
            for m in r.msgs: L('RX:',m)
            r.msgs=[]
            if time.time()-last_tx>1:
                r.tx.write('A at_goal 1'); last_tx=time.time()
            time.sleep(0.2)
    c=clearance()
    order=[nrm(cur+90),nrm(cur),nrm(cur-90),nrm(cur+180)]
    moved=False
    for ax in order:
        if c[ax]<0.42: continue
        if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
        tr,reason=d.forward(0.5,target_h=ax,front_stop=0.23,speed=75)
        step+=1
        if tr>0.28:
            cur=ax; v=DIRV[ax]; cx+=v[0]; cy+=v[1]
            L('w%d %s tr=%.2f pos %.0f,%.0f d5=%s c=%s'%(step,ax,tr,cx,cy,r.d5.last,{a:round(x,2) for a,x in c.items()}))
        else:
            L('w%d %s SHORT tr=%.2f'%(step,ax,tr))
        moved=True
        break
    if not moved:
        L('boxed in?? c=%s'%c); time.sleep(0.5)
