import sys,time,math,collections
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== hunt start ===')
AXES=[5,95,185,275]
DIRV={5:(1,0),95:(0,1),185:(-1,0),275:(0,-1)}
while r.h is None or r.rays is None: r.update(); time.sleep(0.05)
goal_seen=False; last_tx=0; b_at_goal=False; last_rx_t=0; last_rx=''
d5buf=collections.deque(maxlen=40)
def poll():
    global goal_seen,last_tx,b_at_goal,last_rx_t,last_rx
    r.update()
    for m in r.msgs:
        L('RX:',m); last_rx=m; last_rx_t=time.time()
        if 'at_goal 1' in m: b_at_goal=True
    r.msgs[:]=[]
    for e in r.events:
        if 'goal=0' not in e: L('EV:',e)
        if 'goal=1' in e: goal_seen=True
    r.events[:]=[]
    if r.d5.last:
        try: d5buf.append(float(r.d5.last))
        except: pass
    if time.time()-last_tx>2:
        r.tx.write('A pos %d %d goal %d d5 %.2f'%(cx,cy,1 if goal_seen else 0,d5a()))
        if int(time.time())%30<2:
            r.tx.write('A proto: if d5 high I will come to you. If you find goal: PARK+SPIN+broadcast GOAL FOUND. If I find it, I do same and you home on d5.')
        last_tx=time.time()
def d5a():
    return sum(d5buf)/len(d5buf) if d5buf else 0.0
def d5sample(t=0.7):
    end=time.time()+t
    vals=[]
    while time.time()<end:
        poll()
        if r.d5.last:
            try: vals.append(float(r.d5.last))
            except: pass
        time.sleep(0.03)
    return sum(vals)/max(1,len(vals))
def clearance():
    best={ax:0.0 for ax in AXES}
    for s in range(3):
        r.update()
        for ax in AXES:
            rel=((ax-r.h)%360)/22.5
            k0=int(rel)%16; k1=(k0+1)%16
            vals=[v for v in (r.ray(k0),r.ray(k1)) if v is not None]
            if vals: best[ax]=max(best[ax],min(vals))
        time.sleep(0.06)
    return best
def move(ax,dist=0.5,fs=0.30):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(dist,target_h=ax,front_stop=fs,speed=90)
    return tr
def at_goal_hold():
    L('AT GOAL park+spin pos %d,%d'%(cx,cy))
    t0=time.time()
    global last_tx
    while True:
        # spin to make noise
        r.wheels(40,-40)
        poll()
        if time.time()-last_tx>1:
            r.tx.write('A GOAL FOUND at_goal 1 come to me climb d5'); last_tx=time.time()
        time.sleep(0.1)
cx,cy=0,0; step=0; cur=5
lastvisit={(0,0):0}
mode='wall'
while True:
    poll()
    if goal_seen: at_goal_hold()
    v=d5a()
    if mode=='wall' and (v>9.55 or b_at_goal): mode='climb'; L('SWITCH to climb d5=%.2f b_at_goal=%s'%(v,b_at_goal))
    if mode=='climb' and v<0.35 and not b_at_goal: mode='wall'; L('SWITCH to wall d5=%.2f'%v)
    c=clearance()
    if mode=='wall':
        opens=[ax for ax in AXES if c[ax]>=0.42]
        if not opens:
            L('boxed c=%s'%c); time.sleep(0.5); continue
        import random
        def key(ax):
            vv=DIRV[ax]; n=(cx+vv[0],cy+vv[1])
            return (lastvisit.get(n,-1), 0 if ax==cur else 1, random.random())
        ax=min(opens,key=key)
        tr=move(ax)
        step+=1
        if tr>0.25:
            cur=ax; vv=DIRV[ax]; cx+=vv[0]; cy+=vv[1]
            lastvisit[(cx,cy)]=step
            L('w%d %s tr=%.2f pos %d,%d d5=%.2f c=%s'%(step,ax,tr,cx,cy,v,{a:round(x,2) for a,x in c.items()}))
        else:
            vv=DIRV[ax]
            lastvisit[(cx+vv[0],cy+vv[1])]=step+1000  # discourage blocked
            L('w%d %s SHORT tr=%.2f'%(step,ax,tr))
    else:
        # climb: sample d5, peek open dirs, move to best
        v0=d5sample(0.6)
        opens=[ax for ax in AXES if c[ax]>0.5]
        L('c d5=%.3f opens=%s pos %d,%d'%(v0,opens,cx,cy))
        if v0>1.8:
            # very close: try co-location - move toward best open slowly, log rays
            L('CLOSE rays=%s h=%.0f'%([None if x is None else round(x,2) for x in [r.ray(i) for i in range(16)]],r.h))
        if not opens:
            time.sleep(0.4); continue
        if len(opens)==1: best=opens[0]
        else:
            best=None; bv=-1
            for ax in opens:
                if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
                tr,_=d.forward(0.2,target_h=ax,front_stop=0.22,speed=70)
                pv=d5sample(0.55)
                bax=min(AXES,key=lambda x:abs(angdiff(x,(ax+180)%360)))
                d.turn_to(bax); d.forward(max(tr,0.05),target_h=bax,front_stop=0.14,speed=70)
                L('peek %s d5=%.3f (base %.3f)'%(ax,pv,v0))
                if pv>bv: bv=pv; best=ax
        tr=move(best)
        if tr>0.25:
            vv=DIRV[best]; cx+=vv[0]; cy+=vv[1]; cur=best
        L('cmove %s tr=%.2f pos %d,%d'%(best,tr,cx,cy))
