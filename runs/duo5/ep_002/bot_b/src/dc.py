import sys,time,random
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== dc start ===')
AXES=[5,95,185,275]
DIRV={5:(1,0),95:(0,1),185:(-1,0),275:(0,-1)}
OPP={5:185,185:5,95:275,275:95}
PERP={5:(95,275),185:(95,275),95:(5,185),275:(5,185)}
while r.h is None or r.rays is None: r.update(); time.sleep(0.05)
goal_seen=False; last_tx=0
def poll():
    global goal_seen,last_tx
    r.update()
    for m in r.msgs: L('RX:',m)
    r.msgs[:]=[]
    for e in r.events:
        if 'goal=1' in e: goal_seen=True; L('EV:',e)
    r.events[:]=[]
    if time.time()-last_tx>2:
        r.tx.write('B PARK AND SPIN NOW stay put. A doorcrawling to you.'); last_tx=time.time()
def d5s(t=0.45):
    vals=[]; end=time.time()+t
    while time.time()<end:
        poll()
        if r.d5.last:
            try: vals.append(float(r.d5.last))
            except: pass
        time.sleep(0.03)
    return sum(vals)/max(1,len(vals)) if vals else 0.0
def clr():
    best={ax:0.0 for ax in AXES}
    for s in range(3):
        r.update()
        for ax in AXES:
            rel=((ax-r.h)%360)/22.5
            k0=int(rel)%16; k1=(k0+1)%16
            vals=[v for v in (r.ray(k0),r.ray(k1)) if v is not None]
            if vals: best[ax]=max(best[ax],min(vals))
        time.sleep(0.05)
    return best
def move(ax,dist=0.5,fs=0.24,sp=80):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(dist,target_h=ax,front_stop=fs,speed=sp)
    return tr
def freeze_test(v):
    global last_tx
    r.wheels(0,0)
    L('TRIPWIRE d5=%.2f freeze+test'%v)
    t0=time.time()
    while time.time()-t0<50:
        poll()
        if goal_seen: return True
        if time.time()-last_tx>1.5:
            r.tx.write('A STOP TEST stand still d5 %.2f'%v); last_tx=time.time()
        time.sleep(0.05)
    return False
def hold_goal():
    global last_tx
    L('GOAL HOLD')
    while True:
        r.wheels(0,0); poll()
        if time.time()-last_tx>1: r.tx.write('A at_goal 1 GOAL STAY'); last_tx=time.time()
        time.sleep(0.1)
def check(v):
    if goal_seen: hold_goal()
    if v>1.15:
        if freeze_test(v): hold_goal()
        # test failed: nudge toward open dir and retest happens naturally
def climb():
    v=d5s(0.5); check(v)
    fails=0
    while fails<1:
        c=clr(); improved=False
        for ax in sorted(AXES,key=lambda a:-c[a]):
            if c[ax]<0.42: continue
            tr=move(ax)
            if tr<=0.2: continue
            nv=d5s(0.45); check(nv)
            L('dc climb %s tr=%.2f %.2f->%.2f'%(ax,tr,v,nv))
            if nv>v+0.02:
                v=nv; improved=True; break
            move(OPP[ax],dist=tr,fs=0.15,sp=60)
        if not improved: fails+=1
    return v
def crawl(plateau):
    # find barrier axis by 0.22m probes
    pv={}
    for ax in AXES:
        tr=move(ax,dist=0.22,fs=0.14,sp=50)
        pv[ax]=d5s(0.45); check(pv[ax])
        if tr>0.03: move(OPP[ax],dist=tr,fs=0.12,sp=50)
    bar=max(pv,key=pv.get)
    L('dc barrier=%s probes=%s'%(bar,{a:round(x,2) for a,x in pv.items()}))
    for cax in PERP[bar]:
        back=0
        for k in range(10):
            poll()
            tr=move(bar,fs=0.17,sp=55)
            v=d5s(0.4); check(v)
            if tr>0.35:
                L('dc THROUGH %s tr=%.2f d5=%.2f'%(bar,tr,v))
                return True
            elif tr>0.04:
                move(OPP[bar],dist=tr,fs=0.12,sp=50)
            tr=move(cax,fs=0.20,sp=70)
            v=d5s(0.35); check(v)
            L('dc crawl %s k=%d tr=%.2f d5=%.2f'%(cax,k,tr,v))
            if tr>0.2: back+=1
            else:
                L('dc crawl blocked'); break
            if v<plateau-0.35:
                L('dc too far, back'); break
        for i in range(back):
            if move(OPP[cax],fs=0.20,sp=70)<0.2:
                move(bar if random.random()<0.5 else OPP[bar],fs=0.2)
                move(OPP[cax],fs=0.2,sp=70)
    return False
# main
lastvisit={(0,0):0}; cx,cy=0,0; cur=5; step=0
bestv=0.0; bestpos=(0,0)
while True:
    poll()
    if goal_seen: hold_goal()
    v=d5s(0.3); check(v)
    if v>bestv: bestv=v; bestpos=(cx,cy)
    if v>0.55:
        pk=climb()
        L('dc plateau %.2f'%pk)
        if pk>0.55:
            while True:
                got=crawl(pk)
                if got:
                    pk=climb(); L('dc new plateau %.2f'%pk)
                    if pk>1.0: continue
                else: break
        # crawl failed both dirs: wander away a bit then retry
        L('dc crawl failed, wander burst')
        for i in range(8):
            c=clr()
            opens=[ax for ax in AXES if c[ax]>=0.42]
            if not opens: break
            def key(ax):
                vv=DIRV[ax]; n=(cx+vv[0],cy+vv[1])
                return (lastvisit.get(n,-1), random.random())
            ax=min(opens,key=key)
            tr=move(ax); step+=1
            if tr>0.2:
                vv=DIRV[ax]; cx+=vv[0]; cy+=vv[1]; lastvisit[(cx,cy)]=step
        continue
    c=clr()
    opens=[ax for ax in AXES if c[ax]>=0.42]
    if not opens: time.sleep(0.4); continue
    def key(ax):
        vv=DIRV[ax]; n=(cx+vv[0],cy+vv[1])
        return (lastvisit.get(n,-1), abs(n[0]-bestpos[0])+abs(n[1]-bestpos[1]), random.random())
    ax=min(opens,key=key)
    tr=move(ax); step+=1
    if tr>0.2:
        cur=ax; vv=DIRV[ax]; cx+=vv[0]; cy+=vv[1]; lastvisit[(cx,cy)]=step
        L('dcw%d %s pos %d,%d d5=%.2f'%(step,ax,cx,cy,v))
    else:
        vv=DIRV[ax]; lastvisit[(cx+vv[0],cy+vv[1])]=step+80
