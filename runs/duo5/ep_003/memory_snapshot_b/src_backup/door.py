import sys,time,random
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== door start ===')
AXES=[5,95,185,275]
DIRV={5:(1,0),95:(0,1),185:(-1,0),275:(0,-1)}
PERP={5:(95,275),185:(95,275),95:(5,185),275:(5,185)}
OPP={5:185,185:5,95:275,275:95}
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
        r.tx.write('A doorhunt. B STAY PARKED SPINNING until I say STOP TEST.'); last_tx=time.time()
def hold():
    global last_tx
    L('HOLD')
    while True:
        r.wheels(0,0); poll()
        if time.time()-last_tx>1: r.tx.write('A holding at_goal %d'%(1 if goal_seen else 0)); last_tx=time.time()
        time.sleep(0.1)
def d5s(t=0.5):
    vals=[]; end=time.time()+t
    while time.time()<end:
        poll()
        if r.d5.last:
            try: vals.append(float(r.d5.last))
            except: pass
        time.sleep(0.03)
    return sum(vals)/max(1,len(vals))
def check(v):
    if goal_seen: hold()
    if v>1.3:
        L('door CLOSE d5=%.2f'%v); r.tx.write('A adjacent STOP TEST'); hold()
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
def move(ax,fs=0.18,sp=65,dist=0.5):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(dist,target_h=ax,front_stop=fs,speed=sp)
    return tr
def climb():
    # greedy climb until no improvement
    v=d5s(0.6); check(v)
    while True:
        c=clr()
        improved=False
        for ax in sorted(AXES,key=lambda a:-c[a]):
            if c[ax]<0.42: continue
            tr=move(ax)
            nv=d5s(0.5); check(nv)
            L('d climb %s tr=%.2f %.2f->%.2f'%(ax,tr,v,nv))
            if tr>0.2 and nv>v+0.03:
                v=nv; improved=True; break
            if tr>0.2: move(OPP[ax],dist=tr)
        if not improved: return v
v=climb()
L('door peak d5=%.2f'%v)
# find barrier axis: probe each axis 0.22m, measure d5
best_ax=None; bestpv=-1
c=clr()
for ax in AXES:
    tr=move(ax,dist=0.22,fs=0.15,sp=55)
    pv=d5s(0.5); check(pv)
    move(OPP[ax],dist=tr,fs=0.13,sp=55)
    L('d probe %s tr=%.2f pv=%.2f c=%.2f'%(ax,tr,pv,c[ax]))
    if pv>bestpv: bestpv=pv; best_ax=ax
bar=best_ax
crawls=PERP[bar]
L('door barrier=%s crawl=%s'%(bar,crawls))
for cax in crawls:
    steps=0
    while steps<12:
        poll()
        # try barrier axis
        tr=move(bar,fs=0.17,sp=60)
        v=d5s(0.5); check(v)
        if tr>0.35:
            L('d THROUGH %s tr=%.2f d5=%.2f! climbing'%(bar,tr,v))
            v=climb()
            L('d new peak %.2f'%v)
            if v<1.0:
                # still blocked; restart barrier logic crudely: keep climbing loop
                continue
        elif tr>0.05:
            move(OPP[bar],dist=tr,fs=0.13,sp=55)
        # crawl one cell
        tr=move(cax,fs=0.20,sp=70)
        steps+=1
        v=d5s(0.4); check(v)
        L('d crawl %s tr=%.2f d5=%.2f'%(cax,tr,v))
        if tr<0.2:
            L('d crawl blocked %s'%cax); break
        if v<0.45:
            L('d d5 low, reversing'); 
            # go back toward peak
            for i in range(steps):
                move(OPP[cax],fs=0.2,sp=70)
            break
L('door done loop; fallback climb forever')
while True:
    v=climb()
    L('door recl peak %.2f'%v)
    ax=random.choice(AXES)
    move(ax)
