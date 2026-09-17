import sys,time,random
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== fin4 basin explore + lidar variance detector ===')
AXES=[5,95,185,275]
DIRV={5:(1,0),95:(0,1),185:(-1,0),275:(0,-1)}
OPP={5:185,185:5,95:275,275:95}
goal_seen=False; last_tx=0
MSG='A: in your 0.9 zone exploring every opening; watching lidar for your rocking. Keep rocking BIG amplitude. FREEZE at d5>1.02.'
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
def d5s(t=0.5):
    vals=[]; end=time.time()+t
    while time.time()<end:
        poll()
        if r.d5.last:
            try: vals.append(float(r.d5.last))
            except: pass
        time.sleep(0.03)
    return sum(vals)/max(1,len(vals)) if vals else 0.0
def hold_goal():
    global last_tx
    L('GOAL HOLD')
    while True:
        r.wheels(0,0); poll()
        if time.time()-last_tx>1: r.tx.write('A at_goal 1 GOAL STAY PUT'); last_tx=time.time()
        time.sleep(0.1)
def chk(v):
    global last_tx
    if goal_seen: hold_goal()
    if v>1.02:
        r.wheels(0,0); L('TRIPWIRE %.2f'%v)
        t0=time.time()
        while time.time()-t0<90:
            poll()
            if goal_seen: hold_goal()
            if time.time()-last_tx>1.5:
                r.tx.write('A STOP TEST freeze d5 %.2f'%v); last_tx=time.time()
            time.sleep(0.05)
        L('trip no goal, continue')
def mv(ax,fs=0.13,sp=24,dist=0.5):
    if abs(angdiff(ax,r.h))>8: d.turn_to(ax)
    tr,_=d.forward(dist,target_h=ax,front_stop=fs,speed=sp)
    return tr
def variance_scan(t=2.2):
    r.wheels(0,0)
    lo=[9e9]*16; hi=[-9e9]*16
    end=time.time()+t
    h0=None
    while time.time()<end:
        poll()
        if r.h and h0 is None: h0=r.h
        for k in range(16):
            v=r.ray(k)
            if v is None or v<0: continue
            lo[k]=min(lo[k],v); hi[k]=max(hi[k],v)
        time.sleep(0.04)
    spread=[(hi[k]-lo[k]) if hi[k]>-1 else 0 for k in range(16)]
    mk=max(range(16),key=lambda k:spread[k])
    return mk,spread[mk],(h0 or r.h),lo[mk]
while r.h is None or r.rays is None: r.update(); time.sleep(0.05)
cx,cy=0,0; step=0; vis={(0,0):0}
while True:
    poll()
    if goal_seen: hold_goal()
    v=d5s(0.5); chk(v)
    mk,sp,h0,dist=variance_scan(2.2)
    if sp>0.14 and dist<2.5:
        tgt=(h0+22.5*mk)%360
        L('f4 MOTION ray%d spread %.2f dist %.2f -> heading %.0f d5=%.2f'%(mk,sp,dist,tgt,v))
        d.turn_to(min(AXES,key=lambda a:abs(angdiff(a,tgt))))
        tr,_=d.forward(min(0.45,max(0.2,dist-0.18)),front_stop=0.16,speed=22)
        nv=d5s(0.6); chk(nv)
        L('f4 chase tr=%.2f d5 %.2f->%.2f'%(tr,v,nv))
        continue
    # basin explore: prefer unvisited any-direction (accept move, no revert)
    moved=False
    def key(ax):
        n=(cx+DIRV[ax][0],cy+DIRV[ax][1])
        return (vis.get(n,-1), random.random())
    for ax in sorted(AXES,key=key):
        tr=mv(ax)
        step+=1
        if tr>0.3:
            cx+=DIRV[ax][0]; cy+=DIRV[ax][1]; vis[(cx,cy)]=step
            nv=d5s(0.45); chk(nv)
            L('f4 %s %d,%d d5=%.2f'%(ax,cx,cy,nv))
            if nv<v-0.12:
                mv(OPP[ax],fs=0.11,sp=20,dist=tr)  # fell off zone: back
                cx-=DIRV[ax][0]; cy-=DIRV[ax][1]
            moved=True; break
        elif tr>0.03:
            mv(OPP[ax],fs=0.10,sp=18,dist=tr)
            vis[(cx+DIRV[ax][0],cy+DIRV[ax][1])]=step+400
    if not moved: time.sleep(0.2)
