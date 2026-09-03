import sys,time,math,json,collections
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive, angdiff

r=Robot(); d=Drive(r)
log=open('/memory/run.log','a')
track=open('/memory/track.csv','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== explore start ===')

AXES=[5,95,185,275]
CELL=0.45
visits=collections.Counter()
def cellof(x,y): return (round(x/CELL), round(y/CELL))

t0=time.time()
while r.h is None or r.rays is None:
    r.update(); time.sleep(0.05)
    if time.time()-t0>5: break

last_tx=0; cur=None; step=0
goal_seen=False
while True:
    r.update()
    now=time.time()
    for m in r.msgs: L('RX:',m)
    r.msgs=[]
    for e in r.events:
        if 'goal=0' not in e: L('EV:',e)
        if 'goal=1' in e:
            goal_seen=True; L('GOAL REACHED at pose',r.x,r.y)
    r.events=[]
    if goal_seen:
        r.wheels(0,0)
        r.tx.write(json.dumps({'id':'A','at_goal':1,'t':round(now)}))
        time.sleep(1); continue
    if now-last_tx>2:
        r.tx.write(json.dumps({'id':'A','x':round(r.x,2),'y':round(r.y,2),'g':r.goal}))
        last_tx=now
    # clearance in each axis direction
    clear={}
    for ax in AXES:
        rel=(ax-r.h)%360
        k=rel/22.5
        k0=int(k)%16; k1=(k0+1)%16
        w=k-int(k)
        v0=r.ray(k0); v1=r.ray(k1)
        vals=[v for v in (v0,v1) if v is not None]
        clear[ax]=min(vals) if vals else 3.0
    # score
    best=None; bs=-9
    for ax in AXES:
        c=clear[ax]
        if c<CELL+0.12: continue
        a=math.radians(ax)
        nc=cellof(r.x+CELL*math.cos(a), r.y+CELL*math.sin(a))
        s=min(c,1.5) - 1.2*visits[nc]
        if cur is not None:
            if ax==cur: s+=0.4
            if (ax-cur)%360==180: s-=0.8
        if s>bs: bs=s; best=ax
    step+=1
    track.write('%.1f,%.2f,%.2f,%s,%s,%s\n'%(now,r.x,r.y,r.h,r.goal,r.d5.last)); track.flush()
    if best is None:
        # dead end: turn around
        cur=(cur+180)%360 if cur is not None else 185
        L('deadend turn %s clear=%s pose %.2f %.2f'%(cur,clear,r.x,r.y))
        d.turn_to(cur)
        continue
    cur=best
    visits[cellof(r.x,r.y)]+=1
    if abs(angdiff(cur,r.h))>10:
        d.turn_to(cur)
    tr,reason=d.forward(CELL,target_h=cur)
    L('step%d dir=%s went %.2f %s pose %.2f %.2f clear %s'%(step,cur,tr,reason,r.x,r.y,{a:round(c,2) for a,c in clear.items()}))
