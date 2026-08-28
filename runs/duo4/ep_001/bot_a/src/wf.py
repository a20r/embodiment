import time, math, sys
sys.path.insert(0,'/bot/src')
from bot import IO, clean

io = IO()
log = open('/memory/wf.log','a', buffering=1)
T0=time.time()
prev=[None]
lastping=[0.0]
lastlog=[0.0]
goal_flag=[False]

def now(): return time.time()-T0

def poll(t=0.04):
    io.poll(t)
    tn=time.time()
    if tn-lastping[0]>2:
        lastping[0]=tn
        io.send('PING alpha')
    for m in io.msgs:
        log.write('%.1f RX %s\n'%(now(),m)); print('RX',m,flush=True)
    io.msgs=[]
    st=io.latest.get(9,'')
    if 'goal=1' in st and not goal_flag[0]:
        goal_flag[0]=True
        log.write('%.1f GOALFLAG %s\n'%(now(),st))

def sensors():
    while True:
        poll()
        l=io.lidar(); h=io.heading()
        if l and h is not None:
            l=clean(l,prev[0]); prev[0]=l
            return l,h

def logstate(tag,l,h):
    log.write('%.1f %s h=%.1f l=%s d0=%s d6=%s d7=%s %s\n'%(now(),tag,h,
        ','.join('%.2f'%v for v in l),io.latest.get(0),io.latest.get(6),io.latest.get(7),io.latest.get(9)))

state='fwd'
tstate=time.time()
lastsnap=None; lastsnapt=time.time(); stuck_count=0
recov_stage=0; recov_t=0

while True:
    l,h=sensors()
    t=time.time()
    F=min(l[0],l[1],l[15]); R=min(l[12],l[13]); FR=l[14]; L=min(l[3],l[4])
    if t-lastlog[0]>2:
        lastlog[0]=t; logstate(state,l,h)
    if state=='fwd':
        if F<0.25:
            state='turnleft'; tstate=t
            io.drive(0,0); continue
        err=R-0.25
        turn=90*err
        if FR<0.25: turn-=35
        if l[15]<0.28: turn-=35
        if l[1]<0.28: turn+=25
        if L<0.15: turn+=20   # push away from left wall
        turn=max(-50,min(50,turn))
        spd=5 if F>0.5 else 2.5
        io.drive(turn,spd)
        # stuck check
        if lastsnap and t-lastsnapt>1.2:
            d=max(abs(a-b) for a,b in zip(l,lastsnap))
            if d<0.05:
                stuck_count+=1
                log.write('%.1f STUCK d=%.3f cnt=%d\n'%(now(),d,stuck_count))
                state='recover'; recov_stage=0; tstate=t
            lastsnap=l; lastsnapt=t
        elif not lastsnap or t-lastsnapt>1.2:
            lastsnap=l; lastsnapt=t
    elif state=='turnleft':
        io.drive(-60,0)
        if F>0.32 and l[0]>0.3:
            io.drive(0,0); state='fwd'; lastsnap=None
        elif t-tstate>7:
            # give up: face most open ray
            io.drive(0,0)
            state='fwd'; lastsnap=None
    elif state=='recover':
        # wiggle: back, rotate away from nearer side, forward
        ph=t-tstate
        if ph<1.0:
            io.drive(-40 if L<R else 40, -3)
        elif ph<2.0:
            io.drive(-40 if L<R else 40, 3)
        else:
            state='fwd'; lastsnap=None
    time.sleep(0.02)
