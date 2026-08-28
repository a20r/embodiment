import time, math, sys
sys.path.insert(0,'/bot/src')
from bot import IO, clean

io = IO()
log = open('/memory/crawl.log','a', buffering=1)
T0=time.time()
prev=[None]; lastping=[0.0]
def now(): return time.time()-T0
def wrap(a): return (a+180)%360-180

def poll(t=0.04):
    io.poll(t)
    tn=time.time()
    if tn-lastping[0]>2:
        lastping[0]=tn
        io.send('PING alpha')
        st=io.latest.get(9,'')
        log.write('%.1f HB d0=%s d6=%s d7=%s %s\n'%(now(),io.latest.get(0),io.latest.get(6),io.latest.get(7),st))
    for m in io.msgs:
        log.write('%.1f RX %s\n'%(now(),m)); print('RX',m,flush=True)
    io.msgs=[]

def sensors():
    while True:
        poll()
        l=io.lidar(); h=io.heading()
        if l and h is not None:
            l=clean(l,prev[0]); prev[0]=l
            return l,h

def turn_to(target, tol=3):
    t0=time.time()
    while time.time()-t0<12:
        l,h=sensors()
        d=wrap(target-h)
        if abs(d)<tol:
            io.drive(0,0)
            # settle
            for _ in range(3): sensors()
            d=wrap(target-io.heading())
            if abs(d)<tol: return True
            continue
        mag=min(70,max(8,abs(d)*1.2))
        io.drive(-mag if d>0 else mag,0)
        time.sleep(0.02)
    io.drive(0,0); return False

def ray(l,h,wd):
    return l[int(round(((wd-h)%360)/22.5))%16]

def crawl(wd, maxdist=10.0):
    """Crawl along world direction wd. Returns (reason, dist_moved)."""
    dist=0.0; last=time.time()
    lastsnap=None; lastsnapt=time.time(); stucks=0
    t0=time.time()
    while True:
        l,h=sensors()
        t=time.time(); dt=t-last; last=t
        F=min(l[0],l[15],l[1])
        if F<0.23:
            io.drive(0,0); return 'wall',dist
        if dist>maxdist:
            io.drive(0,0); return 'max',dist
        # openings to the side after some progress
        if dist>0.35:
            rr=ray(l,h,(wd-90)%360)
            if rr>0.7:
                io.drive(0,0); return 'right_open',dist
        dh=wrap(wd-h)
        lft=min(l[3],l[4],l[5]); rgt=min(l[11],l[12],l[13])
        turn=-1.8*dh
        if lft<0.5 and rgt<0.5:
            turn+=90*(rgt-lft)
        elif lft<0.18: turn+=25
        elif rgt<0.18: turn-=25
        turn=max(-30,min(30,turn))
        spd=4 if F>0.6 else 1.5
        io.drive(turn,spd)
        dist+=0.1*spd*dt  # rough m at cmd (0.5m/s at 5)
        # stuck detection
        if lastsnap is None or t-lastsnapt>1.3:
            if lastsnap is not None:
                d=max(abs(a-b) for a,b in zip(l,lastsnap))
                if d<0.05:
                    stucks+=1
                    log.write('%.1f STUCK@%s cnt=%d dist=%.2f\n'%(now(),wd,stucks,dist))
                    dist-=0.65  # didn't actually move
                    if stucks>=4:
                        io.drive(0,0); return 'stuck',dist
                    # recovery: back off, realign
                    io.drive(0,-2); time.sleep(0.8); io.drive(0,0)
                    turn_to(wd,2)
            lastsnap=l; lastsnapt=t
        time.sleep(0.02)

def openness(l,h):
    d={}
    for wd in (0,90,180,270):
        idx=int(round(((wd-h)%360)/22.5))%16
        d[wd]=max(l[idx], l[(idx+1)%16]*0.6, l[(idx-1)%16]*0.6)
    return d

cur=None
blocked={}   # wd -> consecutive stuck failures
while True:
    l,h=sensors()
    dirs=openness(l,h)
    log.write('%.1f NODE h=%.1f dirs=%s l=%s\n'%(now(),h,{k:round(v,2) for k,v in dirs.items()},','.join('%.2f'%v for v in l)))
    if cur is None:
        cur=max(dirs,key=lambda k:dirs[k])
    order=[(cur-90)%360,cur,(cur+90)%360,(cur+180)%360]
    choice=None
    for d in order:
        if dirs[d]>0.5 and blocked.get(d,0)<2:
            choice=d; break
    if choice is None:
        # allow blocked ones again
        blocked={}
        for d in order:
            if dirs[d]>0.5: choice=d; break
        if choice is None: choice=max(dirs,key=lambda k:dirs[k])
    turn_to(choice)
    res,dist=crawl(choice)
    log.write('%.1f CRAWL %s res=%s dist=%.2f\n'%(now(),choice,res,dist))
    if res=='stuck':
        blocked[choice]=blocked.get(choice,0)+1
    else:
        blocked={}
    cur=choice
