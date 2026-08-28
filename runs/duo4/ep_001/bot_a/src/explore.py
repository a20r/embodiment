import time, math, sys
sys.path.insert(0,'/bot/src')
from bot import IO, clean
io=IO()
log=open('/memory/explore.log','a',buffering=1)
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
        log.write('%.1f HB d0=%s d6=%s %s\n'%(now(),io.latest.get(0),io.latest.get(6),io.latest.get(9)))
    for m in io.msgs:
        log.write('%.1f RX %s\n'%(now(),m)); print('RX',m,flush=True)
    io.msgs=[]
    st=io.latest.get(9,'')
    if 'goal=1' in st:
        log.write('%.1f GOAL! %s\n'%(now(),st))
        io.send('I AM AT GOAL')
def sensors(dur=0.05):
    t=time.time()
    while time.time()-t<dur: poll()
    l=io.lidar(); h=io.heading()
    while l is None or h is None:
        poll(); l=io.lidar(); h=io.heading()
    l=clean(l,prev[0]); prev[0]=l
    return l,h
def turn_to(tgt,tol=5):
    t0=time.time()
    while time.time()-t0<8:
        l,h=sensors(0.03)
        d=wrap(tgt-h)
        if abs(d)<tol: io.drive(0,0); return True
        mag=min(90,max(15,abs(d)*1.5))
        io.drive(-mag if d>0 else mag,0)
        time.sleep(0.02)
    io.drive(0,0); return False
def fan(l,i): return min(l[(i-1)%16],l[i],l[(i+1)%16])
avoid=[]
def pick(l,h,prefer=None):
    tn=time.time(); best=None; bv=-1
    for i in range(16):
        wd=(h+i*22.5)%360
        v=fan(l,i)
        if any(abs(wrap(wd-a))<30 and tn<e for a,e in avoid): v*=0.3
        if prefer is not None: v*=(1.25-0.5*abs(wrap(wd-prefer))/180)
        if v>bv: bv=v; best=wd
    return best
lastsnap=None; lastsnapt=time.time(); stuck=0
heading_tgt=None
while True:
    l,h=sensors()
    f=min(l[0],l[1],l[15]); ffan=fan(l,0)
    if heading_tgt is None: heading_tgt=pick(l,h)
    if f<0.3:
        io.drive(0,-10); time.sleep(0.12); io.drive(0,0)
        l,h=sensors(0.1)
        heading_tgt=pick(l,h,prefer=(heading_tgt-90)%360)
        log.write('%.1f BLOCK f=%.2f h=%.0f new=%.0f l=%s\n'%(now(),f,h,heading_tgt,','.join('%.2f'%v for v in l)))
        turn_to(heading_tgt)
        lastsnap=None; continue
    dh=wrap(heading_tgt-h)
    turn=-2.0*dh
    lft=min(l[3],l[4]); rgt=min(l[12],l[13])
    if lft<0.5 and rgt<0.5: turn+=70*(rgt-lft)
    elif lft<0.17: turn+=20
    elif rgt<0.17: turn-=20
    spd=14 if f>0.6 else 8
    io.drive(max(-50,min(50,turn)),spd)
    t=time.time()
    if lastsnap is None or t-lastsnapt>1.2:
        if lastsnap is not None:
            d=max(abs(a-b) for a,b in zip(l,lastsnap))
            if d<0.05:
                stuck+=1
                log.write('%.1f STUCK cnt=%d tgt=%.0f h=%.1f\n'%(now(),stuck,heading_tgt,h))
                avoid.append((heading_tgt,t+30)); avoid[:]=avoid[-6:]
                io.drive(0,-25); time.sleep(0.5)
                io.drive(0,0)
                l,h=sensors(0.1)
                heading_tgt=pick(l,h)
                turn_to(heading_tgt)
            else: stuck=0
        lastsnap=l; lastsnapt=t
    time.sleep(0.02)
