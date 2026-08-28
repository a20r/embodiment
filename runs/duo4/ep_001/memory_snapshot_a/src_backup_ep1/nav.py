import time, math, sys, os
sys.path.insert(0,'/bot/src')
from bot import IO, clean

io = IO()
log = open('/memory/trail.log','a', buffering=1)
T0 = time.time()
x=y=0.0
prev=[None]
lastping=[0.0]

def L():
    l = io.lidar()
    if l is None: return None
    l = clean(l, prev[0]); prev[0]=l
    return l

def wrap(a): return (a+180)%360-180

def poll(t=0.05):
    io.poll(t)
    now=time.time()
    if now-lastping[0]>2:
        lastping[0]=now
        io.send('PING alpha x=%.2f y=%.2f'%(x,y))
        log.write('%.1f POS x=%.2f y=%.2f h=%s d0=%s d6=%s d7=%s %s\n'%(now-T0,x,y,io.latest.get(2),io.latest.get(0),io.latest.get(6),io.latest.get(7),io.latest.get(9)))
    for m in io.msgs:
        log.write('%.1f RX %s\n'%(now-T0,m)); print('RX',m,flush=True)
    io.msgs=[]
    st=io.latest.get(9,'')
    if 'goal=1' in st:
        log.write('%.1f GOAL %s x=%.2f y=%.2f\n'%(now-T0,st,x,y))

def turn_to(target, tol=5):
    t0=time.time()
    while time.time()-t0<15:
        time.sleep(0.02); poll(0.03)
        h=io.heading()
        if h is None: continue
        d=wrap(target-h)
        if abs(d)<tol:
            io.drive(0,0); return True
        mag=min(80, max(12, abs(d)*1.5))
        io.drive(-mag if d>0 else mag, 0)
    io.drive(0,0); return False

def ray(l, h, wd):
    idx = int(round(((wd-h)%360)/22.5))%16
    return l[idx]

def forward(dist, target_h):
    # drive ~dist meters holding target_h, centering; stop early if front blocked
    global x,y
    MPS=0.5
    t0=time.time(); dur=dist/MPS
    last=t0
    while True:
        time.sleep(0.02); poll(0.03)
        now=time.time()
        h=io.heading(); l=L()
        if h is None or l is None: continue
        dt=now-last; last=now
        if now-t0>dur: break
        f=min(l[0],l[1],l[15])
        if f<0.22: break
        dh=wrap(target_h-h)
        turn=-dh*2.5
        r=l[12]; lf=l[4]
        if r<0.45 and lf<0.45:
            turn += 35*(r-lf)
        elif r<0.30: turn -= 25*(0.30-r)*4
        elif lf<0.30: turn += 25*(0.30-lf)*4
        turn=max(-60,min(60,turn))
        spd = 5 if f>0.5 else 2
        io.drive(turn,spd)
        v=MPS*(spd/5.0)
        x+=v*dt*math.cos(math.radians(h)); y+=v*dt*math.sin(math.radians(h))
    io.drive(0,0)

def snapshot():
    t0=time.time()
    while True:
        poll(0.03)
        if io.heading() is not None and io.lidar() is not None: break
        if time.time()-t0>5: raise RuntimeError('no sensors')
    for _ in range(4): poll(0.03)
    return L(), io.heading()

def open_dirs():
    l,h=snapshot()
    res={}
    for wd in (0,90,180,270):
        # use min of ray and neighbors to be safe? use exact ray
        idx = int(round(((wd-h)%360)/22.5))%16
        res[wd]=l[idx]
    return res,h

heading0 = None
# main loop: right-hand rule
cur = None
while True:
    dirs,h = open_dirs()
    if cur is None:
        # initial: face most open
        cur = max(dirs, key=lambda k:dirs[k])
    order = [(cur-90)%360, cur, (cur+90)%360, (cur+180)%360]
    choice=None
    for d in order:
        if dirs[d] > 0.45:
            choice=d; break
    if choice is None:
        choice=max(dirs,key=lambda k:dirs[k])
    hh=io.heading()
    if hh is None or abs(wrap(choice-hh))>10:
        turn_to(choice)
    cur=choice
    forward(0.45, cur)
