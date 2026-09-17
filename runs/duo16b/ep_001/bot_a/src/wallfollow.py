import os, select, time, math
D='/dev/robot/'
def read(p, timeout=0.3):
    fd=os.open(D+p, os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,2000000).decode().strip()
        except: out=''
    os.close(fd); return out
def w(p,msg):
    if isinstance(msg,(int,float)): msg=f"{msg}\n"
    try:
        fd=os.open(D+p,os.O_WRONLY|os.O_NONBLOCK); os.write(fd,msg.encode()); os.close(fd)
    except Exception: pass
def fl(x,d=0.0):
    try: return float(x)
    except: return d

def scan():
    s=read('d2'); pts=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            r,e,a=map(float,p.split(',')); pts.append((r,e,a))
        except: pass
    return pts

def wallstats():
    # nearest surface in horizontal band
    pts=scan()
    hor=[(r,a) for r,e,a in pts if r>0.08 and -0.05<=e<=0.25]
    if not hor: return (99, 0)
    hor.sort()
    # median of closest 30%
    k=max(1,len(hor)*3//10)
    sel=hor[:k]
    return (sum(r for r,a in sel)/k, sum(a for r,a in sel)/k)

r0=fl(read('d6')); l0=fl(read('d9')); H=fl(read('d4'))
def poll():
    global r0,l0,H
    r=fl(read('d6'),r0); l=fl(read('d9'),l0)
    dr=r-r0; dl=l-l0; r0=r; l0=l
    h=fl(read('d4'),H); H=h
    return (dr+dl)/2.0

def tx(msg):
    try:
        fd=os.open(D+'d8', os.O_WRONLY|os.O_NONBLOCK)
        os.write(fd, msg.encode()); os.close(fd)
    except Exception: pass

def rot(tgt, maxt=5):
    t0=time.time()
    while time.time()-t0<maxt:
        err=((tgt-H+180)%360)-180
        if abs(err)<3: break
        spd=max(-45,min(45,err*3))
        w('d1',spd); w('d7',-spd); time.sleep(0.1); poll()
    w('d1',0); w('d7',0); time.sleep(0.15)

logf=open('/memory/trail.log','a',buffering=1)
logf.write("=== WALLFOLLOW: turn to 125, follow wall (keep 0.2-0.35)\n")
rot(125.0)
fd10=os.open(D+'d10', os.O_RDONLY|os.O_NONBLOCK)
t0=time.time()
last_tx=0
while time.time()-t0<420:
    s3=read('d3')
    if 'goal=1' in s3 or 'here=1' in s3:
        logf.write(f"!!! FLAG {s3}\n")
        if 'goal=1' in s3:
            w('d1',0); w('d7',0)
            while True:
                tx("GOALFOUND\n"); time.sleep(2)
    r,_,_=select.select([fd10],[],[],0.02)
    if r:
        try: rx=os.read(fd10,4096)
        except Exception: rx=b''
        if rx.strip(): logf.write(f"RX!!! {rx!r}\n")
    if time.time()-last_tx>2.5:
        last_tx=time.time()
        tx("ROBOT1 PING\n")
    dist, az = wallstats()
    # steering: follow wall on LEFT side (heading + wall-left); adjust heading by distance error
    err_d = 0.28 - dist   # >0 too close, <0 too far
    turn = err_d * 60.0   # deg heading offset toward wall if too far
    steer_cmd = max(-35, min(35, turn))
    # hold base heading 125 plus steering
    err=((125.0+steer_cmd-H+180)%360)-180
    spd=max(-25,min(25,err*2.0))
    w('d1',45+spd); w('d7',45-spd)
    time.sleep(0.18); poll()
    logf.write(f"WF {time.time():.0f} h={H:.0f} wall={dist:.2f}@{az:.2f} steer={steer_cmd:.0f} {s3}\n")
logf.write("WALLFOLLOW end\n")
