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

def farstats():
    s=read('d2'); best=(0,0)
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            r,e,a=map(float,p.split(','))
            if r>best[0]: best=(r,a)
        except: pass
    return best  # (range, az)

r0=fl(read('d6')); l0=fl(read('d9')); X=0.0; Y=0.0; H=fl(read('d4'))
def poll():
    global r0,l0,X,Y,H
    r=fl(read('d6'),r0); l=fl(read('d9'),l0)
    dr=r-r0; dl=l-l0; r0=r; l0=l
    h=fl(read('d4'),H); H=h
    fwd=(dr+dl)/2.0; a=math.radians(H)
    X+=fwd*math.cos(a); Y+=fwd*math.sin(a)
    return fwd

def broadcast(msg):
    try:
        fd=os.open(D+'d8', os.O_WRONLY|os.O_NONBLOCK)
        os.write(fd, msg.encode()); os.close(fd)
    except Exception: pass

logf=open('/memory/trail.log','a',buffering=1)
logf.write("=== CHASE2 start\n")

def rot(delta, maxt=4):
    t0=time.time()
    while time.time()-t0<maxt:
        err=((delta+180)%360)-180
        if abs(err)<3: break
        spd=max(-50,min(50,err*3))
        w('d1',spd); w('d7',-spd); time.sleep(0.1); poll()
    w('d1',0); w('d7',0); time.sleep(0.15)

prev=99; same=0
while True:
    s3=read('d3')
    if 'goal=1' in s3:
        logf.write(f"!!! GOAL {time.time():.0f} {s3}\n"); w('d1',0); w('d7',0)
        while True:
            broadcast("AT GOAL\n"); time.sleep(2)
    rng, az = farstats()
    logf.write(f"CHASE {time.time():.0f} rng={rng:.3f} az={az:.3f} h={H:.0f} {s3}\n")
    if rng < 0.5:
        logf.write("CHASE: object within 0.5, slowing\n")
    if abs(rng-prev)<0.01: same+=1
    else: same=0
    prev=rng
    if same>=4:
        logf.write("CHASE: range stopped shrinking\n")
        break
    # steer toward azimuth of far return, then drive
    rot(az*57.3*0.9)
    t0=time.time(); mvd=0
    while time.time()-t0<1.2:
        w('d1',85); w('d7',85); time.sleep(0.15); mvd+=poll()
        s3=read('d3')
        if 'goal=1' in s3:
            logf.write(f"!!! GOAL mid {s3}\n"); break
    w('d1',0); w('d7',0)
logf.write("CHASE2 ended\n")
