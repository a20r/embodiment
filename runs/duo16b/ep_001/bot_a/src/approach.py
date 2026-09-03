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
    return best

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

def rot(tgt, maxt=5):
    t0=time.time()
    while time.time()-t0<maxt:
        err=((tgt-H+180)%360)-180
        if abs(err)<3: break
        spd=max(-50,min(50,err*3))
        w('d1',spd); w('d7',-spd); time.sleep(0.1); poll()
    w('d1',0); w('d7',0); time.sleep(0.15)

logf=open('/memory/trail.log','a',buffering=1)
logf.write("=== APPROACH start\n")
for leg in range(20):
    s3=read('d3')
    if 'goal=1' in s3:
        logf.write(f"!!! GOAL {time.time():.0f} x={X:.0f} y={Y:.0f} {s3}\n")
        w('d1',0); w('d7',0)
        while True:
            broadcast(f"GOALFOUND x={X:.0f} y={Y:.0f}\n"); time.sleep(2)
    rng, az = farstats()
    wb = (H + az*57.2958) % 360   # world bearing of far object
    logf.write(f"APP {time.time():.0f} leg={leg} rng={rng:.3f} az={az:.3f} wb={wb:.0f} h={H:.0f} {s3}\n")
    if rng < 0.12:
        logf.write("APP: very close, stopping to check\n")
        w('d1',0); w('d7',0); time.sleep(2)
        continue
    rot(wb)
    t0=time.time(); mvd=0
    while time.time()-t0<1.3:
        w('d1',80); w('d7',80); time.sleep(0.15); mvd+=poll()
    w('d1',0); w('d7',0); time.sleep(0.1)
logf.write("APPROACH ended\n")
