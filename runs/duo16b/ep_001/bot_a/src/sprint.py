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

def rot(tgt, maxt=6):
    t0=time.time()
    while time.time()-t0<maxt:
        err=((tgt-H+180)%360)-180
        if abs(err)<3: break
        spd=max(-50,min(50,err*3))
        w('d1',spd); w('d7',-spd); time.sleep(0.1); poll()
    w('d1',0); w('d7',0); time.sleep(0.2)

def scan_best():
    """rotate 360 in 24 steps, return (best_world_bearing, best_range)"""
    h0=H; best=(0,h0)
    cur=h0
    for k in range(24):
        rng,az=farstats()
        wb=(cur+az*57.2958)%360
        if rng>best[0]: best=(rng,wb)
        t0=time.time()
        while time.time()-t0<3:
            err=(((cur+15)-H+180)%360)-180
            if abs(err)<4: break
            spd=max(-45,min(45,err*2.0))
            w('d1',spd); w('d7',-spd); time.sleep(0.1); poll()
        w('d1',0); w('d7',0); time.sleep(0.1)
        cur=fl(read('d4'),cur)
    rot(h0,6)
    return best

logf=open('/memory/trail.log','a',buffering=1)
logf.write("=== SPRINT start\n")
while True:
    s3=read('d3')
    if 'goal=1' in s3:
        logf.write(f"!!! GOAL {time.time():.0f} x={X:.0f} y={Y:.0f} {s3}\n")
        w('d1',0); w('d7',0)
        while True:
            broadcast(f"GOALFOUND x={X:.0f} y={Y:.0f}\n"); time.sleep(2)
    rng, wb = scan_best()
    logf.write(f"SCAN {time.time():.0f} best_rng={rng:.3f} at wb={wb:.0f} x={X:.0f} y={Y:.0f}\n")
    if rng < 0.3:
        logf.write("SPRINT: object closer than 0.3 — careful approach\n")
        # slow careful approach
        for k in range(6):
            s3=read('d3')
            if 'goal=1' in s3: break
            r2,wb2 = farstats()
            if r2<0.05: 
                w('d1',0); w('d7',0); time.sleep(1.5); continue
            rot((H+wb2*57.2958)%360)
            t0=time.time()
            while time.time()-t0<0.5:
                w('d1',40); w('d7',40); time.sleep(0.1); poll()
            w('d1',0); w('d7',0); time.sleep(0.1)
        continue
    # sprint toward wb, re-aim every 3s using scan-free dead-reckoning hold
    leg=0
    while leg<5:
        leg+=1
        s3=read('d3')
        if 'goal=1' in s3: break
        rot(wb)  # re-aim to the LOCKED world bearing (not noisy az)
        t0=time.time(); mvd=0
        while time.time()-t0<2.0:
            # heading hold
            err=((wb-H+180)%360)-180
            spd=max(-40,min(40,err*2.0))
            w('d1',85+spd); w('d7',85-spd)
            time.sleep(0.15); mvd+=poll()
        w('d1',0); w('d7',0); time.sleep(0.1)
        cur=farstats()[0]
        logf.write(f"SPRINT leg={leg} mvd={mvd:.0f} cur_rng={cur:.3f} x={X:.0f} y={Y:.0f} h={H:.0f}\n")
        if cur<0.35: break
