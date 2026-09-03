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
            x,z,y=map(float,p.split(',')); pts.append((x,z,y))
        except: pass
    return pts

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

def rot(tgt, maxt=4):
    t0=time.time()
    while time.time()-t0<maxt:
        err=((tgt-H+180)%360)-180
        if abs(err)<3: break
        spd=max(-45,min(45,err*3))
        w('d1',spd); w('d7',-spd); time.sleep(0.1); poll()
    w('d1',0); w('d7',0); time.sleep(0.2)

logf=open('/memory/trail.log','a',buffering=1)
logf.write("=== REACH: approach tall object (other robot?) at h~270\n")
fd10=os.open(D+'d10', os.O_RDONLY|os.O_NONBLOCK)
TARGET=271.0
rot(TARGET,4)
t0=time.time()
while time.time()-t0<240:
    s3=read('d3'); d11=read('d11')
    r,_,_=select.select([fd10],[],[],0.03)
    if r:
        try: rx=os.read(fd10,4096)
        except Exception: rx=b''
        if rx.strip(): logf.write(f"RX!!! {rx!r}\n")
    if 'goal=1' in s3 or 'here=1' in s3:
        logf.write(f"!!! FLAG {s3} d11={d11}\n")
        if 'goal=1' in s3:
            w('d1',0); w('d7',0)
            while True:
                tx("GOALFOUND\n"); time.sleep(2)
    # object signature: median x of points with z>0.5, x<1.0
    pts=scan()
    tall=[x for x,z,y in pts if z>0.5 and 0.05<x<1.2]
    tall.sort()
    med = tall[len(tall)//2] if tall else None
    logf.write(f"RC {time.time():.0f} h={H:.0f} objX={med} d11={d11} {s3}\n")
    if med is None or med>1.0:
        # lost it, sweep
        rot(H+15,2)
        continue
    if med>0.18:
        err=((TARGET-H+180)%360)-180
        spd=max(-20,min(20,err*2.0))
        spd_f=18 if med<0.35 else 40
        w('d1',spd_f+spd); w('d7',spd_f-spd)
        time.sleep(0.2); poll()
        w('d1',0); w('d7',0)
    else:
        # within 0.18: STOP, hold, TX burst, watch
        w('d1',0); w('d7',0)
        tx("ROBOT1 NEXT TO YOU. ACK\n")
        time.sleep(0.8)
logf.write("REACH end\n")
