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
logf.write("=== SLOWCRAWL start\n")
# slow lawnmower: leg E/W, stop 1.5s every 1.5s of crawl, offset N between legs
legdir = 0  # 0=east(90), 1=west(270)
import random
def rot(tgt, maxt=5):
    t0=time.time()
    while time.time()-t0<maxt:
        err=((tgt-H+180)%360)-180
        if abs(err)<3: break
        spd=max(-45,min(45,err*3))
        w('d1',spd); w('d7',-spd); time.sleep(0.1); poll()
    w('d1',0); w('d7',0); time.sleep(0.15)

T0=time.time()
while time.time()-T0<900:
    s3=read('d3'); d11=read('d11'); d0=read('d0'); d5=read('d5')
    logf.write(f"SC {time.time():.0f} x={X:.0f} y={Y:.0f} h={H:.0f} {s3} d11={d11} d0={d0} d5={d5}\n")
    if 'goal=1' in s3:
        logf.write(f"!!! GOAL {s3}\n")
        w('d1',0); w('d7',0)
        while True:
            broadcast(f"GOALFOUND x={X:.0f} y={Y:.0f}\n"); time.sleep(2)
    # crawl 1.5s slow
    tgt = 90 if legdir==0 else 270
    t0=time.time()
    while time.time()-t0<1.5:
        err=((tgt-H+180)%360)-180
        spd=max(-25,min(25,err*2.0))
        w('d1',22+spd); w('d7',22-spd)
        time.sleep(0.15); poll()
    w('d1',0); w('d7',0)
    # stop 1.5s
    time.sleep(1.5)
    # turn to next leg: alternate E/W, after each pair offset south by crawling 90deg
    if random.random()<0.5:
        legdir = 1-legdir
        rot(H+ (28 if random.random()<0.5 else -28))
    else:
        rot(tgt)
logf.write("SLOWCRAWL end\n")
