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

def lstats():
    s=read('d2'); med=None; mx=0; steeps=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            r,e,a=map(float,p.split(','))
            if r>0.02:
                if mx<r: mx=r
                if e<-0.5 and r>0.05: steeps.append(r)
        except: pass
    return mx, (sum(steeps)/len(steeps) if steeps else 0), len(steeps)

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

logf=open('/memory/trail.log','a',buffering=1)
logf.write("=== DESCEND start\n")
TARGET=90.0
stall=0
t_start=time.time()
while time.time()-t_start < 2400:
    s3=read('d3')
    if 'goal=1' in s3 or 'here=1' in s3:
        logf.write(f"!!! FLAG {time.time():.0f} x={X:.0f} y={Y:.0f} h={H:.0f} {s3} d11={read('d11')}\n")
        if 'goal=1' in s3:
            w('d1',0); w('d7',0)
            while True:
                broadcast(f"GOALFOUND x={X:.0f} y={Y:.0f}\n"); time.sleep(2)
    mx, sm, sn = lstats()
    d11=read('d11')
    logf.write(f"DES {time.time():.0f} x={X:.0f} y={Y:.0f} h={H:.0f} max={mx:.2f} steepmean={sm:.2f} d11={d11}\n")
    # heading hold on TARGET
    t0=time.time(); mvd=0.0
    while time.time()-t0<2.5:
        err=((TARGET-H+180)%360)-180
        spd=max(-40,min(40,err*2.0))
        w('d1',85+spd); w('d7',85-spd)
        time.sleep(0.15); mvd+=poll()
    w('d1',0); w('d7',0); time.sleep(0.1)
    if mvd<50:
        stall+=1
        logf.write(f"DES STALL {stall} mvd={mvd:.0f}\n")
        if stall>=3:
            # try neighbor headings
            TARGET = (TARGET + (30*stall if stall%2 else -30*stall)) % 360
            logf.write(f"DES new target {TARGET}\n")
            stall=0
        t0=time.time()
        while time.time()-t0<0.6:
            w('d1',-50); w('d7',-50); time.sleep(0.15); poll()
        w('d1',0); w('d7',0)
    else:
        stall=0
logf.write("DESCEND timeout\n")
