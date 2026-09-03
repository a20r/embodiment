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
    s=read('d2'); med=None; mx=0; downs=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            r,e,a=map(float,p.split(','))
            if r>0.02:
                if mx<r: mx=r
                if 0.3<e<1.6: downs.append(r)
        except: pass
    downs.sort()
    medd = downs[len(downs)//2] if downs else 0
    return mx, medd

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
logf.write("=== DESCENT2 W\n")
TARGET=272.0
stall=0
t0=time.time()
while time.time()-t0<780:
    s3=read('d3')
    mx, medd = lstats()
    d11=read('d11')
    logf.write(f"D2 {time.time():.0f} x={X:.0f} y={Y:.0f} h={H:.0f} max={mx:.2f} downmed={medd:.2f} d11={d11} {s3}\n")
    if 'goal=1' in s3 or 'here=1' in s3:
        logf.write(f"!!! FLAG {s3}\n")
        if 'goal=1' in s3:
            w('d1',0); w('d7',0)
            while True:
                broadcast(f"GOALFOUND x={X:.0f} y={Y:.0f}\n"); time.sleep(2)
    t1=time.time(); mvd=0.0
    while time.time()-t1<3.0:
        err=((TARGET-H+180)%360)-180
        spd=max(-40,min(40,err*2.0))
        w('d1',90+spd); w('d7',90-spd)
        time.sleep(0.15); mvd+=poll()
    w('d1',0); w('d7',0); time.sleep(0.1)
    if mvd<80:
        stall+=1
        logf.write(f"D2 STALL {stall} mvd={mvd:.0f}\n")
        t2=time.time()
        while time.time()-t2<0.7:
            w('d1',-55); w('d7',-55); time.sleep(0.15); poll()
        w('d1',0); w('d7',0)
        if stall>=2:
            TARGET = (TARGET + (35*stall)) % 360
            logf.write(f"D2 target->{TARGET}\n")
            stall=0
    else:
        stall=0
logf.write("D2 end\n")
