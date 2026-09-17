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
            x,z,y=map(float,p.split(',')); pts.append((x,z))
        except: pass
    return pts

def prof():
    pts=scan()
    out={}
    for lo,hi in [(0.3,0.6),(0.6,1.0),(1.0,1.5),(1.5,2.2)]:
        sel=[z for x,z in pts if lo<=x<hi]
        if sel:
            sel.sort(); out[round(lo,1)]=round(sel[len(sel)//2],2)
    zs=[z for x,z in pts if x>0.25]
    out['zmin']=round(min(zs),2) if zs else None
    out['zmax']=round(max(zs),2) if zs else None
    return out

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
    w('d1',0); w('d7',0); time.sleep(0.2)

logf=open('/memory/trail.log','a',buffering=1)
logf.write("=== GODOWN2: descend ramp, follow downhill\n")
TARGET=300.0
stall=0
t0=time.time()
while time.time()-t0<600:
    s3=read('d3')
    if 'goal=1' in s3 or 'here=1' in s3:
        logf.write(f"!!! FLAG {s3}\n")
        if 'goal=1' in s3:
            w('d1',0); w('d7',0)
            while True:
                tx("GOALFOUND\n"); time.sleep(2)
    p=prof()
    logf.write(f"GD {time.time():.0f} h={H:.0f} {p} d11={read('d11')} {s3}\n")
    t1=time.time(); mvd=0.0
    while time.time()-t1<2.0:
        err=((TARGET-H+180)%360)-180
        spd=max(-30,min(30,err*2.0))
        w('d1',55+spd); w('d7',55-spd)
        time.sleep(0.18); mvd+=poll()
    w('d1',0); w('d7',0); time.sleep(0.1)
    if mvd<40:
        stall+=1
        logf.write(f"GD STALL {stall} mvd={mvd:.0f}\n")
        t2=time.time()
        while time.time()-t2<0.7:
            w('d1',-50); w('d7',-50); time.sleep(0.15); poll()
        w('d1',0); w('d7',0)
        if stall>=2:
            # re-scan: pick heading with most continuous downward profile among 260..360
            best=None
            for cand in range(260, 370, 15):
                rot(cand, 3)
                pp=prof()
                zmid = pp.get(1.0, 0)
                score = -zmid  # lower far-ground = more downhill ahead
                if best is None or score>best[0]: best=(score, cand, pp)
            logf.write(f"GD rescan best={best}\n")
            TARGET=best[1]; stall=0
    else:
        stall=0
logf.write("GODOWN2 end\n")
