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

def lipz():
    pts=scan()
    sel=[z for x,z in pts if 0.2<=x<0.45]
    if not sel: return None
    sel.sort(); return sel[len(sel)//2]

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
logf.write("=== DESCEND3: find gentlest lip, traverse+descend\n")
# Step 1: back away from lip
t0=time.time()
while time.time()-t0<2.0:
    w('d1',-50); w('d7',-50); time.sleep(0.2); poll()
w('d1',0); w('d7',0)
# Step 2: probe lips at headings 250..350, find max lipz (least negative = gentlest)
best=None
for cand in range(250, 360, 12):
    rot(cand, 3)
    lz=lipz()
    logf.write(f"D3 cand h={cand} lipz={lz}\n")
    if lz is not None and (best is None or lz>best[0]):
        best=(lz,cand)
logf.write(f"D3 best lip={best}\n")
TARGET=best[1] if best else 290.0
rot(TARGET,4)
# Step 3: descend attempts with side-slip: alternate heading TARGET±20 while creeping
t0=time.time()
phase=0
while time.time()-t0<420:
    s3=read('d3')
    if 'goal=1' in s3 or 'here=1' in s3:
        logf.write(f"!!! FLAG {s3}\n")
        if 'goal=1' in s3:
            w('d1',0); w('d7',0)
            while True:
                tx("GOALFOUND\n"); time.sleep(2)
    lz=lipz()
    logf.write(f"D3 h={H:.0f} lipz={lz} d11={read('d11')} {s3}\n")
    # creep forward
    t1=time.time(); mvd=0.0
    while time.time()-t1<1.5:
        err=((TARGET-H+180)%360)-180
        spd=max(-25,min(25,err*2.0))
        w('d1',45+spd); w('d7',45-spd)
        time.sleep(0.18); mvd+=poll()
    w('d1',0); w('d7',0); time.sleep(0.1)
    if mvd<35:
        # stuck: alternate target +-25 to traverse
        phase+=1
        TARGET=(TARGET + (25 if phase%2 else -25))%360
        logf.write(f"D3 stuck, traverse TARGET->{TARGET}\n")
        rot(TARGET,3)
logf.write("DESCEND3 end\n")
