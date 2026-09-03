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

def sample_d11(n=5):
    vals=[]
    for i in range(n):
        v=fl(read('d11'), 9.9)
        if v<9: vals.append(v)
        time.sleep(0.06)
    if not vals: return 9.9
    vals.sort()
    return vals[0]  # min = closest

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
logf.write("=== HOTCOLD start\n")

def drive_for(dur, spd=70):
    t0=time.time(); mvd=0.0
    while time.time()-t0<dur:
        w('d1',spd); w('d7',spd); time.sleep(0.12); mvd+=poll()
    w('d1',0); w('d7',0); time.sleep(0.2)
    return mvd

cur = H
best = sample_d11()
logf.write(f"HC start d11min={best:.3f} h={H:.0f}\n")
step=1.1
sign=+1
fail=0
t0=time.time()
while time.time()-t0<1500:
    s3=read('d3')
    if 'goal=1' in s3:
        logf.write(f"!!! GOAL {time.time():.0f} x={X:.0f} y={Y:.0f} h={H:.0f} {s3} d11={sample_d11()}\n")
        w('d1',0); w('d7',0)
        while True:
            broadcast(f"GOALFOUND x={X:.0f} y={Y:.0f}\n"); time.sleep(2)
    # probe current heading
    d_before = sample_d11()
    mvd = drive_for(0.9)
    d_after = sample_d11()
    logf.write(f"HC {time.time():.0f} h={H:.0f} x={X:.0f} y={Y:.0f} d11 {d_before:.3f}->{d_after:.3f} mvd={mvd:.0f}\n")
    if d_after < best - 0.005:
        best = d_after
        fail = 0
        if best < 0.10:
            drive_for(0.4, 40)  # creep
        continue
    else:
        fail += 1
        if fail>=2:
            sign = -sign
            fail = 0
        # turn and retry
        cur = H + sign*55
        rot(cur)
        if fail==0:
            pass
logf.write("HOTCOLD timeout\n")
