import os, select, time, math, json
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

def floor_med():
    s=read('d2'); vals=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            r,e,a=map(float,p.split(','))
            if r>0.02 and -0.1<=e<=0.35: vals.append(r)
        except: pass
    if not vals: return -1
    vals.sort(); return vals[len(vals)//2]

def far_max():
    s=read('d2'); mx=0
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            r,e,a=map(float,p.split(','))
            if r>mx: mx=r
        except: pass
    return mx

r0=fl(read('d6')); l0=fl(read('d9'))
X=0.0; Y=0.0; H=fl(read('d4'))
def poll():
    global r0,l0,X,Y,H
    r=fl(read('d6'),r0); l=fl(read('d9'),l0)
    dr=r-r0; dl=l-l0; r0=r; l0=l
    h=fl(read('d4'),H); H=h
    fwd=(dr+dl)/2.0
    a=math.radians(H)
    X+=fwd*math.cos(a); Y+=fwd*math.sin(a)
    return fwd

def broadcast(msg):
    try:
        fd=os.open(D+'d8', os.O_WRONLY|os.O_NONBLOCK)
        os.write(fd, msg.encode()); os.close(fd)
    except Exception: pass

logf=open('/memory/trail.log','a',buffering=1)
logf.write(f"=== EXPLORE3 start {time.time():.0f} h={H:.0f}\n")

T0=time.time()
TURN0=0.012; MINT=0.0008
last=0.0; stall=0
while True:
    s3=read('d3'); d0=read('d0'); d5=read('d5'); d11=read('d11')
    if 'goal=1' in s3:
        logf.write(f"!!! GOAL {time.time():.0f} x={X:.1f} y={Y:.1f} h={H:.0f} {s3}\n")
        w('d1',0); w('d7',0)
        while True:
            broadcast(f"GOALFOUND x={X:.0f} y={Y:.0f} h={H:.0f}\n")
            time.sleep(2)
    if 'here=1' in s3 or d5=='1' or d0=='1':
        logf.write(f"EVENT {time.time():.0f} x={X:.1f} y={Y:.1f} h={H:.0f} {s3} d0={d0} d5={d5} d11={d11}\n")
    el=time.time()-T0
    turn=max(MINT, TURN0*math.exp(-el/1200.0))
    moved=0.0; t0=time.time()
    while time.time()-t0<1.0:
        w('d1',100); w('d7', max(0, 100-turn*3500))
        time.sleep(0.15)
        moved+=poll()
    w('d1',0); w('d7',0)
    if time.time()-last>5:
        last=time.time()
        logf.write(f"POS {time.time():.0f} x={X:.0f} y={Y:.0f} h={H:.0f} fl={floor_med():.2f} far={far_max():.2f} d11={d11} d5={d5}\n")
    if moved<20:
        stall+=1
        logf.write(f"STALL {stall} moved={moved:.0f} x={X:.0f} y={Y:.0f} d5={d5} d0={d0}\n")
        # gentle backoff (avoid ramming companion)
        t0=time.time()
        while time.time()-t0<0.7:
            w('d1',-50); w('d7',-50); time.sleep(0.15); poll()
        w('d1',0); w('d7',0)
        tgt=H+70+40*stall
        t0=time.time()
        while time.time()-t0<4:
            err=((tgt-H+180)%360)-180
            if abs(err)<5: break
            spd=max(-55,min(55,err*2.5))
            w('d1',spd); w('d7',-spd); time.sleep(0.1); poll()
        w('d1',0); w('d7',0)
        if stall>=5: stall=0
    else:
        stall=0
