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
        time.sleep(0.05)
    vals.sort()
    return vals[0] if vals else 9.9

r0=fl(read('d6')); l0=fl(read('d9')); X=0.0; Y=0.0; H=fl(read('d4'))
def poll():
    global r0,l0,X,Y,H
    r=fl(read('d6'),r0); l=fl(read('d9'),l0)
    dr=r-r0; dl=l-l0; r0=r; l0=l
    h=fl(read('d4'),H); H=h
    fwd=(dr+dl)/2.0; a=math.radians(H)
    X+=fwd*math.cos(a); Y+=fwd*math.sin(a)
    return fwd

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
    w('d1',0); w('d7',0); time.sleep(0.15)

logf=open('/memory/trail.log','a',buffering=1)
logf.write("=== CLOSEIN start\n")

# Phase 1: gradient approach on d11 until < 0.32
best=sample_d11()
logf.write(f"CI start d11={best:.3f}\n")
sign=+1; fail=0
t0=time.time()
while time.time()-t0<420 and best>0.30:
    s3=read('d3')
    if 'goal=1' in s3:
        logf.write(f"!!! GOAL {s3}\n"); w('d1',0); w('d7',0); break
    d0v=sample_d11()
    t1=time.time()
    while time.time()-t1<0.8:
        w('d1',60); w('d7',60); time.sleep(0.12); poll()
    w('d1',0); w('d7',0)
    d1v=sample_d11()
    logf.write(f"CI {time.time():.0f} h={H:.0f} d11 {d0v:.3f}->{d1v:.3f}\n")
    if d1v < best-0.004:
        best=d1v; fail=0; continue
    fail+=1
    if fail>=2: sign=-sign; fail=0
    rot(H+sign*60)

logf.write(f"CI phase1 done d11={sample_d11():.3f}\n")
# Phase 2: TX burst + listen
msgs=[b"PING\n",b"HELLO\n",b"ROBOT1 HERE\n",b"ACK\n",b"GOAL?\n",b"WHERE ARE YOU\n",b"1\n",b"STATUS\n"]
fd10=os.open(D+'d10', os.O_RDONLY|os.O_NONBLOCK)
t0=time.time(); k=0; got=[]
while time.time()-t0<240:
    tx(msgs[k%len(msgs)]); k+=1
    t1=time.time()
    while time.time()-t1<0.5:
        r,_,_=select.select([fd10],[],[],0.05)
        if r:
            try: d=os.read(fd10,4096)
            except Exception: d=b''
            if d.strip(): got.append(d); logf.write(f"RX!!! {d!r}\n")
        d5=read('d5'); d0v=read('d0')
        if d5=='1' or d0v=='1':
            logf.write(f"FLAG d5={d5} d0={d0v} d11={read('d11')}\n")
            got.append(b'FLAG')
    d11=sample_d11(3)
    logf.write(f"CI-TX {time.time():.0f} d11={d11:.3f} msgs={k} rx={len(got)}\n")
    if got: break
logf.write(f"CLOSEIN done rx={got[:3]}\n")
