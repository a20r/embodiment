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
            r,e,a=map(float,p.split(',')); pts.append((r,e,a))
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
    w('d1',0); w('d7',0); time.sleep(0.15)

logf=open('/memory/trail.log','a',buffering=1)
logf.write("=== INSPECT: approach fixed object at wb=316\n")
fd10=os.open(D+'d10', os.O_RDONLY|os.O_NONBLOCK)
TARGET=316.0
phase='approach'
t0=time.time()
while time.time()-t0<300:
    s3=read('d3'); d11=read('d11'); d5=read('d5'); d0v=read('d0')
    r,_,_=select.select([fd10],[],[],0.02)
    if r:
        try: rx=os.read(fd10,4096)
        except Exception: rx=b''
        if rx.strip(): logf.write(f"RX!!! {rx!r}\n")
    if 'goal=1' in s3 or 'here=1' in s3:
        logf.write(f"!!! FLAG {s3} d11={d11} d5={d5}\n")
        if 'goal=1' in s3:
            w('d1',0); w('d7',0)
            while True:
                tx("GOALFOUND\n"); time.sleep(2)
    if phase=='approach':
        pts=scan()
        hor=[(r,a) for r,e,a in pts if r>0.1 and -0.05<=e<=0.25]
        hor.sort()
        near = hor[len(hor)//2] if len(hor)>3 else (9,0)
        logf.write(f"IN {time.time():.0f} h={H:.0f} nearhor={near[0]:.2f}@{near[1]:.2f} d11={d11} d5={d5} {s3}\n")
        # heading hold toward target
        err=((TARGET-H+180)%360)-180
        spd=max(-30,min(30,err*2.0))
        spd_f = 55 if near[0]>0.25 else 15
        w('d1',spd_f+spd); w('d7',spd_f-spd)
        time.sleep(0.2); mvd=poll()
        w('d1',0); w('d7',0)
        if near[0]<0.18 or (d11 and fl(d11,9)<0.2) or mvd<5:
            logf.write("IN: ARRIVED at object vicinity. Stopping.\n")
            phase='inspect'
            w('d1',0); w('d7',0)
            tx("ROBOT1 ARRIVED. HELLO?\n")
    elif phase=='inspect':
        # burst TX + listen + full scan dump
        tx("ROBOT1 HERE. ACK\n")
        pts=scan()
        hor=sorted([(r,a) for r,e,a in pts if r>0.05 and -0.1<=e<=0.3])
        logf.write(f"INS {time.time():.0f} d11={d11} d5={d5} d0={d0v} nhor={len(hor)} minhor={hor[0] if hor else '-'} {s3}\n")
        time.sleep(1.0)
logf.write("INSPECT end\n")
