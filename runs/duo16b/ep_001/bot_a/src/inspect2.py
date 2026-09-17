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

def comp_of(pts):
    hits=[(r,a) for r,e,a in pts if r>0.25 and r<1.1 and -0.05<=e<=0.25]
    if len(hits)<4: return None
    hits.sort()
    rr=[h[0] for h in hits]
    med=rr[len(rr)//2]
    sel=[a for (r,a) in hits if abs(r-med)<0.18]
    az=sum(sel)/len(sel)
    return (med, az, len(hits))

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

logf=open('/memory/trail.log','a',buffering=1)
logf.write("=== INSPECT2 approach object via cluster tracking\n")
fd10=os.open(D+'d10', os.O_RDONLY|os.O_NONBLOCK)
phase='approach'
lost=0
t0=time.time()
while time.time()-t0<420:
    s3=read('d3'); d11=read('d11'); d5=read('d5')
    r,_,_=select.select([fd10],[],[],0.02)
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
    if phase=='approach':
        c=comp_of(scan())
        if c:
            lost=0
            cr,ca,cn=c
            logf.write(f"I2 {time.time():.0f} obj r={cr:.2f}@az{ca:.2f} n={cn} h={H:.0f} d11={d11} {s3}\n")
            if cr<0.22:
                logf.write("I2: object within 0.22 - STOP, inspect\n")
                phase='inspect'; w('d1',0); w('d7',0); continue
            wb=(H+ca*57.2958)%360
            err=((wb-H+180)%360)-180
            spd=max(-30,min(30,err*2.5))
            spd_f = 50 if cr>0.4 else 22
            w('d1',spd_f+spd); w('d7',spd_f-spd)
            time.sleep(0.2); poll()
        else:
            lost+=1
            w('d1',0); w('d7',0)
            if lost>3:
                logf.write(f"I2 lost object, rotating h={H:.0f}\n")
                w('d1',35); w('d7',-35); time.sleep(0.5); poll()
                w('d1',0); w('d7',0)
            time.sleep(0.3)
    elif phase=='inspect':
        tx("ROBOT1 HERE. ACK\n")
        pts=scan()
        c=comp_of(pts)
        logf.write(f"I2S {time.time():.0f} obj={c} d11={d11} d5={d5} {s3}\n")
        time.sleep(0.8)
logf.write("INSPECT2 end\n")
