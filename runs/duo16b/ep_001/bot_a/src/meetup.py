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
    hits=[(r,a) for r,e,a in pts if r>0.3 and r<0.95 and -0.05<=e<=0.25]
    if not hits: return (0,0,0)
    hits.sort()
    # use the closest big cluster: take median range of hits
    rr=sorted(h[0] for h in hits)
    return (rr[len(rr)//2], hits[len(hits)//2][1], len(hits))

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
logf.write("=== MEETUP start\n")
fd10=os.open(D+'d10', os.O_RDONLY|os.O_NONBLOCK)
msgs=[b"PING ROBOT1\n",b"HELLO\n",b"ROBOT1 HERE\n",b"ACK?\n"]
k=0
t0=time.time()
while time.time()-t0<420:
    s3=read('d3')
    if 'goal=1' in s3 or 'here=1' in s3:
        logf.write(f"!!! FLAG {s3}\n")
        if 'goal=1' in s3:
            w('d1',0); w('d7',0)
            while True:
                tx("GOALFOUND\n"); time.sleep(2)
    pts=scan()
    cr,ca,cn = comp_of(pts)
    # listen
    r,_,_=select.select([fd10],[],[],0.02)
    rx=b''
    if r:
        try: rx=os.read(fd10,4096)
        except Exception: rx=b''
    if rx.strip(): logf.write(f"RX!!! {rx!r}\n")
    if cr>0.25 and cn>5:
        # steer toward companion bearing
        wb=(H+ca*57.2958)%360
        err=((wb-H+180)%360)-180
        spd=max(-30,min(30,err*2.0))
        spd_f = 45 if cr>0.45 else 18
        w('d1',spd_f+spd); w('d7',spd_f-spd)
        time.sleep(0.2); poll()
        logf.write(f"MU {time.time():.0f} comp r={cr:.2f}@az{ca:.2f} n={cn} h={H:.0f} {s3}\n")
    else:
        # lost it: rotate to search
        w('d1',30); w('d7',-30); time.sleep(0.3); poll()
        w('d1',0); w('d7',0)
        logf.write(f"MU search h={H:.0f}\n")
    # tx every 2s
    if int(time.time()*1)%2==0 and int(time.time()*1)!=getattr(meetup,'_lp',-1):
        meetup._lp=int(time.time()*1)
        tx(msgs[k%len(msgs)]); k+=1
logf.write("MEETUP end\n")
