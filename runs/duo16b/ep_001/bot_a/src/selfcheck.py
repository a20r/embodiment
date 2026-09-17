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
    r0=r; l0=l
    h=fl(read('d4'),H); H=h

logf=open('/memory/trail.log','a',buffering=1)
for k in range(3):
    w('d1',0); w('d7',0); time.sleep(0.3)
    pts=scan()
    # histogram in (r rounded to 0.02, e rounded to 0.05)
    from collections import Counter
    c=Counter((round(r,2),round(e,2)) for r,e,a in pts)
    top=c.most_common(12)
    logf.write(f"SELF h={H:.0f} top_clusters={top}\n")
    # rotate 120
    t0=time.time()
    while time.time()-t0<3:
        err=(((H+120)-H+180)%360)-180
        if abs(err)<4: break
        spd=max(-45,min(45,err*2.0))
        w('d1',spd); w('d7',-spd); time.sleep(0.1); poll()
    w('d1',0); w('d7',0); time.sleep(0.3)
logf.write("SELFCHECK end\n")
