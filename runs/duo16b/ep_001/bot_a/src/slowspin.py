import os,select,time,statistics
D='/dev/robot/'
def read(p,timeout=0.15):
    fd=os.open(D+p,os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,9999999).decode().strip()
        except: out=''
    os.close(fd); return out
def w(p,v):
    try:
        fd=os.open(D+p,os.O_WRONLY|os.O_NONBLOCK); os.write(fd,f"{v}\n".encode()); os.close(fd)
    except Exception: pass
def fl(x,d=0.0):
    try: return float(x)
    except: return d
log=open('/memory/slowspin.log','w',buffering=1)
log.write("=== SLOWSPIN: 6 deg/s rotate, log clusters+d11 every 1.2s\n")
fd10=os.open(D+'d10',os.O_RDONLY|os.O_NONBLOCK)
t0=time.time(); slogtime=0
w('d1',4); w('d7',-4)
while time.time()-t0<90:
    s3=read('d3',0.05)
    r,_,_=select.select([fd10],[],[],0.02)
    if r:
        try: rx=os.read(fd10,4096).decode(errors='replace').strip()
        except: rx=''
        if rx: log.write(f"RX!!! {rx!r}\n")
    if time.time()-slogtime>1.2:
        slogtime=time.time()
        H=fl(read('d4')); d11=read('d11',0.1)
        pts=[]
        for p in read('d2',0.2).split(';'):
            p=p.strip()
            if not p: continue
            try: pts.append(tuple(map(float,p.split(','))))
            except: pass
        cl=[(r,e,a) for r,e,a in pts if 0.1<=r<=1.0 and e>-0.3]
        bins={}
        for rr,ee,aa in cl:
            b=round(aa*5)/5
            bins.setdefault(b,[]).append(rr)
        summary=" ".join(f"{b:+.1f}:{statistics.median(v):.2f}({len(v)})" for b,v in sorted(bins.items()) if len(v)>=5)
        log.write(f"S {time.time()-t0:.0f} h={H:.0f} d11={d11} | {summary} {s3}\n")
    if 'goal=1' in s3 or 'here=1' in s3:
        w('d1',0); w('d7',0); log.write(f"FLAG!!! {s3}\n"); break
w('d1',0); w('d7',0)
log.write("slowspin end\n")
