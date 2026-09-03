import os,select,time
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
def tx(m):
    try:
        fd=os.open(D+'d8',os.O_WRONLY|os.O_NONBLOCK); os.write(fd,m.encode()); os.close(fd)
    except Exception: pass
def fl(x,d=0.0):
    try: return float(x)
    except: return d
w('d1',0); w('d7',0)
log=open('/memory/touch.log','w',buffering=1)
log.write("=== TOUCH: stopped next to companion, radio burst, lidar look\n")
fd10=os.open(D+'d10',os.O_RDONLY|os.O_NONBLOCK)
msgs=[b"ROBOT1 TOUCHING YOU. HELLO!\n",b"ACK PLEASE. WHERE IS GOAL?\n",b"R1 HERE. GOALFOUND?\n",b"HELLO ROBOT2\n"]
t0=time.time(); lasttx=0; k=0; lastscan=0
while time.time()-t0<70:
    s3=read('d3',0.05)
    r,_,_=select.select([fd10],[],[],0.03)
    if r:
        try: rx=os.read(fd10,4096).decode(errors='replace').strip()
        except: rx=''
        if rx: log.write(f"RX!!! {rx!r}\n")
    if time.time()-lasttx>2.5:
        lasttx=time.time(); tx(msgs[k%len(msgs)]); k+=1
    if time.time()-lastscan>6:
        lastscan=time.time()
        d11=read('d11',0.1); d5=read('d5',0.1); H=fl(read('d4'))
        pts=[]
        for p in read('d2',0.2).split(';'):
            p=p.strip()
            if not p: continue
            try: pts.append(tuple(map(float,p.split(','))))
            except: pass
        near=[(r_,e,a) for r_,e,a in pts if 0.05<=r_<=0.6 and e>-0.3]
        import statistics
        bins={}
        for rr,ee,aa in near:
            b=round(aa*5)/5
            bins.setdefault(b,[]).append(rr)
        summary=" ".join(f"{b:+.1f}:{statistics.median(v):.2f}({len(v)})" for b,v in sorted(bins.items()) if len(v)>=4)
        log.write(f"T {time.time()-t0:.0f} h={H:.0f} d11={d11} d5={d5} | {summary} {s3}\n")
    if 'goal=1' in s3 or 'here=1' in s3:
        log.write(f"FLAG!!! {s3}\n"); break
log.write("touch end\n")
