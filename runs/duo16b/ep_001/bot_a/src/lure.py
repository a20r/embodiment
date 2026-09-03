import os,select,time
D='/dev/robot/'
def read(p,timeout=0.1):
    fd=os.open(D+p,os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,9999999).decode().strip()
        except: out=''
    os.close(fd); return out
def tx(m):
    try:
        fd=os.open(D+'d8',os.O_WRONLY|os.O_NONBLOCK); os.write(fd,m.encode()); os.close(fd)
    except Exception: pass
log=open('/memory/lure.log','w',buffering=1)
log.write("=== LURE: hold at beacon, sample d3 fast, TX COME\n")
fd10=os.open(D+'d10',os.O_RDONLY|os.O_NONBLOCK)
g=h=0; n=0
t0=time.time(); lasttx=0; lastd11log=0
while time.time()-t0<150:
    s3=read('d3',0.03)
    n+=1
    if 'goal=1' in s3: g+=1; log.write(f"GOAL1 hit! {s3}\n")
    if 'here=1' in s3: h+=1; log.write(f"HERE1 hit! {s3}\n")
    r,_,_=select.select([fd10],[],[],0.01)
    if r:
        try: rx=os.read(fd10,4096).decode(errors='replace').strip()
        except: rx=''
        if rx: log.write(f"RX!!! {rx!r}\n")
    if time.time()-lasttx>3:
        lasttx=time.time()
        tx("R1 IS AT TALL BEACON. COME TO ME. GOAL MAY NEED BOTH.\n")
    if time.time()-lastd11log>2:
        lastd11log=time.time()
        d11=read('d11',0.1); d5=read('d5',0.1)
        log.write(f"L {time.time()-t0:.0f} d11={d11} d5={d5} d3samples={n} g1={g} h1={h} {s3}\n")
log.write(f"LURE end g1={g} h1={h} n={n}\n")
