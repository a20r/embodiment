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
def fl(x,d=0.0):
    try: return float(x)
    except: return d
TGT=200.0
log=open('/memory/chase200.log','w',buffering=1)
log.write("=== CHASE companion toward 200\n")
fd10=os.open(D+'d10',os.O_RDONLY|os.O_NONBLOCK)
t0=time.time(); lastd=None
while time.time()-t0<45:
    H=fl(read('d4'))
    err=((TGT-H+180)%360)-180
    spd=max(-12,min(12,err*1.5))
    d11=fl(read('d11',0.08),-1)
    # slow down when close
    v=25 if (lastd is None or d11<0 or d11>0.2) else 12
    w('d1',v+spd); w('d7',v-spd)
    time.sleep(0.15)
    lastd=d11
    s3=read('d3',0.05)
    r,_,_=select.select([fd10],[],[],0.01)
    if r:
        try: rx=os.read(fd10,4096).decode(errors='replace').strip()
        except: rx=''
        if rx: log.write(f"RX!!! {rx!r}\n")
    # deep check ahead
    pts=[]
    for p in read('d2',0.12).split(';'):
        p=p.strip()
        if not p: continue
        try: pts.append(tuple(map(float,p.split(','))))
        except: pass
    deep=[t[0] for t in pts if t[1]<-0.9 and abs(t[2])<0.15 and t[0]>0.05]
    deepmed=sorted(deep)[len(deep)//2] if deep else -1
    d5=read('d5',0.05)
    log.write(f"C {time.time()-t0:.1f} h={H:.0f} d11={d11} deepmed={deepmed} d5={d5} {s3}\n")
    if 'goal=1' in s3 or 'here=1' in s3:
        w('d1',0); w('d7',0); log.write(f"FLAG!!! {s3}\n"); break
    if d5.strip()=='1':
        w('d1',0); w('d7',0)
        log.write("CONTACT - stop\n")
        time.sleep(2)
w('d1',0); w('d7',0)
log.write("chase200 end\n")
