import os,select,time,math
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
TGT=106.0
log=open('/memory/drive106.log','w',buffering=1)
log.write("=== DRIVE toward 106, watch d11/far/tall\n")
fd10=os.open(D+'d10',os.O_RDONLY|os.O_NONBLOCK)
t0=time.time()
while time.time()-t0<40:
    H=fl(read('d4'))
    err=((TGT-H+180)%360)-180
    spd=max(-15,min(15,err*1.5))
    w('d1',25+spd); w('d7',25-spd)
    time.sleep(0.15)
    s3=read('d3',0.05); d11=read('d11',0.05)
    r,_,_=select.select([fd10],[],[],0.01)
    if r:
        try: rx=os.read(fd10,4096).decode(errors='replace').strip()
        except: rx=''
        if rx: log.write(f"RX!!! {rx!r}\n")
    pts=[]
    for p in read('d2',0.15).split(';'):
        p=p.strip()
        if not p: continue
        try: pts.append(tuple(map(float,p.split(','))))
        except: pass
    tall=[t for t in pts if t[1]>0.35 and 0.05<t[0]<3]
    far=[t for t in pts if t[0]>0.9 and 0.02<t[0]]
    fl_=sorted(t[0] for t in far)
    log.write(f"D {time.time()-t0:.1f} h={H:.0f} d11={d11} tall={len(tall)} tallr={sorted(t[0] for t in tall)[len(tall)//2] if tall else -1:.2f} far={len(far)} farr={fl_[len(fl_)//2] if fl_ else -1:.2f} {s3}\n")
    if 'goal=1' in s3 or 'here=1' in s3:
        w('d1',0); w('d7',0); log.write(f"FLAG!!! {s3}\n"); break
    d5=read('d5',0.05)
    if d5.strip()=='1':
        w('d1',0); w('d7',0); log.write("CONTACT d5 - stopping\n"); time.sleep(1)
w('d1',0); w('d7',0)
log.write("drive106 end\n")
