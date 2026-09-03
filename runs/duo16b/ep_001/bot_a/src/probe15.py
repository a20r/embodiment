import os, select, time

D='/dev/robot/'
def read(p, timeout=0.25):
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

def lmin():
    s=read('d2'); vals=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            v=float(p.split(',')[0])
            if v>0: vals.append(v)
        except: pass
    return min(vals) if vals else 99

# drive forward slowly toward whatever is ahead; log d5/d0 vs lidar min
log=[]
t0=time.time()
while time.time()-t0<6:
    w('d1',40); w('d7',40); time.sleep(0.25)
    log.append((lmin(), read('d5'), read('d0'), read('d4')))
    if lmin() < 0.1 and read('d5')=='1': 
        log.append(("STOP cond",)); break
w('d1',0); w('d7',0)
for l in log: print(l)
