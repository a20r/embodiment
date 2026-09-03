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

def deep():
    s=read('d2'); best=0; n=0
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            r,e,a=map(float,p.split(','))
            if r>0.05 and e<-0.9:
                n+=1
                if r>best: best=r
        except: pass
    return best, n

r0=fl(read('d6')); l0=fl(read('d9')); H=fl(read('d4'))
def poll():
    global r0,l0,H
    r=fl(read('d6'),r0); l=fl(read('d9'),l0)
    r0=r; l0=l
    h=fl(read('d4'),H); H=h

logf=open('/memory/trail.log','a',buffering=1)
res=[]
cur=H
for k in range(24):
    b,n = deep()
    res.append((round(cur,0), round(b,2), n))
    t0=time.time()
    while time.time()-t0<2.5:
        err=(((cur+15)-H+180)%360)-180
        if abs(err)<4: break
        spd=max(-45,min(45,err*2.0))
        w('d1',spd); w('d7',-spd); time.sleep(0.1); poll()
    w('d1',0); w('d7',0); time.sleep(0.1)
    cur=fl(read('d4'),cur)
for r in res: logf.write(f"CLIFFMAP h={r[0]:.0f} deeprange={r[1]} n={r[2]}\n")
print("done")
