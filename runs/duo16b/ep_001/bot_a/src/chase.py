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

def farstats():
    s=read('d2'); vals=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            r,e,a=map(float,p.split(','))
            if r>0.3: vals.append((r,a))
        except: pass
    if not vals: return 0,0,0
    mx=max(vals)
    return mx[0], mx[1], len(vals)

r0=fl(read('d6')); l0=fl(read('d9')); X=0.0; Y=0.0; H=fl(read('d4'))
def poll():
    global r0,l0,X,Y,H
    r=fl(read('d6'),r0); l=fl(read('d9'),l0)
    dr=r-r0; dl=l-l0; r0=r; l0=l
    h=fl(read('d4'),H); H=h
    fwd=(dr+dl)/2.0; a=math.radians(H)
    X+=fwd*math.cos(a); Y+=fwd*math.sin(a)
    return fwd

def rot(tgt):
    t0=time.time()
    while time.time()-t0<6:
        err=((tgt-H+180)%360)-180
        if abs(err)<4: break
        spd=max(-50,min(50,err*2.5))
        w('d1',spd); w('d7',-spd); time.sleep(0.1); poll()
    w('d1',0); w('d7',0); time.sleep(0.2)

print("far before:", farstats())
rot(100)
print("at h=100, far:", farstats())
for k in range(6):
    t0=time.time(); mvd=0
    while time.time()-t0<1.5:
        w('d1',90); w('d7',90); time.sleep(0.15); mvd+=poll()
    w('d1',0); w('d7',0); time.sleep(0.2)
    print(f"drove {mvd:.0f} ticks; x={X:.0f} y={Y:.0f} h={H:.0f}; far:", farstats())
