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

def deep_of(pts):
    best=(0,0)
    for r,e,a in pts:
        if r>0.05 and e<-0.9 and r>best[0]: best=(r,a)
    return best

def comp_of(pts):
    # returns at horizon, plausible companion range
    hits=[(r,a) for r,e,a in pts if r>0.35 and r<0.85 and -0.05<=e<=0.25]
    if not hits: return (0,0,0)
    mx=max(hits)
    return (mx[0], mx[1], len(hits))

r0=fl(read('d6')); l0=fl(read('d9')); H=fl(read('d4'))
def poll():
    global r0,l0,H
    r=fl(read('d6'),r0); l=fl(read('d9'),l0)
    r0=r; l0=l
    h=fl(read('d4'),H); H=h

logf=open('/memory/trail.log','a',buffering=1)
logf.write("=== SWEEP2\n")
cur=H
out=[]
for k in range(24):
    pts=scan()
    dm,da = deep_of(pts)
    cr,ca,cn = comp_of(pts)
    out.append((round(cur,0), round(dm,2), round(da,2), round(cr,2), round(ca,2), cn))
    t0=time.time()
    while time.time()-t0<2.5:
        err=(((cur+15)-H+180)%360)-180
        if abs(err)<4: break
        spd=max(-45,min(45,err*2.0))
        w('d1',spd); w('d7',-spd); time.sleep(0.1); poll()
    w('d1',0); w('d7',0); time.sleep(0.1)
    cur=fl(read('d4'),cur)
for o in out:
    logf.write(f"SW2 h={o[0]:.0f} deep={o[1]}@az{o[2]} comp={o[3]}@az{o[4]} n={o[5]}\n")
best_deep=max(out, key=lambda o:o[1])
best_comp=max(out, key=lambda o:o[5])
logf.write(f"SW2 BEST deep h={best_deep[0]} r={best_deep[1]} | comp h={best_comp[0]} n={best_comp[5]}\n")
print("done")
