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
            x,z,y=map(float,p.split(',')); pts.append((x,z))
        except: pass
    return pts

r0=fl(read('d6')); l0=fl(read('d9')); H=fl(read('d4'))
def poll():
    global r0,l0,H
    r=fl(read('d6'),r0); l=fl(read('d9'),l0)
    r0=r; l0=l
    h=fl(read('d4'),H); H=h

logf=open('/memory/trail.log','a',buffering=1)
logf.write("=== TERRAMAP: profile per heading (x-bin: median z)\n")
XBINS=[0.4,0.8,1.2,1.6,2.0,2.4]
cur=H
rows=[]
for k in range(18):
    pts=scan()
    prof={}
    for lo,hi in zip(XBINS, XBINS[1:]+[3.5]):
        sel=[z for x,z in pts if lo<=x<hi and x>0.25]
        if sel:
            sel.sort(); prof[round(lo,1)]=round(sel[len(sel)//2],2)
    # also max z (tall stuff) and min z (deep ground)
    zs=[z for x,z in pts if x>0.25]
    prof['zmax']=round(max(zs),2) if zs else None
    prof['zmin']=round(min(zs),2) if zs else None
    rows.append((round(cur,0), prof))
    t0=time.time()
    while time.time()-t0<2.2:
        err=(((cur+20)-H+180)%360)-180
        if abs(err)<4: break
        spd=max(-45,min(45,err*2.0))
        w('d1',spd); w('d7',-spd); time.sleep(0.1); poll()
    w('d1',0); w('d7',0); time.sleep(0.1)
    cur=fl(read('d4'),cur)
for hval,prof in rows:
    logf.write(f"TM h={hval:.0f} {prof}\n")
logf.write("TERRAMAP end\n")
print("done")
