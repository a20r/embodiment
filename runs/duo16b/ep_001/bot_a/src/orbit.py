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

def comp_of(pts):
    hits=[(r,a) for r,e,a in pts if r>0.3 and r<1.0 and -0.05<=e<=0.25]
    if len(hits)<4: return None
    hits.sort()
    rr=[h[0] for h in hits]
    aa=[h[1] for h in hits]
    med=rr[len(rr)//2]
    # azimuths of hits near median range
    sel=[a for (r,a) in hits if abs(r-med)<0.15]
    az=sum(sel)/len(sel)
    return (med, az, len(hits))

r0=fl(read('d6')); l0=fl(read('d9')); H=fl(read('d4'))
def poll():
    global r0,l0,H
    r=fl(read('d6'),r0); l=fl(read('d9'),l0)
    r0=r; l0=l
    h=fl(read('d4'),H); H=h

logf=open('/memory/trail.log','a',buffering=1)
logf.write("=== ORBIT: standing still, tracking companion\n")
w('d1',0); w('d7',0)
samples=[]
t0=time.time()
while time.time()-t0<150:
    s3=read('d3')
    if 'goal=1' in s3 or 'here=1' in s3:
        logf.write(f"!!! FLAG {s3}\n")
    c=comp_of(scan())
    if c:
        r_,az_,n_=c
        b=math.radians(H+az_*57.2958)
        samples.append((r_*math.cos(b), r_*math.sin(b), H+az_*57.2958, r_, time.time()-t0))
    time.sleep(1.2)
if samples:
    xs=[s[0] for s in samples]; ys=[s[1] for s in samples]
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    devs=[math.hypot(x-mx,y-my) for x,y in [(s[0],s[1]) for s in samples]]
    rs=[s[3] for s in samples]; bs=[s[2] for s in samples]
    logf.write(f"ORB n={len(samples)} center=({mx:.2f},{my:.2f}) dist={math.hypot(mx,my):.2f} "
               f"spread_mean={sum(devs)/len(devs):.2f} spread_max={max(devs):.2f}\n")
    logf.write(f"ORB r_range=[{min(rs):.2f},{max(rs):.2f}] bearing_range=[{min(bs):.0f},{max(bs):.0f}]\n")
    for s in samples[-20:]:
        logf.write(f"ORB pt t={s[4]:.0f} r={s[3]:.2f} wb={s[2]:.0f}\n")
else:
    logf.write("ORB: no companion sightings!\n")
logf.write("ORBIT end\n")
