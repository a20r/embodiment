import os, select, time, math, json
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

def capture():
    s=read('d2'); pts=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            r,e,a=map(float,p.split(',')); pts.append((r,e,a))
        except: pass
    return pts

# rotate 360 in 24 steps; at each, capture; summarize horiz-band returns
h0=fl(read('d4'))
cur=h0
report={}
for k in range(24):
    pts=capture()
    # bucket horiz-band returns (elev -0.05..0.35) by azimuth in 0.05 bins
    buckets={}
    for r,e,a in pts:
        if r>0.05 and -0.05<=e<=0.35:
            b=round(a/0.05)*0.05
            buckets.setdefault(b,[]).append(r)
    summary={b:(round(min(v),2),round(max(v),2),len(v)) for b,v in sorted(buckets.items())}
    report[round(cur,1)]=summary
    # rotate 15 deg
    t0=time.time()
    while time.time()-t0<3:
        err=((cur+15-fl(read('d4'),cur)+180)%360)-180
        if abs(err)<4: break
        spd=max(-45,min(45,err*2.0))
        w('d1',spd); w('d7',-spd); time.sleep(0.1)
    w('d1',0); w('d7',0); time.sleep(0.15)
    cur=fl(read('d4'),cur)
for h in sorted(report):
    s=report[h]
    # print headings with returns beyond 0.25 (beyond floor)
    interesting={b:v for b,v in s.items() if v[1]>0.25}
    print(f"h={h:6.1f} far_returns={interesting if interesting else '-'}")
