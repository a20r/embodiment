import os, select, time
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

def obj():
    s=read('d2'); hits=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            r,e,a=map(float,p.split(','))
            if r>0.15 and r<1.1 and -0.05<=e<=0.25: hits.append((r,a))
        except: pass
    if len(hits)<3: return None
    hits.sort(); rr=[h[0] for h in hits]; med=rr[len(rr)//2]
    sel=[a for (r,a) in hits if abs(r-med)<0.18]
    return med, sum(sel)/len(sel), len(hits)

def drive(spd, dur):
    t0=time.time()
    while time.time()-t0<dur:
        w('d1',spd); w('d7',spd); time.sleep(0.15)
    w('d1',0); w('d7',0)

for phase,spd,dur in [("STOP",0,4),("FWD25",25,3),("STOP",0,4),("FWD45",45,3),("STOP",0,4),("REV", -30, 2),("STOP",0,3)]:
    if spd: drive(spd,dur)
    else: time.sleep(dur)
    o=obj()
    print(phase, "obj:", "None" if not o else "%.3f@az%.2f n=%d" % o, "d11:", read('d11'))
