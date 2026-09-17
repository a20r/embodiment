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

def band_stats():
    s=read('d2'); horiz=[]; up=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            r,e,a=map(float,p.split(','))
            if r>0.02:
                if -0.1<=e<=0.3: horiz.append(r)
                elif e>0.3: up.append(r)
        except: pass
    def med(v): 
        v=sorted(v); return v[len(v)//2] if v else -1
    return med(horiz), (max(horiz) if horiz else -1), med(up), (max(up) if up else -1)

def enc(): return fl(read('d6')), fl(read('d9'))

r0,l0=enc()
print("start: horizMed horizMax upMed upMax | enc", band_stats(), r0, l0)
for k in range(4):
    t0=time.time()
    while time.time()-t0<1.0:
        w('d1',50); w('d7',50); time.sleep(0.1)
    w('d1',0); w('d7',0); time.sleep(0.2)
    r1,l1=enc()
    print(f"after {(k+1)}s fwd: dEnc={(r1-r0+l1-l0)/2:.0f}", band_stats(), r1, l1)
    if band_stats()[0] < 0.1: print("WALL CLOSE, stopping"); break
