import os, select, time, math
LOG=open('/memory/navlog.txt','a',buffering=1)
def log(m): LOG.write(f"[{time.time():.0f}] MOWER {m}\n")
def readp(p, timeout=0.15):
    try:
        fd=os.open(f'/dev/robot/{p}',os.O_RDONLY)
        r,_,_=select.select([fd],[],[],timeout)
        d=os.read(fd,1<<22) if r else b''
        os.close(fd); return d.decode().strip()
    except Exception: return ''
def w(p,s):
    try:
        fd=os.open(f'/dev/robot/{p}',os.O_WRONLY); os.write(fd,(s+'\n').encode()); os.close(fd)
    except Exception as e: log(f"WERR{p}:{e}")
def fl():
    t=readp('d3')
    try:
        parts=dict(kv.split('=') for kv in t.split() if '=' in kv)
        return int(parts.get('goal','0')), int(parts.get('here','0'))
    except Exception: return 0,0
def poi():
    c=readp('d2',1.2); out=[]
    try:
        for p in c.split(';'):
            if not p: continue
            x,y,z=map(float,p.split(','))
            r=math.hypot(x,y)
            if r>0.55: out.append((r, round(math.degrees(math.atan2(y,x)))))
        out.sort()
        if not out: return []
        pr=out[0][0]
        res=[(round(r,1),a) for r,a in out if r>max(pr+0.8,1.2) and abs(a)<100]
        return res[:3]
    except Exception: return []
def rd9():
    try: return int(readp('d9'))
    except Exception: return None
log("mower2 start")
legs=0
while True:
    g,h=fl()
    if g or h:
        w('d1','0'); w('d7','0')
        log(f"ARRIVED? g={g} h={h} d9={readp('d9')} d6={readp('d6')}")
        w('d8',f"R1 AT {readp('d9')} {readp('d6')}")
        time.sleep(0.5); continue
    legs+=1
    log(f"leg {legs} begin d9={rd9()} d6={readp('d6')}")
    t0=time.time(); d0=rd9(); pins=0
    while time.time()-t0<22:
        g,h=fl()
        if g or h: break
        w('d1','6'); w('d7','0')
        time.sleep(0.5)
        P=poi()
        if P: log(f"POI leg{legs} {P} d9={rd9()}")
        d1=rd9()
        if d0 is not None and d1 is not None and abs(d1-d0)<4:
            pins+=1
            log(f"leg{legs} PINNED d9={d1}")
            w('d1','-3'); time.sleep(1.1)
            w('d1','0')
            td = 20 if (legs+pins)%2==0 else -20
            t2=time.time()
            while time.time()-t2<2.2: w('d7',str(td)); time.sleep(0.05)
            w('d7','0')
            d0=rd9(); t0=time.time()  # restart leg clock
        else:
            if d1 is not None: d0=d1
        if pins>=4:
            log(f"leg{legs} too many pins, ending"); break
    w('d1','0')
    if g or h: continue
    # snake turn: +90deg, short reverse jog, +90deg again
    for phase in range(2):
        t1=time.time()
        while time.time()-t1<5.1: w('d7','20'); time.sleep(0.05)
        w('d7','0'); w('d1','0'); time.sleep(0.2)
        if phase==0:
            w('d1','-6'); time.sleep(4.0); w('d1','0'); time.sleep(0.2)
