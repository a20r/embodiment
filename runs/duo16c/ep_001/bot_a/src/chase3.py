import os, select, time, math
LOG=open('/memory/navlog.txt','a',buffering=1)
def log(m): LOG.write(f"[{time.time():.0f}] CHASE3 {m}\n")
def readp(p, timeout=0.12):
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
def ahead_obj():
    c=readp('d2',1.2); best=None
    try:
        for p in c.split(';'):
            if not p: continue
            x,y,z=map(float,p.split(','))
            r=math.hypot(x,y)
            if r>0.55:
                az=math.degrees(math.atan2(y,x))
                if abs(az)<35 and (best is None or r<best[0]): best=(round(r,2),round(az))
    except Exception: pass
    return best
log("chase3v2 start")
loop=0
last_d9=None; stuck=0
while True:
    loop+=1
    g,h=fl()
    b=readp('d4'); d9=readp('d9')
    if g or h:
        w('d1','0'); w('d7','0')
        log(f"ARRIVED? g={g} h={h} d9={d9} d6={readp('d6')}")
        w('d8',f"R1 ATGOAL {d9}")
        time.sleep(0.4); continue
    try: err=((float(b)+180)%360)-180
    except Exception: err=0
    w('d7', str(max(-20,min(20,int(round(err*2))))))
    w('d1','6')
    if loop%20==0:
        log(f"loop={loop} d4={b} d9={d9} d6={readp('d6')}")
        w('d8',f"R1 CHASE d9={d9}")
    # pin recovery
    try:
        d9i=int(d9)
        if last_d9 is not None and abs(d9i-last_d9)<2:
            stuck+=0.1
        else:
            stuck=0; last_d9=d9i
    except Exception: pass
    if stuck>2.5:
        log(f"PINNED d9={d9} -> escape")
        w('d1','-4'); time.sleep(1.2)
        w('d1','0'); td=18 if loop%2 else -18
        t2=time.time()
        while time.time()-t2<1.5: w('d7',str(td)); time.sleep(0.05)
        w('d7','0'); stuck=0; last_d9=None
    o=ahead_obj()
    if o is not None:
        log(f"OBJ-AHEAD r={o[0]} az={o[1]} d9={d9}")
        if o[0]<1.2:
            w('d1','1')   # gentle closure
        elif o[0]<2.5:
            w('d1','3')
    time.sleep(0.1)
