import os, select, time, math
LOG = open('/memory/navlog.txt','a', buffering=1)
def log(m): LOG.write(f"[{time.time():.0f}] {m}\n")
def readp(p, timeout=0.15):
    try:
        fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
        r,_,_ = select.select([fd],[],[],timeout)
        d = os.read(fd, 65536) if r else b''
        os.close(fd); return d.decode().strip()
    except Exception: return ''
def w(p,s):
    try:
        fd=os.open(f'/dev/robot/{p}',os.O_WRONLY); os.write(fd,(s+'\n').encode()); os.close(fd)
    except Exception as e: log(f"WERR {p} {e}")
def flags():
    t=readp('d3')
    try:
        parts=dict(kv.split('=') for kv in t.split() if '=' in kv)
        return int(parts.get('goal','0')), int(parts.get('here','0')), int(parts.get('tick','0'))
    except Exception: return 0,0,0
def bearing():
    try: return float(readp('d4'))
    except Exception: return None
def lidar_ahead():
    c=readp('d2',1.5); best=None
    try:
        for p in c.split(';'):
            if not p: continue
            x,y,z=map(float,p.split(','))
            r=math.hypot(x,y)
            if r<0.55: continue
            az=math.degrees(math.atan2(y,x))
            if abs(az)<50 and (best is None or r<best[0]): best=(r,az)
    except Exception: pass
    return best
def toint(s, default=None):
    try: return int(s)
    except Exception: return default
log("nav3 started")
loop=0; last_d9=None; stuck=0; d9_0=None; warned=False
while True:
    loop+=1
    g,h,tick=flags(); b=bearing()
    d9=toint(readp('d9'),None); d6=toint(readp('d6'),None)
    if loop%25==0:
        log(f"loop={loop} goal={g} here={h} d4={b} d9={d9} d6={d6}")
        w('d8', f"R1 POS d9={d9} d6={d6} tick={tick}")
    if g or h:
        w('d1','0'); w('d7','0')
        log(f"ARRIVED? goal={g} here={h} d4={b} d9={d9} d6={d6}")
        w('d8', f"R1 ATGOAL d9={d9} d6={d6} tick={tick}")
        time.sleep(0.5); continue
    if d9 is not None:
        if d9_0 is None: d9_0=d9
        if not warned and d9-d9_0>800:
            log("WARNING: >800 units toward goal, no flags.")
            warned=True
    if b is None:
        w('d1','0'); w('d7','0'); time.sleep(0.2); continue
    err=((b+180)%360)-180
    if loop%3==0:
        obst=lidar_ahead()
        if obst and obst[0]<0.45:
            w('d1','0')
            d7v=int(-math.copysign(22,obst[1]))
            w('d7',str(d7v)); log(f"OBST r={obst[0]:.2f} az={obst[1]:.0f} d7={d7v}")
            time.sleep(0.25); continue
    if abs(err)>8:
        w('d1','0'); w('d7',str(max(-22,min(22,int(round(err*1.2))))))
    else:
        w('d7',str(max(-20,min(20,int(round(err*2))))))
        w('d1','4')
        if last_d9 is not None and d9 is not None and abs(d9-last_d9)<1: stuck+=0.15
        elif d9 is not None: stuck=0; last_d9=d9
        if stuck>4:
            log(f"STUCK d9={d9}"); w('d1','-2'); time.sleep(1.2)
            w('d1','0'); w('d7','18' if loop%2 else '-18'); time.sleep(0.8)
            w('d7','0'); stuck=0; last_d9=None
    time.sleep(0.15)
