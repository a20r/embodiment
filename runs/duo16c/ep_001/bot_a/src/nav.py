import os, select, time, math, sys

LOG = open('/memory/navlog.txt','a', buffering=1)
def log(msg):
    LOG.write(f"[{time.time():.0f}] {msg}\n")

def readp(p, timeout=0.15):
    try:
        fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
        r,_,_ = select.select([fd],[],[],timeout)
        data = os.read(fd, 65536) if r else b''
        os.close(fd)
        return data.decode().strip()
    except Exception:
        return ''

def w(p, s):
    try:
        fd = os.open(f'/dev/robot/{p}', os.O_WRONLY)
        os.write(fd, (s+'\n').encode()); os.close(fd)
    except Exception as e:
        log(f"WERR {p} {e}")

def bearing():
    try: return float(readp('d4'))
    except: return None

def flags():
    t = readp('d3')
    try:
        parts = dict(kv.split('=') for kv in t.split() if '=' in kv)
        return int(parts.get('goal','0')), int(parts.get('here','0')), int(parts.get('tick','0'))
    except:
        return 0,0,0

def lidar_ahead():
    c = readp('d2', 1.5)
    best=None
    try:
        for p in c.split(';'):
            if not p: continue
            x,y,z = map(float,p.split(','))
            r = math.hypot(x,y)
            if r<0.55: continue
            az = math.degrees(math.atan2(y,x))
            if abs(az)<50:
                if best is None or r<best[0]: best=(r,az)
    except Exception:
        pass
    return best

def radio(msg):
    w('d8', msg)

log("nav.py started pid=%d"%os.getpid())
state='drive'; last_d9=None; stuck_t=0; loop=0
escape_until=0
try:
    while True:
        loop+=1
        g,h,tick = flags()
        b = bearing()
        d9 = readp('d9'); d6=readp('d6'); d11=readp('d11')
        now=time.time()
        if loop%20==0:
            log(f"loop={loop} state={state} goal={g} here={h} d4={b} d9={d9} d6={d6} d11={d11}")
            radio(f"R1 POS d9={d9} d6={d6} d4={b} tick={tick}")
        if g or h:
            w('d1','0'); w('d7','0')
            log(f"ARRIVED? goal={g} here={h} d4={b} d9={d9} d6={d6} d11={d11} tick={tick}")
            radio(f"R1 ATGOAL d9={d9} d6={d6} tick={tick}")
            time.sleep(0.5)
            continue
        if now < escape_until:
            time.sleep(0.1); continue
        if b is None:
            w('d1','0'); w('d7','0'); time.sleep(0.2); continue
        err = ((b+180)%360)-180
        obst = lidar_ahead() if loop%3==0 else None
        if obst and obst[0]<0.8:
            w('d1','0')
            d7v = int(-math.copysign(22, obst[1]))  # obstacle left(az>0)->turn right(d7<0)
            w('d7', str(d7v))
            state='avoid'
            log(f"OBSTACLE r={obst[0]:.2f} az={obst[1]:.0f} -> d7={d7v}")
            time.sleep(0.3)
            continue
        if abs(err)>12:
            w('d1','0')
            d7v = max(-20, min(20, int(round(err*1.8))))
            w('d7', str(d7v))
            state='turn'
        else:
            w('d7', str(max(-20, min(20, int(round(err*1.5))))))
            w('d1','2')
            state='drive'
            # stuck detection
            try:
                if last_d9 is not None and abs(int(d9)-last_d9)<1:
                    stuck_t+=0.2
                else:
                    stuck_t=0; last_d9=int(d9)
            except: pass
            if stuck_t>4:
                log(f"STUCK d9={d9} -> escape")
                w('d1','-2'); time.sleep(1.3)
                w('d1','0'); w('d7', str(18 if loop%2 else -18)); time.sleep(0.9)
                w('d7','0'); stuck_t=0; last_d9=None
                escape_until=time.time()+0.2
        time.sleep(0.2)
except Exception as e:
    log(f"CRASH {type(e).__name__} {e}")
    w('d1','0'); w('d7','0')
