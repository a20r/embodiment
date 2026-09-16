import math, time, collections, os
from scene import read_line as rl

def wr(port, val):
    for _ in range(3):
        try:
            fd = os.open(f'/dev/robot/{port}', os.O_WRONLY|os.O_NONBLOCK)
            os.write(fd, f'{val}\n'.encode()); os.close(fd); return
        except Exception:
            time.sleep(0.02)
def stop():
    wr('d1','0'); wr('d7','0')
def wheels(v1, v7, dur, period=0.1):
    end=time.time()+dur
    while time.time()<end:
        wr('d1',str(v1)); wr('d7',str(v7)); time.sleep(period)
    stop()
def enc():
    return float(rl('d6',0.4) or 0), float(rl('d9',0.4) or 0)
def heading():
    return float(rl('d4',0.5) or 0)
def scan_pts(tries=6):
    for _ in range(tries):
        s = rl('d2',1.0)
        if s: return [tuple(map(float,t.split(','))) for t in s.split(';') if t]
        time.sleep(0.05)
    return []
def prof(pts, nb=72):
    az=collections.defaultdict(list)
    for x,y,z in pts:
        r=math.hypot(x,y); a=math.degrees(math.atan2(y,x))
        b=int((a+180)/(360/nb))%nb
        az[b].append(r)
    return {b: sorted(rs)[len(rs)//2] for b,rs in az.items()}
def est_rotation(P0, P1):
    nb=len(P0); best=None; bs=0
    for s in range(-nb//2, nb//2):
        d=sum(abs(P1[(b+s)%nb]-P0[b]) for b in range(nb))
        if best is None or d<best: best=d; bs=s
    return bs*360/nb, best/nb
def flags():
    return rl('d3',0.4)
