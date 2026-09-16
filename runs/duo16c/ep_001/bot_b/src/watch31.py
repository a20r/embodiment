import math, time, collections
from scene import read_line as rl

def scan_pts():
    for _ in range(5):
        s = rl('d2',1.0)
        if s: return [tuple(map(float,t.split(','))) for t in s.split(';') if t]
        time.sleep(0.05)
    return []

def prof(pts, nb=36):
    az=collections.defaultdict(list)
    for x,y,z in pts:
        r=math.hypot(x,y); a=math.degrees(math.atan2(y,x))
        b=int((a+180)/10)%nb
        az[b].append(r)
    return {b: sorted(rs)[len(rs)//2] for b,rs in az.items()}

print('passive watch 30s, no commands:')
prev=None
for k in range(10):
    P = prof(scan_pts())
    h = rl('d4',0.3)
    s = ' '.join(f'{b}:{P[b]:.2f}' for b in sorted(P) if P[b]<0.6)
    print(f'{k}: close bins: {s} | d4={h}')
    time.sleep(3)
