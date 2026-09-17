import math, time, collections
from scene import read_line as rl, stream

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

P0 = prof(scan_pts()); h0 = rl('d4',0.5)
print('BACK 3s')
stream('-1','-1',3.0, period=0.05)
stream('0','0',0.5)
h1 = rl('d4',0.5)
P1 = prof(scan_pts())
print('heading:', h0, '->', h1)
common = sorted(set(P0)&set(P1))
print(' '.join(f'{b}:{P1[b]-P0[b]:+.2f}' for b in common))
# where is free space? ranges
print('P1 ranges:', ' '.join(f'{b}:{P1[b]:.2f}' for b in sorted(P1)))
