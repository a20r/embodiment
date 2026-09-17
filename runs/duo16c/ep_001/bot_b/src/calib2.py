import math, time, collections
from scene import read_line as rl, stream

def scan_pts():
    for _ in range(5):
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

def enc():
    return float(rl('d6',0.5) or 0), float(rl('d9',0.5) or 0)

P0 = prof(scan_pts()); h0 = rl('d4',0.5); e0=enc()
print('SPIN 10s d1=2 d7=-2')
stream('2','-2',10.0, period=0.05)
h1 = rl('d4',0.5); e1=enc()
P1 = prof(scan_pts())
stream('0','0',1.0)
print('heading:', h0, '->', h1, ' enc delta:', e1[0]-e0[0], e1[1]-e0[1])
common = sorted(set(P0)&set(P1))
print('profile diffs (72 bins, every 2nd):')
print(' '.join(f'{b}:{P1[b]-P0[b]:+.2f}' for b in common[::2]))
print('P0 bin12-13 vals:', {b: round(P0[b],2) for b in [11,12,13,14]})
print('P1 bin12-13 vals:', {b: round(P1[b],2) for b in [11,12,13,14]})
