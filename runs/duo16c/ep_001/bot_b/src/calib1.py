import math, time, collections
from scene import read_line as rl, stream

def scan_pts():
    for _ in range(5):
        s = rl('d2',1.0)
        if s:
            return [tuple(map(float,t.split(','))) for t in s.split(';') if t]
        time.sleep(0.05)
    return []

def prof(pts):
    az=collections.defaultdict(list)
    for x,y,z in pts:
        r=math.hypot(x,y); a=math.degrees(math.atan2(y,x))
        b=int((a+180)/10)%36
        az[b].append(r)
    return {b: sorted(rs)[len(rs)//2] for b,rs in az.items()}

def enc(): 
    a=rl('d6',0.5); b=rl('d9',0.5)
    return float(a or 0), float(b or 0)

def drive(v1, v7, dur):
    e0 = enc(); h0 = rl('d4',0.5)
    t0=time.time()
    stream(v1, v7, dur, period=0.05)
    e1 = enc(); h1 = rl('d4',0.5)
    return e1[0]-e0[0], e1[1]-e0[1], h0, h1

P0 = prof(scan_pts())
print('fwd...')
r = drive('1','1',2.0)
print('enc d6,d9 delta:', r[0], r[1], 'heading', r[2], '->', r[3])
P1 = prof(scan_pts())
print('back...')
r = drive('-1','-1',2.0)
print('enc delta:', r[0], r[1], 'heading', r[2], '->', r[3])
P2 = prof(scan_pts())
print('spin...')
r = drive('1','-1',2.0)
print('enc delta:', r[0], r[1], 'heading', r[2], '->', r[3])
P3 = prof(scan_pts())
stream('0','0',1.0)
print('stopped')

def cmp(pa, pb, name):
    common = sorted(set(pa)&set(pb))
    # fit shift: for each bin, diff
    diffs = [(b, pb[b]-pa[b]) for b in common]
    print(name, ' '.join(f'{b}:{d:+.2f}' for b,d in diffs[::2]))

cmp(P0,P1,'P0->P1(fwd)')
cmp(P1,P2,'P1->P2(back)')
cmp(P2,P3,'P2->P3(spin)')
