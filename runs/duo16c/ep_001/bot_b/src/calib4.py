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

def test(name, v1, v7, dur):
    P0 = prof(scan_pts()); h0 = rl('d4',0.5)
    t0=time.time()
    stream(v1, v7, dur, period=0.05)
    stream('0','0',0.4)
    h1 = rl('d4',0.5)
    P1 = prof(scan_pts())
    common = sorted(set(P0)&set(P1))
    tot = sum(abs(P1[b]-P0[b]) for b in common)/len(common)
    mx = max(common, key=lambda b: abs(P1[b]-P0[b]))
    print(f'{name}: L1avg={tot:.3f} maxbin={mx}({P0[mx]:.2f}->{P1[mx]:.2f}) heading {h0}->{h1} time={time.time()-t0:.0f}s')

test('fwd 2,2 4s', '2','2',4.0)
test('back 2,2 4s', '-2','-2',4.0)
test('fwd 3,3 3s', '3','3',3.0)
