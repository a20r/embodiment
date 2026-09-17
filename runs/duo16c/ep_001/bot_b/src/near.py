import time, math, subprocess
from scene import read_line as rl

def scan_pts():
    for _ in range(5):
        s = rl('d2', 1.0)
        if s:
            return [tuple(map(float,t.split(','))) for t in s.split(';') if t]
        time.sleep(0.05)
    return []

for k in range(10):
    pts = scan_pts()
    near = [p for p in pts if 0 < p[0] < 0.35 and abs(p[1]) < 0.35]
    if near:
        xs=[p[0] for p in near]; ys=[p[1] for p in near]; zs=[p[2] for p in near]
        print(f'{k}: n={len(near):4d} x[{min(xs):.3f},{max(xs):.3f}] y[{min(ys):.3f},{max(ys):.3f}] z[{min(zs):.3f},{max(zs):.3f}] cx={sum(xs)/len(xs):.3f} cy={sum(ys)/len(ys):.3f}')
    else:
        print(f'{k}: nothing near')
    print('   d0=%s d5=%s d11=%s d3=%s d4=%s' % (rl('d0',0.3), rl('d5',0.3), rl('d11',0.3), rl('d3',0.3), rl('d4',0.3)))
    time.sleep(1.5)
