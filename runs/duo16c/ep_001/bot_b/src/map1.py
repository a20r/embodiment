import math
from scene import read_line as rl
s = rl('d2', 1.0)
pts = [tuple(map(float,t.split(','))) for t in s.split(';') if t]
print('n =', len(pts))
# top-down ASCII map: 2m x 2m grid at 0.1m? use 60x40 chars covering 6x4 m
W,H = 100, 50
sx, sy = 6.0, 3.0   # meters covered (x forward, y left)
grid = [[ ' ' for _ in range(W)] for _ in range(H)]
cnt = [[0]*W for _ in range(H)]
for x,y,z in pts:
    ix = int((x + 1.0) / sx * (W-1))
    iy = int((y + 1.0) / sy * (H-1))
    if 0 <= ix < W and 0 <= iy < H:
        cnt[iy][ix] += 1
for iy in range(H-1, -1, -1):
    row = ''.join('#' if c>3 else ('+' if c>0 else ' ') for c in cnt[iy])
    print(f'{iy:2d}|{row}|')
print('   +' + '-'*W + '+')
print(f'    x from -1.0 to {sx-1.0} m, y from -1.0 to {sy-1.0} m; row {H-1}=y max')
# range histogram
import collections
rs = sorted(math.hypot(x,y) for x,y,z in pts)
print('range deciles:', [round(rs[int(len(rs)*q/10)],2) for q in range(11)])
