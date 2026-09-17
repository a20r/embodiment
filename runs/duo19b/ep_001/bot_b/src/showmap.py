import json, math, sys
CELL = 0.2
occ = {}; free = {}
rows = [json.loads(l) for l in open('/bot/map.log')]
for r in rows:
    x, y, h, s = r['x'], r['y'], r['h'], r['s']
    for k, d in enumerate(s):
        if d < 0: continue
        a = math.radians(h + 22.5 * k)
        dx, dy = math.sin(a), math.cos(a)
        n = int(d / (CELL / 2))
        for i in range(n):
            t = i * CELL / 2
            c = (int(math.floor((x + dx * t) / CELL)), int(math.floor((y + dy * t) / CELL)))
            free[c] = free.get(c, 0) + 1
        if d < 1.35:
            c = (int(math.floor((x + dx * d) / CELL)), int(math.floor((y + dy * d) / CELL)))
            occ[c] = occ.get(c, 0) + 1
path = set((int(math.floor(r['x'] / CELL)), int(math.floor(r['y'] / CELL))) for r in rows)
cells = set(occ) | set(free)
xs = [c[0] for c in cells]; ys = [c[1] for c in cells]
x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
last = rows[-1]; lc = (int(math.floor(last['x'] / CELL)), int(math.floor(last['y'] / CELL)))
print(f'{len(rows)} scans; x[{x0*CELL:.1f},{x1*CELL:.1f}] y[{y0*CELL:.1f},{y1*CELL:.1f}]; robot at ({last["x"]:.2f},{last["y"]:.2f}) hdg {last["h"]}')
for yy in range(y1, y0 - 1, -1):
    line = ''
    for xx in range(x0, x1 + 1):
        c = (xx, yy)
        o = occ.get(c, 0); f = free.get(c, 0)
        if c == lc: ch = 'R'
        elif c in path: ch = '.'
        elif o >= 2 and o >= f * 0.3: ch = '#'
        elif f > 0: ch = ' '
        else: ch = '?'
        line += ch
    print(line)
