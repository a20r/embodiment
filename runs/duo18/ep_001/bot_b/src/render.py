#!/usr/bin/env python3
"""Render ASCII map from /bot/track.log: '#' lidar hits, '.' path, 'R' last pose, 'G' goal event."""
import json, math, sys, collections
CELL = float(sys.argv[1]) if len(sys.argv) > 1 else 0.1
MAXR = 2.5
hits = collections.Counter(); path = set(); last = None; goals = []
for line in open("/bot/track.log"):
    try: r = json.loads(line)
    except: continue
    if "event" in r:
        if r["event"] == "GOAL" and last: goals.append(last)
        continue
    x, y, h = r["x"], r["y"], r["h"]; last = (x, y, h)
    path.add((int(round(x/CELL)), int(round(y/CELL))))
    for i, v in enumerate(r["scan"]):
        if v is None or v >= MAXR or v < 0.05: continue
        a = math.radians(h + i*22.5)
        hx, hy = x + v*math.sin(a), y + v*math.cos(a)
        hits[(int(round(hx/CELL)), int(round(hy/CELL)))] += 1
cells = set(hits) | path
xs = [c[0] for c in cells]; ys = [c[1] for c in cells]
x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
print(f"# cell={CELL} x:[{x0*CELL:.1f},{x1*CELL:.1f}] y:[{y0*CELL:.1f},{y1*CELL:.1f}] top=north; R=({last[0]:.2f},{last[1]:.2f},h{last[2]:.0f})")
rc = (int(round(last[0]/CELL)), int(round(last[1]/CELL)))
gcs = set((int(round(g[0]/CELL)), int(round(g[1]/CELL))) for g in goals)
for j in range(y1, y0-1, -1):
    row = ""
    for i in range(x0, x1+1):
        c = (i, j)
        if c == rc: row += "R"
        elif c in gcs: row += "G"
        elif c in path: row += "."
        elif hits.get(c, 0) >= 2: row += "#"
        elif hits.get(c, 0) == 1: row += "+"
        else: row += " "
    print(row)
