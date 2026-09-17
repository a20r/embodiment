import os, select, time
fd = os.open('/dev/robot/d2', os.O_RDONLY)
r,_,_ = select.select([fd],[],[],3)
data = os.read(fd, 1<<22).decode().strip()
os.close(fd)
pts = [tuple(map(float,p.split(','))) for p in data.split(';') if p]
print("n =", len(pts))
xs=[p[0] for p in pts]; ys=[p[1] for p in pts]; zs=[p[2] for p in pts]
print("x: %.2f..%.2f  y: %.2f..%.2f  z: %.2f..%.2f" % (min(xs),max(xs),min(ys),max(ys),min(zs),max(zs)))
yls = sorted(set(round(y,3) for y in ys))
print("unique y layers:", len(yls), yls[:40])
# angular coverage: azimuths of far points
import math
far = [p for p in pts if p[0]>3]
print("far points (x>3):", len(far))
for p in far[:10]: print(p)
