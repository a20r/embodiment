import sys, time
sys.path.insert(0,'/bot/src')
from rob import *
s = read_port('d2', 2.0).decode(errors='replace')
lines = s.split('\n')
print('num lines in buffer:', len(lines), 'lens:', [len(l) for l in lines[:5]])
big = max(lines, key=len)
pts=[]
for p in big.split(';'):
    q=p.split(',')
    if len(q)==3:
        try: pts.append(tuple(float(v) for v in q))
        except: pass
print('pts in longest line:', len(pts))
# group by y value
from collections import defaultdict
rows=defaultdict(list)
for x,y,z in pts: rows[y].append((x,z))
ys=sorted(rows)
print('distinct y rows:', len(ys))
print('first y:', ys[:6])
print('row y=0.0: n=%d z[%.3f,%.3f] x[%.3f,%.3f]'%(len(rows[ys[0]]),min(z for _,z in rows[ys[0]]),max(z for _,z in rows[ys[0]]),min(x for x,_ in rows[ys[0]]),max(x for x,_ in rows[ys[0]])))
import statistics
for y in ys[::40]:
    r=rows[y]
    print('y=%.3f n=%d x[%.2f,%.2f] z[%.2f,%.2f]'%(y,len(r),min(x for x,_ in r),max(x for x,_ in r),min(z for _,z in r),max(z for _,z in r)))
# check z ordering within a row
r=rows[ys[0]]
print('first row first 12 (x,z):', [(round(x,3),round(z,3)) for x,z in r[:12]])
