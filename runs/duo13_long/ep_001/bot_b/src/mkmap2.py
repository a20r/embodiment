import math
samples=[]
for line in open("/bot/src/scan1.txt"):
    parts=line.split(None,2)
    if len(parts)<3: continue
    t=float(parts[0]); h=float(parts[1])
    l=[float(x) for x in parts[2].strip("[]\n").split(",")]
    samples.append((h,l))
pts=[]
for h,l in samples:
    for i,d in enumerate(l):
        if d<0: continue
        off=(i-8)*22.5
        a=math.radians(h+off)
        pts.append((d*math.cos(a), d*math.sin(a)))
xs=[p[0] for p in pts]; ys=[p[1] for p in pts]
res=0.12
W=100; H=56
minx,maxx=min(xs),max(xs); miny,maxy=min(ys),max(ys)
grid=[[0]*W for _ in range(H)]
for x,y in pts:
    cx=int((x-minx)/(maxx-minx)*(W-1)); cy=int((y-miny)/(maxy-miny)*(H-1))
    grid[H-1-cy][cx]+=1
for row in grid:
    print("".join("#" if v>=6 else ("+" if v>=3 else ("." if v>0 else " ")) for v in row))
print("x range %.2f..%.2f  y %.2f..%.2f" % (minx,maxx,miny,maxy))
