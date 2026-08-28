import math
lines=open("/memory/cover_trail.csv").read().splitlines()[290:]
pts=[];poses=[]
for line in lines:
    p=line.split(",")
    try:
        x,y,h=float(p[1]),float(p[2]),float(p[3]); rays=[float(q) for q in p[4:20]]
    except: continue
    poses.append((x,y))
    for i,d in enumerate(rays):
        if 0<d<2.9:
            a=math.radians(h+22.5*i)
            pts.append((x+d*math.cos(a),y+d*math.sin(a)))
xs=[p[0] for p in pts];ys=[p[1] for p in pts]
mnx,mxx,mny,mxy=min(xs),max(xs),min(ys),max(ys)
W,H=100,44
import collections
cnt=collections.Counter()
def cell(x,y): return int((mxy-y)/(mxy-mny+1e-9)*(H-1)),int((x-mnx)/(mxx-mnx+1e-9)*(W-1))
for x,y in pts: cnt[cell(x,y)]+=1
g=[[' ']*W for _ in range(H)]
for (r,c),n in cnt.items(): g[r][c]='#' if n>2 else ':'
for x,y in poses:
    r,c=cell(x,y)
    if g[r][c] != '#': g[r][c]='.'
r,c=cell(*poses[-1]); g[r][c]='R'
print(f"x[{mnx:.1f},{mxx:.1f}] y[{mny:.1f},{mxy:.1f}]")
print("\n".join("".join(q) for q in g))
