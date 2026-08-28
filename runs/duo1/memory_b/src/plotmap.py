import math
pts=[]; poses=[]
for line in open("/memory/trail.csv"):
    p=line.split(",")
    try:
        x,y,h = float(p[1]),float(p[2]),float(p[3])
        rays=[float(v) for v in p[4:20]]
    except: continue
    poses.append((x,y))
    for i,d in enumerate(rays):
        if 0<d<2.9:
            a=math.radians(h+22.5*i)
            pts.append((x+d*math.cos(a), y+d*math.sin(a)))
xs=[p[0] for p in pts]; ys=[p[1] for p in pts]
mnx,mxx,mny,mxy=min(xs),max(xs),min(ys),max(ys)
W,H=70,35
grid=[[' ']*W for _ in range(H)]
def cell(x,y):
    c=int((x-mnx)/(mxx-mnx+1e-9)*(W-1)); r=int((mxy-y)/(mxy-mny+1e-9)*(H-1)); return r,c
for x,y in pts:
    r,c=cell(x,y); grid[r][c]='#'
for x,y in poses:
    r,c=cell(x,y)
    if grid[r][c]!='#': grid[r][c]='.'
r,c=cell(*poses[-1]); grid[r][c]='R'
print(f"x[{mnx:.1f},{mxx:.1f}] y[{mny:.1f},{mxy:.1f}]")
print("\n".join("".join(row) for row in grid))
