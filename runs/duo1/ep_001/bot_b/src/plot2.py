import math
pts=[]; poses=[]
for line in open("/memory/trail2.csv"):
    p=line.split(",")
    try:
        x,y,h=float(p[1]),float(p[2]),float(p[3])
        rays=[float(v) for v in p[4:20]]
    except: continue
    poses.append((x,y))
    for i,d in enumerate(rays):
        if 0<d<2.9:
            a=math.radians(h+22.5*i)
            pts.append((x+d*math.cos(a),y+d*math.sin(a)))
mnx=min(p[0] for p in poses)-1; mxx=max(p[0] for p in poses)+1
mny=min(p[1] for p in poses)-1; mxy=max(p[1] for p in poses)+1
W,H=100,46
import collections
cnt=collections.Counter()
def cell(x,y):
    c=int((x-mnx)/(mxx-mnx)*(W-1)); r=int((mxy-y)/(mxy-mny)*(H-1)); return r,c
for x,y in pts:
    if mnx<=x<=mxx and mny<=y<=mxy: cnt[cell(x,y)]+=1
grid=[[' ']*W for _ in range(H)]
for (r,c),n in cnt.items():
    if n>=3: grid[r][c]='#'
    elif n>=1: grid[r][c]=':'
for x,y in poses:
    r,c=cell(x,y)
    grid[r][c]='.'
r,c=cell(*poses[-1]); grid[r][c]='R'
print(f"x[{mnx:.1f},{mxx:.1f}] y[{mny:.1f},{mxy:.1f}] each col={(mxx-mnx)/W:.2f} row={(mxy-mny)/H:.2f}")
print("\n".join("".join(row) for row in grid))
