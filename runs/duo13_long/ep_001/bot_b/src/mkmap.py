import math
samples=[]
for line in open("/bot/src/scan1.txt"):
    parts=line.split(None,2)
    if len(parts)<3: continue
    t=float(parts[0]); h=float(parts[1])
    l=[float(x) for x in parts[2].strip("[]\n").split(",")]
    samples.append((h,l))

def test(offset_mode):
    pts=[]
    for h,l in samples:
        for i,d in enumerate(l):
            if d<0: continue
            if offset_mode=="c": off=(i-8)*22.5
            else: off=i*22.5
            a=math.radians(h+off)
            pts.append((d*math.cos(a), d*math.sin(a)))
    return pts

# crude cleanliness metric: histogram occupancy grid sharpness
def score(pts, res=0.1):
    grid={}
    for x,y in pts:
        k=(round(x/res), round(y/res))
        grid[k]=grid.get(k,0)+1
    occupied=len(grid); total=len(pts)
    # fraction of points falling in cells with >=5 points
    big=sum(v for v in grid.values() if v>=5)
    return occupied, big/total

for mode in ("c","a"):
    pts=test(mode)
    print(mode, score(pts))
