import json, math, sys
RES=1.0  # grid cell size in world units
# collect poses+scans from track.log (multiple runs appended; detect resets by x,y jumps)
rows=[json.loads(l) for l in open("/memory/track.log")]
# segments: new segment when jump > 50 from previous row
segs=[]; cur=[rows[0]]
for a,b in zip(rows,rows[1:]):
    if math.hypot(b['x']-a['x'],b['y']-a['y'])>50:
        segs.append(cur); cur=[b]
    else: cur.append(b)
segs.append(cur)
print("segments:", [len(s) for s in segs])
minx=miny=1e9; maxx=maxy=-1e9
pts=[]
for seg in segs:
    for r in seg:
        x,y,h=r['x'],r['y'],r['h']
        for k,v in enumerate(r['s']):
            if v<0: v=3.0  # max range no-return
            a=math.radians(h+k*22.5)
            # free cells along ray (step 0.5)
            d=0.4
            while d < v-0.3:
                fx,fy=x+d*math.cos(a), y+d*math.sin(a)
                pts.append((fx,fy,0)); d+=0.6
            if 0.1 < v < 2.9:
                ex,ey=x+v*math.cos(a), y+v*math.sin(a)
                pts.append((ex,ey,1))
        minx=min(minx,x);maxx=max(maxx,x);miny=min(miny,y);maxy=max(maxy,y)
print("bounds x",round(minx),round(maxx),"y",round(miny),round(maxy))
# grid
W=int((maxx-minx)/RES)+2; H=int((maxy-miny)/RES)+2
free=[[0]*W for _ in range(H)]; occ=[[0]*W for _ in range(H)]
for px,py,o in pts:
    cx=int((px-minx)/RES); cy=int((py-miny)/RES)
    if 0<=cx<W and 0<=cy<H:
        (occ if o else free)[cy][cx]+=1
grid=[]
for j in range(H-1,-1,-1):
    row=""
    for i in range(W):
        if occ[j][i]>=3: row+="#"
        elif free[j][i]>=2: row+="."
        else: row+=" "
    grid.append(row)
open("/memory/map.txt","w").write("\n".join(grid))
print("map size",W,"x",H,"written to /memory/map.txt")
# print coarse view: downsample by 4
for j in range(0,H,4):
    print("".join(grid[j][i] for i in range(0,W,4)))
