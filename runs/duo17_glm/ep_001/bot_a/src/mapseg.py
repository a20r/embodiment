import json, math, sys
NSEG=int(sys.argv[1]) if len(sys.argv)>1 else 1
RES=float(sys.argv[2]) if len(sys.argv)>2 else 0.5
rows=[json.loads(l) for l in open("/memory/track.log")]
segs=[]; cur=[rows[0]]
for a,b in zip(rows,rows[1:]):
    if math.hypot(b['x']-a['x'],b['y']-a['y'])>50: segs.append(cur); cur=[b]
    else: cur.append(b)
segs.append(cur)
print("segments:",[len(s) for s in segs])
rows=segs[-NSEG]
pts=[]
for r in rows:
    x,y,h=r['x'],r['y'],r['h']
    pts.append((x,y,2))
    for k,v in enumerate(r['s']):
        if v<0: v=3.0
        a=math.radians(h+k*22.5)
        d=0.4
        while d < v-0.3:
            pts.append((x+d*math.cos(a), y+d*math.sin(a),0)); d+=0.6
        if 0.1 < v < 2.9:
            pts.append((x+v*math.cos(a), y+v*math.sin(a),1))
xs=[p[0] for p in pts]; ys=[p[1] for p in pts]
minx,maxx,miny,maxy=min(xs),max(xs),min(ys),max(ys)
W=int((maxx-minx)/RES)+2; H=int((maxy-miny)/RES)+2
free=[[0]*W for _ in range(H)]; occ=[[0]*W for _ in range(H)]; trail=[[0]*W for _ in range(H)]
for px,py,o in pts:
    cx=int((px-minx)/RES); cy=int((py-miny)/RES)
    if 0<=cx<W and 0<=cy<H:
        if o==2: trail[cy][cx]+=1
        elif o==1: occ[cy][cx]+=1
        else: free[cy][cx]+=1
out=[]
for j in range(H-1,-1,-1):
    row=""
    for i in range(W):
        if trail[j][i]>0: row+="o"
        elif occ[j][i]>=3: row+="#"
        elif free[j][i]>=2: row+="."
        else: row+=" "
    out.append(row)
print("last %d rows: x[%.0f..%.0f] y[%.0f..%.0f]"%(len(rows),minx,maxx,miny,maxy))
open("/memory/map_recent.txt","w").write("\n".join(out))
print("\n".join(out))
