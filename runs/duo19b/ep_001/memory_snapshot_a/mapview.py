import json, math, sys
rows=[json.loads(l) for l in open('/tmp/log.jsonl')]
res=0.1; occ={}; free=set(); path=set()
for r in rows[::3]:
    if not r['scan'] or r['busy']=='rot': continue
    x,y,h=r['x'],r['y'],r['h']; path.add((int(math.floor(x/res)),int(math.floor(y/res))))
    for i,v in enumerate(r['scan']):
        if v is None or v<0.05: continue
        a=math.radians(h+22.5*i)
        if v<2.5:
            ex,ey=x+v*math.cos(a),y+v*math.sin(a); k=(int(math.floor(ex/res)),int(math.floor(ey/res))); occ[k]=occ.get(k,0)+1
        n=int(v/res)
        for j in range(1,n): free.add((int(math.floor((x+j*res*math.cos(a))/res)),int(math.floor((y+j*res*math.sin(a))/res))))
xs=[k[0] for k in occ]; ys=[k[1] for k in occ]
x0,x1,y0,y1=min(xs),max(xs),min(ys),max(ys)
cur=rows[-1]; ck=(int(math.floor(cur['x']/res)),int(math.floor(cur['y']/res)))
print('map x:%.1f..%.1f y:%.1f..%.1f (rows=y desc, cols=x). #=wall .=seen free o=path R=robot'%(x0*res,x1*res,y0*res,y1*res))
for yy in range(y1,y0-1,-1):
    line=''
    for xx in range(x0,x1+1):
        k=(xx,yy)
        if k==ck: c='R'
        elif k in path: c='o'
        elif occ.get(k,0)>=3: c='#'
        elif k in free: c='.'
        elif occ.get(k,0)>0: c='+'
        else: c=' '
        line+=c
    print('%5.1f %s'%(yy*res,line))
