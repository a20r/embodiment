import json, math, sys
tmin=float(sys.argv[1]) if len(sys.argv)>1 else 0
res=0.15
pts={}; poses=[]
for line in open('/memory/maplog.jsonl'):
    try: r=json.loads(line)
    except: continue
    if r.get('t',0)<tmin or 'L' not in r: continue
    poses.append((r['x'],r['y']))
    for i,d in enumerate(r['L']):
        if d is None or d<0 or d>2.3: continue
        b=math.radians((r['h']+22.5*i)%360)
        pts[(round((r['x']+d*math.cos(b))/res),round((r['y']+d*math.sin(b))/res))]=1
xs=[k[0] for k in pts]+[round(p[0]/res) for p in poses]
ys=[k[1] for k in pts]+[round(p[1]/res) for p in poses]
x0,x1,y0,y1=min(xs),max(xs),min(ys),max(ys)
grid=[[' ']*(x1-x0+1) for _ in range(y1-y0+1)]
for (x,y) in pts: grid[y-y0][x-x0]='#'
for p in poses: grid[round(p[1]/res)-y0][round(p[0]/res)-x0]='.'
px,py=poses[-1]
grid[round(py/res)-y0][round(px/res)-x0]='R'
for row in reversed(grid): print(''.join(row))
print("x[%.1f,%.1f] y[%.1f,%.1f] poses=%d last=%.2f,%.2f"%(x0*res,x1*res,y0*res,y1*res,len(poses),px,py))
