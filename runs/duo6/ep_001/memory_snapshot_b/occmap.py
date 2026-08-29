import json,math,sys
cell=0.25
occ={}; free={}
pts=[]
for line in open('/memory/traj.jsonl'):
    try: d=json.loads(line)
    except: continue
    if 'x' not in d: continue
    x,y,h,l=d['x'],d['y'],d['h'],d['l']
    pts.append((x,y))
    for i,r in enumerate(l):
        if r<0: continue
        a=math.radians(h+22.5*i)
        # free cells along ray
        steps=int(r/cell)
        for s in range(steps):
            fx=x+math.cos(a)*s*cell; fy=y+math.sin(a)*s*cell
            free[(int(fx//cell),int(fy//cell))]=1
        if r<2.98:
            ox=x+math.cos(a)*r; oy=y+math.sin(a)*r
            k=(int(ox//cell),int(oy//cell))
            occ[k]=occ.get(k,0)+1
ks=list(occ)+list(free)
xs=[k[0] for k in ks]; ys=[k[1] for k in ks]
x0,x1=min(xs),max(xs); y0,y1=min(ys),max(ys)
print('grid',x0,x1,y0,y1)
for gy in range(y1,y0-1,-1):
    row=''
    for gx in range(x0,x1+1):
        k=(gx,gy)
        if occ.get(k,0)>=3: row+='#'
        elif k in free: row+='.'
        else: row+=' '
    print(row)
# mark robot
print('robot at', pts[-1])
