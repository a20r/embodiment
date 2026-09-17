import json,sys
pts=[]
for line in open('/memory/traj.jsonl'):
    try:
        d=json.loads(line)
        if 'x' in d: pts.append((d['x'],d['y']))
    except: pass
xs=[p[0] for p in pts]; ys=[p[1] for p in pts]
print('n=',len(pts),'x range',min(xs),max(xs),'y range',min(ys),max(ys))
W,H=100,40
gx=lambda x:int((x-min(xs))/(max(xs)-min(xs)+1e-9)*(W-1))
gy=lambda y:int((y-min(ys))/(max(ys)-min(ys)+1e-9)*(H-1))
grid=[[' ']*W for _ in range(H)]
for i,(x,y) in enumerate(pts):
    grid[gy(y)][gx(x)]='.' 
grid[gy(pts[0][1])][gx(pts[0][0])]='S'
grid[gy(pts[-1][1])][gx(pts[-1][0])]='E'
print('\n'.join(''.join(r) for r in grid))
