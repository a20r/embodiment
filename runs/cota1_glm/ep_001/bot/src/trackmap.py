import csv, math, sys
fn=sys.argv[1] if len(sys.argv)>1 else 'auto3_state.csv'
rows=[]
with open(fn) as f:
    for r in csv.reader(f):
        if len(r)<9: continue
        rows.append((float(r[0]),float(r[2]),float(r[3]),float(r[4]),r[7],[float(x) for x in r[8].split(';')]))
x=y=0.0; th=None; pts=[]; pt=rows[0][0]
# heading from wz integration, drift-corrected toward d3 slowly
d3=rows[0][1]
th=math.radians(d3)
for t,hd,v,wz,mode,b in rows:
    dt=min(t-pt,0.15); pt=t
    th+=wz*dt
    # weak correction toward d3 (handle wrap)
    err=(math.radians(hd)-th+math.pi)%(2*math.pi)-math.pi
    th+=0.02*err
    x+=v*math.cos(th)*dt; y+=v*math.sin(th)*dt
    pts.append((x,y,th,v,mode,b))
print(f'n={len(pts)}  end=({x:.1f},{y:.1f})  th={math.degrees(th)%360:.0f}')
xs=[p[0] for p in pts]; ys=[p[1] for p in pts]
x0,x1=min(xs)-.3,max(xs)+.3; y0,y1=min(ys)-.3,max(ys)+.3
W=78;H=32
grid=[[' ']*W for _ in range(H)]
for i,(xx,yy,thh,v,mode,b) in enumerate(pts):
    j=int((xx-x0)/(x1-x0)*(W-1)); k=int((yy-y0)/(y1-y0)*(H-1))
    ch='#' if mode in('FWD','FAST') else ('b' if mode=='BEND' else 'r')
    grid[H-1-k][j]=ch
print('\n'.join(''.join(r) for r in grid))
print('path len (int):', f'{sum(math.hypot(pts[i][0]-pts[i-1][0],pts[i][1]-pts[i-1][1]) for i in range(1,len(pts))):.1f} m')
