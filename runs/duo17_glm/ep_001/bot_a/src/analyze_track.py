import json,math,collections
rows=[json.loads(l) for l in open("/memory/track.log")]
print("rows",len(rows),"t_end",rows[-1]['t'])
d=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(rows,rows[1:]))
print("path len",round(d))
# bounds
xs=[r['x'] for r in rows]; ys=[r['y'] for r in rows]
print("x range",round(min(xs)),round(max(xs))," y range",round(min(ys)),round(max(ys)))
# ascii path
W,H=100,36
x0,x1=min(xs),max(xs); y0,y1=min(ys),max(ys)
grid=[[' ']*W for _ in range(H)]
for r in rows[::5]:
    cx=int((r['x']-x0)/(x1-x0+1e-9)*(W-1)); cy=int((r['y']-y0)/(y1-y0+1e-9)*(H-1))
    grid[H-1-cy][cx]='#'
print("\n".join("".join(g) for g in grid))
# d11 trend
try:
    dl=[l.split() for l in open("/memory/d11.log")]
    vals=[float(v) for _,v in dl]
    n=len(vals)
    print("d11 first10", [round(v,3) for v in vals[:10]])
    print("d11 last10", [round(v,3) for v in vals[-10:]])
    print("d11 mean", round(sum(vals)/n,3), "min",round(min(vals),3),"max",round(max(vals),3))
except Exception as e: print(e)
