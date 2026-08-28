import sys, math, time
from collections import defaultdict

# parse sensors.log: lines "ts dN payload"
t0 = float(sys.argv[1]) if len(sys.argv)>1 else 0
fn='/bot/src/sensors.log'
events=[]
for line in open(fn):
    try:
        p=line.split()
        ts=float(p[0]); port=p[1]; payload=p[2]
    except Exception: continue
    if ts<t0: continue
    events.append((ts,port,payload))
events.sort(key=lambda e:e[0])

x,y=0.0,0.0
h=None
last_scan=None; last_ts=None
occ=defaultdict(int); free=defaultdict(int)
traj=[]
CELL=0.15
def cell(px,py): return (int(round(px/CELL)), int(round(py/CELL)))
for ts,port,payload in events:
    if port=='d4':
        try: h=float(payload)
        except: pass
    elif port=='d2' and h is not None:
        try: s=[float(v) for v in payload.split(',')]
        except: continue
        if last_scan is not None and ts-last_ts<1.0:
            dt=ts-last_ts
            # estimate signed v along heading via least squares over valid beams
            num=0.0; den=0.0
            for i in range(16):
                a,b=last_scan[i],s[i]
                if a<0 or b<0 or a>1.5 or b>1.5: continue
                dd=b-a
                if abs(dd)>0.12: continue
                c=math.cos(math.radians(h+22.5*i-h))  # beam angle rel motion dir (=heading)
                c=math.cos(math.radians(22.5*i))
                num+= -dd*c; den+= c*c
            v = num/den if den>1e-6 else 0.0
            if abs(v) < 0.005*dt: v=0
            x+=v*math.cos(math.radians(h)); y+=v*math.sin(math.radians(h))
        last_scan=s; last_ts=ts
        traj.append((x,y))
        for i in range(16):
            d=s[i]
            if d<0: continue
            ang=math.radians(h+22.5*i)
            steps=int(d/CELL)
            for k in range(1,min(steps,12)):
                px,py=x+k*CELL*math.cos(ang), y+k*CELL*math.sin(ang)
                free[cell(px,py)]+=1
            if d<2.0:
                px,py=x+d*math.cos(ang), y+d*math.sin(ang)
                occ[cell(px,py)]+=1
# render
cells=set(occ)|set(free)
xs=[c[0] for c in cells]; ys=[c[1] for c in cells]
minx,maxx=min(xs),max(xs); miny,maxy=min(ys),max(ys)
tcells=set(cell(px,py) for px,py in traj)
cur=cell(x,y)
print(f"pose=({x:.2f},{y:.2f}) h={h} extent x[{minx*CELL:.1f},{maxx*CELL:.1f}] y[{miny*CELL:.1f},{maxy*CELL:.1f}]")
for gy in range(maxy,miny-1,-1):
    row=''
    for gx in range(minx,maxx+1):
        c=(gx,gy)
        o=occ.get(c,0); f=free.get(c,0)
        if c==cur: ch='R'
        elif c in tcells: ch='.'
        elif o>=2 and o>f*0.3: ch='#'
        elif f>2: ch=' '
        else: ch='~'
        row+=ch
    print(row)
