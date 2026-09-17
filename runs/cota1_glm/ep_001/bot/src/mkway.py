import csv,math
rows=[]
for r in csv.reader(open('/bot/src/auto8.csv')):
    try: t=float(r[0])
    except: continue
    rows.append((t,float(r[2]),float(r[3]),float(r[4]),float(r[6]),float(r[7])))
CROSS_T=361.8  # crossing t in this run
loop=[r for r in rows if r[0]<=CROSS_T]
print("n=",len(loop),"start",loop[0][1:3],"end",loop[-1][1:3])
# arc-length resample every 2m, then smooth
pts=[(r[1],r[2]) for r in loop]
# smooth positions with moving average
def smooth(p,k=15):
    out=[]
    for i in range(len(p)):
        a=max(0,i-k); b=min(len(p),i+k+1)
        xs=[q[0] for q in p[a:b]]; ys=[q[1] for q in p[a:b]]
        out.append((sum(xs)/len(xs),sum(ys)/len(ys)))
    return out
S=smooth(pts,18)
# resample by arclength
way=[S[0]]; acc=0.0; STEP=2.0
for a,b in zip(S,S[1:]):
    d=math.hypot(b[0]-a[0],b[1]-a[1]); acc+=d
    if acc>=STEP: way.append(b); acc=0.0
print("waypoints:",len(way))
# curvature at each wp via circumradius of triple
def curv(a,b,c):
    A=math.hypot(b[0]-a[0],b[1]-a[1]); B=math.hypot(c[0]-b[0],c[1]-a[1]); C=math.hypot(c[0]-a[0],c[1]-a[1])
    ar=abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/2
    return 4*ar/(A*B*C) if ar>1e-9 else 0.0
cur=[curv(way[i-1],way[i],way[(i+1)%len(way)]) for i in range(len(way))]
# max curvature over +/-2 window (corner-aware)
curm=[]
for i in range(len(way)):
    curm.append(max(cur[(i+d)%len(way)] for d in (-2,-1,0,1,2)))
# speed target: v=sqrt(0.55/cur), cap 1.0, floor 0.35
sp=[min(1.0,max(0.35,math.sqrt(0.55/c))) for c in curm]
with open('/bot/src/waypoints.txt','w') as f:
    for (x,y),v in zip(way,sp):
        f.write(f"{x:.2f} {y:.2f} {v:.2f}\n")
print("first:",way[0],"last:",way[-1])
print("lap length ~",sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(way,way[1:])),"m")
print("speed dist: min",min(sp),"max",max(sp),"avg",sum(sp)/len(sp))
