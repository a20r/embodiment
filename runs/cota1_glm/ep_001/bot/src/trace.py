import csv,math,sys
rows=[]
run=0; prevt=-1
for r in csv.reader(open('/bot/src/auto8.csv')):
    try: t=float(r[0])
    except: continue
    if t<prevt: run+=1
    prevt=t
    rows.append((run,float(r[0]),int(r[1]),float(r[2]),float(r[3]),float(r[4]),float(r[6]),r[10]))
runs={}
for row in rows: runs.setdefault(row[0],[]).append(row)
for run in sorted(runs):
    rr=runs[run]
    path=0.0
    for a,b in zip(rr,rr[1:]):
        path+=math.hypot(b[3]-a[3],b[4]-a[4])
    print(f"run{run}: n={len(rr)} t={rr[-1][1]:.0f}s path={path:.1f}m start=({rr[0][3]:.1f},{rr[0][4]:.1f}) end=({rr[-1][3]:.1f},{rr[-1][4]:.1f}) ticks {rr[0][2]}..{rr[-1][2]}")
# ascii plot of last run
rr=runs[sorted(runs)[-1]]
xs=[r[3] for r in rr]; ys=[r[4] for r in rr]
minx,maxx,miny,maxy=min(xs),max(xs),min(ys),max(ys)
W=100; H=int(W*(maxy-miny)/((maxx-minx)*2.1))+1
grid=[[' ']*W for _ in range(H)]
def plot(x,y,c):
    i=int((maxy-y)/(maxy-miny)*(H-1)); j=int((x-minx)/(maxx-minx)*(W-1))
    if 0<=i<H and 0<=j<W: grid[i][j]=c
for k in range(0,len(rr),2): plot(rr[k][3],rr[k][4],'.')
for k in range(0,len(rr),40): plot(rr[k][3],rr[k][4],'o')
plot(rr[0][3],rr[0][4],'S'); plot(rr[-1][3],rr[-1][4],'E')
print(f"extent x[{minx:.0f},{maxx:.0f}] y[{miny:.0f},{maxy:.0f}]")
for line in grid: print(''.join(line))
