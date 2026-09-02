import sys, time, math
sys.path.insert(0,'/bot/src')
from robot import R
r=R(); r.motors(0,0); time.sleep(0.4)
rg=r.ranges(); h=r.heading()
print("pose est (-0.2,-0.25) h=",h)
print("rg:",[round(v,2) if v and v>=0 else None for v in rg])
grid={}
CELL=0.05
PX,PY=-0.2,-0.25
for k,val in enumerate(rg):
    if val is None or val<0: continue
    ang=h+k*22.5
    ca=math.cos(math.radians(ang)); sa=math.sin(math.radians(ang))
    steps=max(1,int((val-CELL)/CELL)) if val<2.45 else int(2.45/CELL)
    for s in range(1,steps+1):
        dd=s*CELL
        kk=(int(round((PX+ca*dd)/CELL)),int(round((PY+sa*dd)/CELL)))
        if val<2.45 and s==steps:
            if grid.get(kk,0)!=1: grid[kk]=2
        else:
            if grid.get(kk,0)==0: grid[kk]=1
cg={}
for (i,j),v in grid.items():
    ci,cj=i//2,j//2
    if v==2:
        for di in (-1,0,1):
            for dj in (-1,0,1):
                cg[(ci+di,cj+dj)]=-1
    elif v==1:
        if cg.get((ci,cj),0)==0:
            cg[(ci,cj)]=1
xs=[c[0] for c in cg]; ys=[c[1] for c in cg]
print("coarse extent",min(xs),max(xs),min(ys),max(ys))
sym={-1:'#',0:'?',1:' '}
for j in range(max(ys),min(ys)-1,-1):
    row=""
    for i in range(min(xs),max(xs)+1):
        row+=sym.get(cg.get((i,j),-2),'-')
    print(row)
