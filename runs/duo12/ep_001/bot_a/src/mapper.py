import json,math,sys
CELL=0.15
occ={}; free={}
for fn in sys.argv[1:]:
    try: f=open(fn)
    except: continue
    for l in f:
        try:d=json.loads(l)
        except:continue
        x,y,h=d["x"],d["y"],d["h"]; r=d["r"]
        for i in range(16):
            if r[i]<0: continue
            a=math.radians((h+22.5*i)%360)
            # free cells along ray
            steps=int(r[i]/CELL)
            for s in range(steps):
                fx=x+s*CELL*math.cos(a); fy=y+s*CELL*math.sin(a)
                free[(round(fx/CELL),round(fy/CELL))]=free.get((round(fx/CELL),round(fy/CELL)),0)+1
            if r[i]<2.5:
                ox=x+r[i]*math.cos(a); oy=y+r[i]*math.sin(a)
                occ[(round(ox/CELL),round(oy/CELL))]=occ.get((round(ox/CELL),round(oy/CELL)),0)+1
keys=set(occ)|set(free)
xs=[k[0] for k in keys]; ys=[k[1] for k in keys]
x0,x1=min(xs),max(xs); y0,y1=min(ys),max(ys)
print(f"extent x {x0*CELL:.1f}..{x1*CELL:.1f} y {y0*CELL:.1f}..{y1*CELL:.1f}")
for gy in range(y1,y0-1,-1):
    row=""
    for gx in range(x0,x1+1):
        o=occ.get((gx,gy),0); f=free.get((gx,gy),0)
        if o>=2 and o>f*0.3: row+="#"
        elif f>0: row+="."
        else: row+=" "
    print(row)
