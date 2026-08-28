import math, bisect
tr=[]
for line in open("/memory/trail2.csv"):
    p=line.split(",")
    try: tr.append((float(p[0]),float(p[1]),float(p[2]),float(p[3])))
    except: pass
ts=[r[0] for r in tr]
S=[]
for line in open("/memory/radio.csv"):
    p=line.split(",")
    t,x,y,v=float(p[0]),float(p[1]),float(p[2]),float(p[3])
    i=bisect.bisect(ts,t)
    if i==0 or i>=len(ts): continue
    h=tr[i-1][3]
    S.append((x,y,h,v))
print(len(S),"samples")
best=None
for bx in [x/2 for x in range(-30,6)]:
    for by in [y/2 for y in range(-14,12)]:
        # fit scale c minimizing sum (v - c*cos(theta-h))^2
        num=0;den=0
        rows=[]
        for x,y,h,v in S:
            th=math.degrees(math.atan2(by-y,bx-x))
            c=math.cos(math.radians(th-h))
            rows.append((c,v))
        num=sum(c*v for c,v in rows); den=sum(c*c for c,v in rows)+1e-9
        k=num/den
        err=sum((v-k*c)**2 for c,v in rows)
        if best is None or err<best[0]: best=(err,bx,by,k)
print("best", best)
