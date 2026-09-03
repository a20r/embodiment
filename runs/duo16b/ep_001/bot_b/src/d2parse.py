import statistics as st
buf=open('/memory/d2frame.txt').read()
pts=[tuple(map(float,p.split(','))) for p in [p for ln in buf.split('\n') for p in ln.split(';')] if p and ',' in p]
c0=[p[0] for p in pts]; c1=[p[1] for p in pts]; c2=[p[2] for p in pts]
print("n=",len(pts))
for i,c in enumerate([c0,c1,c2]):
    print(f"c{i}: min={min(c):.3f} max={max(c):.3f} mean={st.mean(c):.3f}")
# check raster structure: unique c1 values in order of appearance
seen=[]
for p in pts:
    if not seen or abs(seen[-1][0]-p[1])>1e-6:
        seen.append((p[1],p[2]))
print("c1 transitions (first 30):", [(round(a,3),round(b,3)) for a,b in seen[:30]])
print("num distinct c1:", len(set(round(x,3) for x in c1)), "distinct c2:", len(set(round(x,3) for x in c2)))
