import re
s=open('/memory/d2static.txt').read().replace('\n',';')
pts=[]
for x in s.split(';'):
    m=re.findall(r'-?\d*\.?\d+',x)
    if len(m)==3:
        try: pts.append(tuple(map(float,m)))
        except: pass
def hist(v):
    h={}
    for x in v:
        b=round(x//0.1*0.1,1); h[b]=h.get(b,0)+1
    return dict(sorted(h.items()))
pos=[abs(p[0]) for p in pts if p[0]>=0]; neg=[abs(p[0]) for p in pts if p[0]<0]
print("POS |c0|:",hist(pos))
print("NEG |c0|:",hist(neg))
# rows: check c1 structure within pos
posp=[p for p in pts if p[0]>=0]
rows={}
for r,c1,c2 in posp: rows.setdefault(round(c1,2),[]).append(abs(r))
ks=sorted(rows)[:25]
print("pos rows (c1: n, meanr):", [(k,len(rows[k]),round(sum(rows[k])/len(rows[k]),2)) for k in ks])
