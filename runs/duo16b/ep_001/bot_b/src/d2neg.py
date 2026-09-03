import re
s=open('/memory/d2static.txt').read().replace('\n',';')
pts=[]
for x in s.split(';'):
    m=re.findall(r'-?\d*\.?\d+',x)
    if len(m)==3:
        try: pts.append(tuple(map(float,m)))
        except: pass
print("pts:",len(pts))
neg=[p for p in pts if p[0]<0]
print("negative ranges:",len(neg), neg[:10])
c0=[p[0] for p in pts]; c1=[p[1] for p in pts]; c2=[p[2] for p in pts]
print("c0 range:",min(c0),max(c0))
print("c1 range:",min(c1),max(c1))
print("c2 range:",min(c2),max(c2))
# look at the neighborhood of a negative
if neg:
    i=pts.index(neg[0])
    print("context:",pts[max(0,i-4):i+5])
