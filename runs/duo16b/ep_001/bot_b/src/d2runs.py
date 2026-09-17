import re
s=open('/memory/d2static.txt').read().replace('\n',';')
pts=[]
for x in s.split(';'):
    m=re.findall(r'-?\d*\.?\d+',x)
    if len(m)==3:
        try: pts.append(tuple(map(float,m)))
        except: pass
# runs of sign
runs=[]
cur=pts[0][0]>=0; start=0
for i,p in enumerate(pts):
    sg=p[0]>=0
    if sg!=cur:
        runs.append((cur,i-start)); cur=sg; start=i
runs.append((cur,len(pts)-start))
print("n runs:",len(runs),"first 15:",runs[:15])
import statistics as st
pos=[p for p in pts if p[0]>=0]; neg=[p for p in pts if p[0]<0]
for name,g in [("pos",pos),("neg",neg)]:
    print(name, "n=",len(g), "c0:", round(min(x[0] for x in g),3), round(max(x[0] for x in g),3),
          "c1:", round(min(x[1] for x in g),3), round(max(x[1] for x in g),3),
          "c2:", round(min(x[2] for x in g),3), round(max(x[2] for x in g),3))
# histogram of c1 in 0.1 bins for each
def hist(g,idx):
    h={}
    for x in g: 
        b=round(x[idx]//0.1*0.1,1); h[b]=h.get(b,0)+1
    return dict(sorted(h.items()))
print("pos c1 hist:", hist(pos,1))
print("neg c1 hist:", hist(neg,1))
print("pos c0 hist:", hist(pos,0))
print("neg c0 hist:", hist(neg,0))
