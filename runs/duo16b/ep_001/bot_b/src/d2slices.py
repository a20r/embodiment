import re
s=open('/memory/d2static.txt').read().replace('\n',';')
pts=[]
for x in s.split(';'):
    m=re.findall(r'-?\d*\.?\d+',x)
    if len(m)==3:
        try: pts.append(tuple(map(float,m)))
        except: pass
# find boundaries
def runs():
    out=[];cur=pts[0][0]>=0;start=0
    for i,p in enumerate(pts):
        sg=p[0]>=0
        if sg!=cur: out.append((cur,start,i)); cur=sg; start=i
    out.append((cur,start,len(pts)))
    return out
R=runs()
print("runs:",len(R))
sg,a,b=R[2]
print("POS block", a, b, "sample:")
for p in pts[a+600:a+640]: print("  ",p)
sg,a,b=R[3]
print("NEG block", a, b, "sample:")
for p in pts[a+600:a+640]: print("  ",p)
