import os,select,math,time,collections
def readp(p,timeout=0.3):
    fd=os.open(f'/dev/robot/{p}',os.O_RDONLY)
    r,_,_=select.select([fd],[],[],timeout)
    d=os.read(fd,1<<22) if r else b''
    os.close(fd); return d.decode().strip()
def scan():
    for _ in range(6):
        d=readp('d2',1.5)
        if len(d)>10: return [tuple(map(float,p.split(','))) for p in d.split(';') if p]
        time.sleep(0.2)
    return []
def w(p,v):
    fd=os.open(f'/dev/robot/{p}',os.O_WRONLY); os.write(fd,(v+'\n').encode()); os.close(fd)
def cluster(pts):
    cell={}
    for x,y,z in pts:
        rr=math.hypot(x,y)
        if rr<0.25: continue
        cell.setdefault((round(x/0.15),round(y/0.15)),[]).append((x,y,z))
    seen=set(); comps=[]
    for c in list(cell):
        if c in seen: continue
        stack=[c]; comp=[]
        while stack:
            cur=stack.pop()
            if cur in seen or cur not in cell: continue
            seen.add(cur); comp+=cell[cur]
            cx,cy=cur
            for dx in(-1,0,1):
                for dy in(-1,0,1): stack.append((cx+dx,cy+dy))
        if len(comp)>10: comps.append(comp)
    comps.sort(key=len,reverse=True)
    return comps
A=scan()
print('A npts',len(A),file=open('/memory/movetest.txt','a',buffering=1))
w('d1','3'); time.sleep(3.0); w('d1','0')
time.sleep(0.8)
B=scan()
print('B npts',len(B),file=open('/memory/movetest.txt','a',buffering=1))
# displacement of B points vs nearest A point
def nn(p,Q):
    best=9
    for q in Q:
        d=math.hypot(p[0]-q[0],p[1]-q[1])
        if d<best: best=d
    return best
d9=readp('d9'); d6=readp('d6'); d4=readp('d4')
print(f'after drive d9={d9} d6={d6} d4={d4}',file=open('/memory/movetest.txt','a',buffering=1))
for name,Q in (('A',A),):
    pass
cA=cluster(A); cB=cluster(B)
out=open('/memory/movetest.txt','a',buffering=1)
print('A comps:',[(len(c),round(min(math.hypot(x,y) for x,y,z in c),2),round(max(math.hypot(x,y) for x,y,z in c),2)) for c in cA[:5]],file=out)
print('B comps:',[(len(c),round(min(math.hypot(x,y) for x,y,z in c),2),round(max(math.hypot(x,y) for x,y,z in c),2)) for c in cB[:5]],file=out)
# for each B comp, median NN distance to A points (excluding body)
Aflat=A
for i,c in enumerate(cB[:4]):
    ds=sorted(nn(p,Aflat) for p in c[::3])
    med=ds[len(ds)//2]
    rs=[math.hypot(x,y) for x,y,z in c]
    azs=[math.degrees(math.atan2(y,x))%360 for x,y,z in c]
    print(f'B comp{i}: n={len(c)} medNN_to_A={med:.2f} r=[{min(rs):.2f},{max(rs):.2f}] az=[{min(azs):.0f},{max(azs):.0f}]',file=out)
out.close()
