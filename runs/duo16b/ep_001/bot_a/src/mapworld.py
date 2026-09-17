import os, select, time, math
D='/dev/robot/'
def read(p, timeout=0.3):
    fd=os.open(D+p, os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,2000000).decode().strip()
        except: out=''
    os.close(fd); return out
def fl(x,d=0.0):
    try: return float(x)
    except: return d

def scan():
    s=read('d2'); pts=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            x,z,y=map(float,p.split(',')); pts.append((x,y,z))
        except: pass
    return pts

logf=open('/memory/trail.log','a',buffering=1)
H=fl(read('d4'))
logf.write(f"=== MAPWORLD h={H:.0f}\n")
# merge several captures
allpts=[]
for k in range(4):
    allpts += scan()
    time.sleep(0.3)
# transform to world frame (X=cos(h) fwd, Y=sin(h))
h=math.radians(H)
W=[]
for x,y,z in allpts:
    Xw = x*math.cos(h) - y*math.sin(h)
    Yw = x*math.sin(h) + y*math.cos(h)
    W.append((Xw,Yw,z))
# print z-histogram
from collections import Counter
cz=Counter(round(z,1) for x,y,z in W)
logf.write(f"MAP z_hist={sorted(cz.items())}\n")
# structure: points with z > -0.5 (above/near sensor level), x>0.15 (not self), |Xw|<3
objs=[(Xw,Yw,z) for Xw,Yw,z in W if z>-0.5 and 0.15<x*1 if True]
# simpler: report clusters of tall stuff: z > 0.2 above sensor
tall=[(round(Xw,1),round(Yw,1)) for Xw,Yw,z in W if z>0.3]
ct=Counter(tall)
logf.write(f"MAP tall(z>0.3) top={ct.most_common(15)} n={len(tall)}\n")
# ground level points: z < -0.8: their Xw,Yw histogram
gnd=[(round(Xw*2)/2,round(Yw*2)/2) for Xw,Yw,z in W if z<-0.8]
cg=Counter(gnd)
logf.write(f"MAP ground bins={cg.most_common(20)} n={len(gnd)}\n")
# mid-level (sensor height) objects excluding self: z in [-0.4,0.3], x>0.2
mid=[(round(Xw*2)/2,round(Yw*2)/2) for Xw,Yw,z,x0 in [(Xw,Yw,z,1) for Xw,Yw,z in W] if -0.4<=z<=0.3]
cm=Counter(mid)
logf.write(f"MAP mid bins={cm.most_common(20)} n={len(mid)}\n")
print("done")
