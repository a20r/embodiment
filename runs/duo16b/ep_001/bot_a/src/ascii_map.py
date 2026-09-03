import os, select, time, math
D='/dev/robot/'
def read(p, timeout=0.3):
    fd=os.open(D+p, os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,2000000).decode().strip()
        except: out=''
    os.close(fd); return out

def scan():
    s=read('d2'); pts=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            x,z,y=map(float,p.split(',')); pts.append((x,y,z))
        except: pass
    return pts

allpts=[]
for k in range(5):
    allpts += scan()
    time.sleep(0.25)
print("npts:", len(allpts))
H=0.0  # body frame
# grid 0.25m cells from -1.5..1.5
G={}
for x,y,z in allpts:
    gx=int(round(x*2)); gy=int(round(y*2))
    key=(gx,gy)
    G.setdefault(key, []).append(z)
print("   " + "".join(f"{gy:+d} " for gy in range(-6,7)))
for gx in range(6,-7,-1):
    row=f"{gx:+d} "
    for gy in range(-6,7):
        zs=G.get((gx,gy))
        if not zs: row+=" . "
        else:
            zmax=max(zs); zmin=min(zs); n=len(zs)
            if zmax>0.4: row+=" T "   # tall
            elif zmax>0.05: row+=" h "  # mid-height
            elif zmin<-0.3: row+=" g "  # ground-ish/low
            else: row+=" s "  # sensor level
    print(row)
# detail: for the 4 densest cells, print z-distribution
import collections
cnt={k:len(v) for k,v in G.items()}
top=sorted(cnt.items(), key=lambda kv:-kv[1])[:8]
for (gx,gy),n in top:
    zs=sorted(G[(gx,gy)])
    print(f"cell({gx},{gy}) n={n} z[{zs[0]:.2f}..{zs[-1]:.2f}] quartiles {zs[n//4]:.2f}/{zs[n//2]:.2f}/{zs[3*n//4]:.2f}")
