import os,select,time,json
D='/dev/robot/'
def read(p,timeout=0.2):
    fd=os.open(D+p,os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,9999999).decode().strip()
        except: out=''
    os.close(fd); return out
def fl(x,d=0.0):
    try: return float(x)
    except: return d
time.sleep(3)  # settle
pts=[]
for p in read('d2',0.5).split(';'):
    p=p.strip()
    if not p: continue
    try: pts.append(tuple(map(float,p.split(','))))
    except: pass
H=fl(read('d4'))
print(f"heading={H} npts={len(pts)}")
tall=sorted([(r,e,a) for r,e,a in pts if e>0.35 and r>0.05])
far=sorted([(r,e,a) for r,e,a in pts if r>0.95 and 0.02<r])
print("TALL (r,e,az) sorted by r:")
for t in tall[:12]: print(f"  {t[0]:.2f} {t[1]:.2f} {t[2]:.2f}")
print("  ...")
for t in tall[-12:]: print(f"  {t[0]:.2f} {t[1]:.2f} {t[2]:.2f}")
print(f"FAR n={len(far)}:")
for t in far[:10]: print(f"  {t[0]:.2f} {t[1]:.2f} {t[2]:.2f}")
# az-structure of tall: median r per az bin
import statistics
bins={}
for r,e,a in tall:
    b=round(a,1); bins.setdefault(b,[]).append(r)
print("TALL az-bin medians:")
for b in sorted(bins): print(f"  az={b:+.1f} n={len(bins[b])} rmed={statistics.median(bins[b]):.2f}")
