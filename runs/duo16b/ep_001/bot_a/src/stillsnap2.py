import os,select,time,statistics
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
time.sleep(3)
pts=[]
for p in read('d2',0.5).split(';'):
    p=p.strip()
    if not p: continue
    try: pts.append(tuple(map(float,p.split(','))))
    except: pass
H=fl(read('d4'))
print(f"heading={H} n={len(pts)}")
# all returns with r 0.08..1.2, elev > -0.35 (not deep floor)
mid=[(r,e,a) for r,e,a in pts if 0.08<=r<=1.2 and e>-0.35]
print(f"mid n={len(mid)}")
bins={}
for r,e,a in mid:
    b=round(a*10)/10
    bins.setdefault(b,[]).append((r,e))
for b in sorted(bins):
    v=bins[b]; rs=[x[0] for x in v]; es=[x[1] for x in v]
    if len(v)>=3:
        print(f"az={b:+.1f} n={len(v)} r={min(rs):.2f}-{statistics.median(rs):.2f}-{max(rs):.2f} e={min(es):.2f}..{max(es):.2f}")
# tall-only clusters
tall=[(r,e,a) for r,e,a in pts if e>0.35 and r>0.05]
print(f"\ntall n={len(tall)}")
bins={}
for r,e,a in tall:
    b=round(a*10)/10
    bins.setdefault(b,[]).append((r,e))
for b in sorted(bins):
    v=bins[b]; rs=[x[0] for x in v]; es=[x[1] for x in v]
    print(f"T az={b:+.1f} n={len(v)} r={min(rs):.2f}-{statistics.median(rs):.2f}-{max(rs):.2f} emax={max(es):.2f}")
