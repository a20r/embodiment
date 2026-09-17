import os, select, time
D='/dev/robot/'
def read(p, timeout=0.3):
    fd=os.open(D+p, os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,2000000).decode().strip()
        except: out=''
    os.close(fd); return out

s=read('d2')
pts=[]
for p in s.split(';'):
    p=p.strip()
    if not p: continue
    try:
        r,e,a=map(float,p.split(',')); pts.append((r,e,a))
    except: pass
print("n=",len(pts))
bands = [(-2,-1.0),(-1.0,-0.6),(-0.6,-0.3),(-0.3,-0.1),(-0.1,0.1),(0.1,0.4)]
for lo,hi in bands:
    sel=[r for r,e,a in pts if lo<=e<hi and r>0.02]
    neg=sum(1 for r,e,a in pts if lo<=e<hi)
    if sel:
        sel.sort()
        print(f"elev[{lo},{hi}): n={len(sel)} min={sel[0]:.3f} med={sel[len(sel)//2]:.3f} max={sel[-1]:.3f} (raw={neg})")
    else:
        print(f"elev[{lo},{hi}): empty (raw={neg})")
# azimuth coverage
az=[a for r,e,a in pts]
print("az range:", min(az), max(az))
el=[e for r,e,a in pts]
print("el range:", min(el), max(el))
