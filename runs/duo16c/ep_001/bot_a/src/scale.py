import os,select,math,time
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
out=open('/memory/scaletest.txt','a',buffering=1)
A=scan()
d9a=float(readp('d9'))
w('d1','6'); time.sleep(5.0); w('d1','0'); time.sleep(0.8)
d9b=float(readp('d9'))
B=scan()
dd9=d9b-d9a
print(f'd9 {d9a:.0f}->{d9b:.0f} delta={dd9:.0f}',file=out)
Apts=[(x,y) for x,y,z in A if math.hypot(x,y)>0.25]
Bpts=[(x,y) for x,y,z in B if math.hypot(x,y)>0.25]
def medNN(pts,Q):
    ds=[]
    for p in pts:
        best=9
        for q in Q:
            d=(p[0]-q[0])**2+(p[1]-q[1])**2
            if d<best: best=d
        ds.append(math.sqrt(best))
    ds.sort(); return ds[len(ds)//2]
# grid search dx (fwd) and dy (left) minimizing median NN of B onto A
best=(9,0,0)
for dx in [i*0.1 for i in range(-25,26)]:
    for dy in [-0.2,0,0.2]:
        shifted=[(x-dx,y-dy) for x,y in Bpts[::2]]
        m=medNN(shifted,Apts)
        if m<best[0]: best=(m,dx,dy)
print(f'best alignment: medianNN={best[0]:.3f} at dx={best[1]:.1f} dy={best[2]:.1f} (B shifted by -dx,-dy; robot moved +dx fwd,+dy left)',file=out)
print(f'no-shift medianNN={medNN(Bpts[::2],Apts):.3f}',file=out)
print(f'SCALE: {dd9:.0f} d9-units = {best[1]:.2f} m  -> {dd9/best[1] if best[1] else 0:.1f} units/m',file=out)
out.close()
