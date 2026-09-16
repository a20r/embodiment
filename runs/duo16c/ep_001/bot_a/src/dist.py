import os, select, time, math
def readp(p, timeout=0.15):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    d = os.read(fd, 65536) if r else b''
    os.close(fd); return d.decode().strip()
samples=[]
t0=time.time()
while time.time()-t0<6:
    b=readp('d4'); d9=readp('d9')
    try: samples.append((time.time()-t0, float(b), int(d9)))
    except Exception: pass
    time.sleep(0.15)
# fit: db/dt vs sin(b)
import statistics
pts=[(t,((b+180)%360)-180, s) for t,b,s in samples]
# velocity
v=(pts[-1][2]-pts[0][2])/(pts[-1][0]-pts[0][0])
print("v =",round(v,2),"units/s")
Ds=[]
for i in range(1,len(pts)):
    dt=pts[i][0]-pts[i-1][0]; db=math.radians(pts[i][1]-pts[i-1][1])
    bmid=math.radians((pts[i][1]+pts[i-1][1])/2)
    if abs(dt)>0 and abs(math.sin(bmid))>0.05:
        if abs(db)>1e-9: Ds.append(v*math.sin(bmid)*dt/db)
good=[d for d in Ds if 0<abs(d)<100000]
if good:
    print("D estimates: n=%d median=%.0f mean=%.0f" % (len(good), statistics.median(good), sum(good)/len(good)))
    print("first/last:", [round(d) for d in good[::10]])
else:
    print("no D estimates (bearing too small?)")
