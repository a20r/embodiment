import os,select,time,math
D='/dev/robot/'
def read(p,timeout=0.3):
    fd=os.open(D+p,os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,9999999).decode().strip()
        except: out=''
    os.close(fd); return out
def tx(m):
    try:
        fd=os.open(D+'d8',os.O_WRONLY|os.O_NONBLOCK); os.write(fd,m.encode()); os.close(fd)
    except Exception as e: print('TXerr',e)
def fl(x,d=0.0):
    try: return float(x)
    except: return d
# 1) motion check
r0,l0=fl(read('d6')),fl(read('d9')); time.sleep(1.5)
r1,l1=fl(read('d6')),fl(read('d9'))
print(f"MOTION dr={r1-r0} dl={l1-l0} (should be ~0 if motors off)")
H=fl(read('d4'))
print(f"heading={H}")
# 2) lidar tall-object analysis
pts=[]
for p in read('d2').split(';'):
    p=p.strip()
    if not p: continue
    try: pts.append(tuple(map(float,p.split(','))))
    except: pass
print(f"npts={len(pts)}")
tall=[(r,e,a) for r,e,a in pts if e>0.4 and 0.05<r<3.0]
print(f"tall(e>0.4)={len(tall)}")
# cluster by azimuth
tall.sort(key=lambda t:t[2])
clusters=[]
for t in tall:
    if clusters and abs(t[2]-clusters[-1][-1][2])<0.15: clusters[-1].append(t)
    else: clusters.append([t])
for c in clusters:
    rs=sorted(x[0] for x in c); azs=[x[2] for x in c]
    print(f"  CLUSTER n={len(c)} range_med={rs[len(rs)//2]:.2f} az={azs[0]:.2f}..{azs[-1]:.2f} bearing={H+azs[len(azs)//2]*57.3:.0f}")
# profile of second field
es=[e for r,e,a in pts if 0.02<r]
if es:
    es.sort(); print(f"f2(elev) min={es[0]:.2f} max={es[-1]:.2f}")
# 3) radio burst
print("--- radio burst ---")
for m in ["ROBOT1 TO ROBOT2: DO YOU COPY?\n","STATUS? SEND 'ACK <your d11>'\n","I SEE OBJECT AT RANGE 0.48. IS THAT YOU?\n"]:
    tx(m); time.sleep(0.3)
time.sleep(6)
print(open('/memory/radio_log.txt').read()[-400:])
