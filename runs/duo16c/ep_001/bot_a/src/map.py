import os, select, time, math
def readp(p, timeout=1.0):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    data = os.read(fd, 1<<22) if r else b''
    os.close(fd)
    return data.decode().strip()

c1 = readp('d2',2.0); time.sleep(1.0); c2 = readp('d2',2.0)
print("clouds identical?", c1==c2, "len", len(c1), len(c2))

for k,c in (('c1',c1),('c2',c2)):
    pts=[tuple(map(float,p.split(','))) for p in c.split(';') if p]
    far=[(math.hypot(x,y), math.degrees(math.atan2(y,x)), z) for x,y,z in pts if math.hypot(x,y)>0.7]
    far.sort()
    print(f"--- {k}: {len(far)} far pts; az->range profile:")
    # bin by azimuth 5deg, min range
    import collections
    prof=collections.defaultdict(lambda:9e9)
    for r_,az,z in far:
        b=int((az+180)//5)*5-177
        prof[b]=min(prof[b],r_)
    for b in sorted(prof):
        print(f"  az {b:4d}: {prof[b]:.2f}")
