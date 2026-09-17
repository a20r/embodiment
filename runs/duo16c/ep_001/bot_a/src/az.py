import os, select, math
fd = os.open('/dev/robot/d2', os.O_RDONLY)
r,_,_ = select.select([fd],[],[],3)
data = os.read(fd, 1<<22).decode().strip()
os.close(fd)
pts = [tuple(map(float,p.split(','))) for p in data.split(';') if p]
# azimuth histogram of min range
NB=36
rmin=[9e9]*NB
for x,y,z in pts:
    r_=math.hypot(x,y)
    az=math.degrees(math.atan2(y,x))
    b=int((az+180)//10)%36
    if r_<rmin[b]: rmin[b]=r_
print("azimuth sector (deg center) -> min range")
for b in range(NB):
    az=b*10-175
    print(f"{az:6d}: {rmin[b]:.2f}")
# elevation stats
els=[math.degrees(math.atan2(z, math.hypot(x,y))) for x,y,z in pts]
print("elev min/max: %.1f %.1f" % (min(els), max(els)))
