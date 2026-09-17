import os, select, math
fd = os.open('/dev/robot/d2', os.O_RDONLY)
r,_,_ = select.select([fd],[],[],3)
data = os.read(fd, 1<<22).decode().strip()
os.close(fd)
pts = [tuple(map(float,p.split(','))) for p in data.split(';') if p]
far = [p for p in pts if math.hypot(p[0],p[1])>0.6]
print("points >0.6m:", len(far), "of", len(pts))
for p in sorted(far, key=lambda p:-math.hypot(p[0],p[1]))[:20]:
    r_=math.hypot(p[0],p[1]); az=math.degrees(math.atan2(p[1],p[0]))
    print("r=%.2f az=%.0f z=%.2f"%(r_,az,p[2]))
# range histogram
import collections
h=collections.Counter(int(math.hypot(p[0],p[1])*10)/10 for p in pts)
print(sorted(h.items()))
