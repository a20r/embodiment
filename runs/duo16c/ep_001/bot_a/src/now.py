import os, select, time, math
def readp(p, timeout=0.3):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    data = os.read(fd, 65536) if r else b''
    os.close(fd)
    return data.decode().strip()
for i in range(20):
    print(f"t={i*0.25:.2f} d4={readp('d4'):>6} d5={readp('d5'):>4} d6={readp('d6'):>4} d9={readp('d9'):>4} d11={readp('d11')} d0={readp('d0')}", flush=True)
    time.sleep(0.25)
c = readp('d2',2.0)
pts=[tuple(map(float,p.split(','))) for p in c.split(';') if p]
far=[(math.hypot(x,y), math.degrees(math.atan2(y,x))) for x,y,z in pts if math.hypot(x,y)>0.6]
far.sort()
print("object nearest:", [f"({r:.2f},{a:.0f})" for r,a in far[:5]])
print("az span:", min(a for _,a in far), max(a for _,a in far))
