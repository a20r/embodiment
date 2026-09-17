import os, select, time, math, threading
def readp(p, timeout=0.2):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    data = os.read(fd, 1<<22) if r else b''
    os.close(fd)
    return data.decode().strip()
def w(p, s):
    fd = os.open(f'/dev/robot/{p}', os.O_WRONLY)
    os.write(fd, (s+'\n').encode()); os.close(fd)
def farpts():
    c = readp('d2', 2.0)
    pts=[tuple(map(float,p.split(','))) for p in c.split(';') if p]
    return sorted((round(math.hypot(x,y),2), round(math.degrees(math.atan2(y,x)))) for x,y,z in pts if math.hypot(x,y)>0.55)

w('d1','0'); w('d7','0'); time.sleep(0.5)
print("before:", farpts()[:4], "d9=",readp('d9'), "d4=",readp('d4'), flush=True)
# reverse sustained 4s
t0=time.time()
while time.time()-t0<4: w('d1','-0.5'); time.sleep(0.05)
w('d1','0')
time.sleep(0.5)
print("after rev:", farpts()[:4], "d9=",readp('d9'), "d4=",readp('d4'), flush=True)
time.sleep(2)
print("settled:", farpts()[:4], "d9=",readp('d9'), "d6=",readp('d6'), "d4=",readp('d4'), flush=True)
