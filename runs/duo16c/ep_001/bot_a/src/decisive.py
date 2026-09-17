import os, select, time, math, threading
def readp(p, timeout=0.2):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    data = os.read(fd, 65536) if r else b''
    os.close(fd)
    return data.decode().strip()
def w(p, s):
    fd = os.open(f'/dev/robot/{p}', os.O_WRONLY)
    os.write(fd, (s+'\n').encode()); os.close(fd)

def obj():
    c = readp('d2', 2.0)
    pts=[tuple(map(float,p.split(','))) for p in c.split(';') if p]
    far=[(round(math.hypot(x,y),2), round(math.degrees(math.atan2(y,x)))) for x,y,z in pts if math.hypot(x,y)>0.55]
    far.sort()
    return far[:3]

w('d1','0'); w('d7','0')
time.sleep(1)
# T1: baseline heading
t0=time.time(); h1=[float(readp('d4') or 0) for _ in range(30)] 
time.sleep(0.0)
# T2: d7=1 sustained 5s
t0=time.time()
while time.time()-t0<5: w('d7','1'); time.sleep(0.05)
h2=[float(readp('d4') or 0)]
w('d7','0')
time.sleep(0.5)
h3=[float(readp('d4') or 0)]
print("T1 heading:", h1[::5])
print("T2 after d7=1:", h2, " after stop:", h3)
print("object:", obj())
# T3: d1=0.3 forward 3s
t0=time.time()
d9a = readp('d9')
while time.time()-t0<3: w('d1','0.3'); time.sleep(0.05)
d9b = readp('d9'); w('d1','0')
print("T3 d9 before/after d1=0.3:", d9a, d9b)
time.sleep(0.5)
print("object after T3:", obj())
# T4: d1=-0.5 reverse 3s
t0=time.time()
d9a = readp('d9')
while time.time()-t0<3: w('d1','-0.5'); time.sleep(0.05)
d9b = readp('d9'); w('d1','0')
print("T4 d9 before/after d1=-0.5:", d9a, d9b)
time.sleep(0.5)
print("object after T4:", obj())
w('d7','0'); w('d1','0')
