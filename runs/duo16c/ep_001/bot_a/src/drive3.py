import os, select, time, math, threading
def readp(p, timeout=0.5):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    data = os.read(fd, 1<<22) if r else b''
    os.close(fd)
    return data.decode().strip()
def w(p, s):
    fd = os.open(f'/dev/robot/{p}', os.O_WRONLY)
    os.write(fd, (s+'\n').encode())
    os.close(fd)

running=[True]
def driver(port, val):
    while running[0]:
        w(port, val)
        time.sleep(0.05)

def snapshot(label):
    d4 = readp('d4')
    data = readp('d2', 2.0)
    pts=[tuple(map(float,p.split(','))) for p in data.split(';') if p]
    far=[(math.hypot(x,y), math.degrees(math.atan2(y,x))) for x,y,z in pts if math.hypot(x,y)>1.0]
    far.sort()
    wall = far[0] if far else None
    print(f"{label}: d4={d4} wall={wall}", flush=True)

snapshot("start")
# sustained d1=0.3
th=threading.Thread(target=driver,args=('d1','0.3'),daemon=True); th.start()
t0=time.time()
while time.time()-t0<3.0:
    time.sleep(0.75); snapshot("d1=0.3")
running[0]=False; th.join()
w('d1','0')
snapshot("after d1 test")
# sustained d7=0.5
running[0]=True
th=threading.Thread(target=driver,args=('d7','0.5'),daemon=True); th.start()
t0=time.time()
while time.time()-t0<3.0:
    time.sleep(0.75); snapshot("d7=0.5")
running[0]=False; th.join()
w('d7','0')
snapshot("end")
