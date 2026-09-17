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

def log(dur, label):
    t0=time.time(); rows=[]
    while time.time()-t0<dur:
        rows.append((float(readp('d4') or -999), readp('d6'), readp('d9'), readp('d11')))
        time.sleep(0.3)
    print(f"{label} d4:", [r[0] for r in rows])
    print(f"{label} d6:", [r[1] for r in rows])
    print(f"{label} d9:", [r[2] for r in rows], flush=True)

w('d1','0'); w('d7','0'); time.sleep(1)
log(2.0, "A idle   ")
t0=time.time()
while time.time()-t0<4: w('d1','0.5'); time.sleep(0.05)
w('d1','0')
log(4.0, "B d1=0.5 ")
time.sleep(1)
t0=time.time()
while time.time()-t0<4: w('d7','0.5'); time.sleep(0.05)
w('d7','0')
log(4.0, "D d7=0.5 ")
time.sleep(1)
log(2.0, "E idle   ")
