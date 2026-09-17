import os, select, time
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
        rows.append((readp('d4'), readp('d6'), readp('d9')))
        time.sleep(0.3)
    print(f"{label} d4:", [r[0] for r in rows])
    print(f"{label} d6:", [r[1] for r in rows])
    print(f"{label} d9:", [r[2] for r in rows], flush=True)

w('d1','0'); w('d7','0'); time.sleep(0.5)
t0=time.time()
while time.time()-t0<3: w('d1','1'); time.sleep(0.05)
w('d1','0')
log(2.5, "after d1=1x3s")
t0=time.time()
while time.time()-t0<3: w('d1','3'); time.sleep(0.05)
w('d1','0')
log(2.5, "after d1=3x3s")
t0=time.time()
while time.time()-t0<3: w('d7','10'); time.sleep(0.05)
w('d7','0')
log(2.5, "after d7=10x3s")
