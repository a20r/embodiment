import os, select, time, math
def readp(p, timeout=0.2):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    d = os.read(fd, 1<<22) if r else b''
    os.close(fd); return d.decode().strip()
def w(p,s):
    fd=os.open(f'/dev/robot/{p}',os.O_WRONLY); os.write(fd,(s+'\n').encode()); os.close(fd)
def scan():
    c=readp('d2',2.0)
    out=[]
    for p in c.split(';'):
        if not p: continue
        x,y,z=map(float,p.split(','))
        r=math.hypot(x,y)
        if r>0.55: out.append((round(r,2), round(math.degrees(math.atan2(y,x)))))
    out.sort(); return out[:8]
def pose(): return f"d4={readp('d4')} d6={readp('d6')} d9={readp('d9')}"
w('d1','0'); w('d7','0'); time.sleep(0.5)
print("t0:", scan(), pose(), flush=True)
t0=time.time()
while time.time()-t0<3: w('d1','3'); time.sleep(0.05)
w('d1','0')
print("after 3s fwd:", scan(), pose(), flush=True)
time.sleep(2)
print("settled:", scan(), pose(), flush=True)
t0=time.time()
while time.time()-t0<3: w('d1','3'); time.sleep(0.05)
w('d1','0'); time.sleep(2)
print("after 3s fwd again:", scan(), pose(), flush=True)
