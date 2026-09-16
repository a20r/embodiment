import os, select, time, math, threading, collections
def readp(p, timeout=1.0):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    data = os.read(fd, 1<<22) if r else b''
    os.close(fd)
    return data.decode().strip()
def w(p, s):
    fd = os.open(f'/dev/robot/{p}', os.O_WRONLY)
    os.write(fd, (s+'\n').encode()); os.close(fd)

def obj():
    c = readp('d2', 2.0)
    pts=[tuple(map(float,p.split(','))) for p in c.split(';') if p]
    far=[(math.hypot(x,y), math.degrees(math.atan2(y,x))) for x,y,z in pts if math.hypot(x,y)>0.6]
    if not far: return None
    far.sort()
    r0,a0 = far[0]
    azs=[a for _,a in far]
    return (round(r0,3), round(min(azs),1), round(max(azs),1))

def telem():
    return dict(d0=readp('d0'), d5=readp('d5'), d6=readp('d6'), d9=readp('d9'), d11=readp('d11'), d4=readp('d4'))

print("BASELINE:", obj(), telem(), flush=True)
vals = ['0.5','1','2','5','-2']
for v in vals:
    th=threading.Thread(target=lambda:None,daemon=True)
    t0=time.time()
    while time.time()-t0<2.0:
        w('d1',v); time.sleep(0.05)
    print(f"d1={v}: {obj()} d4={readp('d4')}", flush=True)
w('d1','0')
for v in ['0.5','1','2','-2']:
    t0=time.time()
    while time.time()-t0<2.0:
        w('d7',v); time.sleep(0.05)
    print(f"d7={v}: {obj()} d4={readp('d4')}", flush=True)
w('d7','0')
print("END:", obj(), telem(), flush=True)
