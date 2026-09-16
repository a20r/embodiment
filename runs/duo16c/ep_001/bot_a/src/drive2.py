import os, select, time, math
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

def wall():
    data = readp('d2', 2.0)
    pts=[tuple(map(float,p.split(','))) for p in data.split(';') if p]
    far=[(math.hypot(x,y), math.degrees(math.atan2(y,x))) for x,y,z in pts if math.hypot(x,y)>1.0]
    if not far: return None
    far.sort()
    return far[0]

print("start wall:", wall(), flush=True)
tests = [
 ('d1','0.3'), ('d7','0.3'),
 ('d1','0.3,0.3'), ('d1','0.3 0.3'),
 ('d7','0.3,0.3'), ('d7','0.3 0.3'),
 ('d1','{"v":0.3,"w":0.0}'), ('d1','v=0.3'),
 ('d7','{"w":0.5}'), ('d7','w=0.5'),
]
for p,s in tests:
    t0=time.time()
    while time.time()-t0<1.5:
        w(p,s); time.sleep(0.1)
    print(f"after {p}<-{s!r}: wall={wall()}", flush=True)
w('d1','0'); w('d7','0')
print("end wall:", wall(), flush=True)
