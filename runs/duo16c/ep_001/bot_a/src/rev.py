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

def ahead():
    c = readp('d2', 2.0)
    pts=[tuple(map(float,p.split(','))) for p in c.split(';') if p]
    far=[(math.hypot(x,y), math.degrees(math.atan2(y,x))) for x,y,z in pts if math.hypot(x,y)>0.55]
    if not far: return None
    far.sort(); return (round(far[0][0],2), round(far[0][1]))

def phase(label, dur, port, val):
    t0=time.time(); hs=[]; d9s=[]
    th=None
    if val is not None:
        def drv():
            tt=time.time()
            while time.time()-tt<dur: w(port,val); time.sleep(0.05)
        th=threading.Thread(target=drv,daemon=True); th.start()
    while time.time()-t0<dur:
        hs.append(readp('d4')); d9s.append(readp('d9')); time.sleep(0.4)
    if th: th.join()
    print(f"{label}: d4={hs} d9={d9s}", flush=True)

w('d1','0'); w('d7','0'); time.sleep(0.5)
print("ahead:", ahead())
phase("rev d7=-1 3s", 3.0, 'd7', '-1')
print("ahead:", ahead())
phase("idle 2s", 2.0, None, None)
phase("rev d1=-1 3s", 3.0, 'd1', '-1')
print("ahead:", ahead())
phase("d7=+1 3s", 3.0, 'd7', '1')
print("ahead:", ahead())
w('d1','0'); w('d7','0')
