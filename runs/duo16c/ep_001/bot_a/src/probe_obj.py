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
        if r>0.55:
            out.append((round(r,2), round(math.degrees(math.atan2(y,x)))))
    out.sort()
    return out[:6]
w('d1','0'); w('d7','0'); time.sleep(0.5)
print("start:", scan(), "d4=",readp('d4'), "d3=",readp('d3'), flush=True)
# creep forward 1s at d1=1
t0=time.time()
while time.time()-t0<1.0: w('d1','1'); time.sleep(0.05)
w('d1','0')
print("after fwd1s:", scan(), "d4=",readp('d4'), "d3=",readp('d3'), flush=True)
time.sleep(1.0)
print("settled:", scan(), "d4=",readp('d4'), "d3=",readp('d3'), flush=True)
# creep again
t0=time.time()
while time.time()-t0<1.0: w('d1','1'); time.sleep(0.05)
w('d1','0'); time.sleep(1)
print("after fwd2:", scan(), "d4=",readp('d4'), "d3=",readp('d3'), flush=True)
time.sleep(2)
print("settled2 (did it move?):", scan(), "d4=",readp('d4'), "d3=",readp('d3'), flush=True)
