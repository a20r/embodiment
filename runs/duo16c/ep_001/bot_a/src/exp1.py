import os, select, time, math
def readp(p, timeout=0.15):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    d = os.read(fd, 1<<22) if r else b''
    os.close(fd); return d.decode().strip()
def w(p,s):
    fd=os.open(f'/dev/robot/{p}',os.O_WRONLY); os.write(fd,(s+'\n').encode()); os.close(fd)
def fl():
    try:
        parts=dict(kv.split('=') for kv in readp('d3').split() if '=' in kv)
        return f"g={parts.get('goal')} h={parts.get('here')}"
    except Exception: return "?"
def follower():
    c=readp('d2',1.5)
    out=[]
    for p in c.split(';'):
        if not p: continue
        x,y,z=map(float,p.split(','))
        r=math.hypot(x,y)
        if r>0.55: out.append((round(r,2), round(math.degrees(math.atan2(y,x)))))
    out.sort()
    return out[:4]
w('d1','0'); w('d7','0')
t0=time.time()
while time.time()-t0<55:
    tx = int(time.time()-t0)%4==0
    if tx:
        for msg in ("R1 POS 0 0", "HELLO", "1"):
            w('d8', msg)
    print(f"t={time.time()-t0:.0f} d4={readp('d4')} {fl()} d6={readp('d6')} d9={readp('d9')} fol={follower()}", flush=True)
    time.sleep(4)
