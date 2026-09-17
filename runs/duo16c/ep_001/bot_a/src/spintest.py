import os, select, time, math
def readp(p, timeout=0.2):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    d = os.read(fd, 1<<22) if r else b''
    os.close(fd); return d.decode().strip()
def w(p,s):
    fd=os.open(f'/dev/robot/{p}',os.O_WRONLY); os.write(fd,(s+'\n').encode()); os.close(fd)
def fl():
    t=readp('d3')
    try:
        parts=dict(kv.split('=') for kv in t.split() if '=' in kv)
        return f"g={parts.get('goal')} h={parts.get('here')}"
    except Exception: return "?"
def hover():
    c=readp('d2',1.5)
    out=[]
    for p in c.split(';'):
        if not p: continue
        x,y,z=map(float,p.split(','))
        r=math.hypot(x,y)
        if r>0.55: out.append(round(math.degrees(math.atan2(y,x))))
    return sorted(out)[:3]+sorted(out)[-3:] if out else []
w('d1','0'); w('d7','0'); time.sleep(0.5)
print("t0 d4=",readp('d4'),"d6=",readp('d6'),"hover=",hover(),flush=True)
# spin CCW slowly
t0=time.time()
while time.time()-t0<26:
    w('d7','8'); time.sleep(0.05)
    el=time.time()-t0
    if abs(el%5)<0.06:
        print(f"t={el:.0f} d4={readp('d4')} d6={readp('d6')} {fl()} hover={hover()}", flush=True)
w('d7','0')
time.sleep(1)
print("end d4=",readp('d4'),"d6=",readp('d6'),"hover=",hover(),flush=True)
