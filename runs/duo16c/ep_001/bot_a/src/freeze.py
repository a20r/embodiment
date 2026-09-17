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
def far():
    c=readp('d2',1.5)
    out=[]
    for p in c.split(';'):
        if not p: continue
        x,y,z=map(float,p.split(','))
        r=math.hypot(x,y)
        if r>0.55: out.append((round(r,1), round(math.degrees(math.atan2(y,x)))))
    out.sort(); return out[:5]
w('d1','0'); w('d7','0')
t0=time.time()
while time.time()-t0<90:
    el=time.time()-t0
    if int(el)%5==0:
        print(f"t={el:.0f} d4={readp('d4')} d11={readp('d11')} {fl()} far={far()}", flush=True)
        try:
            fd=os.open('/dev/robot/d8',os.O_WRONLY)
            os.write(fd,b'R1 STOPPED. COME. SEND POS.\n'); os.close(fd)
        except Exception: pass
    time.sleep(1)
