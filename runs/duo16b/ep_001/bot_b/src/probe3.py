import os, time, select

def samples(name, dur=1.0):
    fd = os.open('/dev/robot/'+name, os.O_RDONLY | os.O_NONBLOCK)
    out = []
    t0 = time.time()
    while time.time()-t0 < dur:
        r,_,_ = select.select([fd],[],[],0.1)
        if r:
            try:
                d = os.read(fd, 4096).decode().strip()
                if d: out.append(d)
            except Exception: pass
    os.close(fd)
    return out

for n in ['d0','d2','d3','d4','d5','d6','d9','d10','d11']:
    s = samples(n, 0.8)
    print(f"{n}: n={len(s)} first={s[:3]} last={s[-2:] if s else ''}")
