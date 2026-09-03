import os, select, time

D = '/dev/robot/'
def read(p, timeout=0.3):
    fd = os.open(D+p, os.O_RDONLY | os.O_NONBLOCK)
    r,_,_ = select.select([fd], [], [], timeout)
    out = ''
    if r:
        try: out = os.read(fd, 2000000).decode().strip()
        except Exception: out = ''
    os.close(fd)
    return out

for k in range(2):
    s = read('d2')
    pts = [p for p in s.split(';') if p.strip()]
    rows = [tuple(map(float,p.split(','))) for p in pts]
    els = [r[1] for r in rows]; azs = [r[2] for r in rows]; rs = [r[0] for r in rows]
    neg = sum(1 for r in rs if r < 0)
    print(f"cap{k}: n={len(rows)} elev[{min(els):.3f},{max(els):.3f}] az[{min(azs):.3f},{max(azs):.3f}]")
    print(f"  range[{min(rs):.3f},{max(rs):.3f}] neg={neg} meanpos={sum(x for x in rs if x>0)/max(1,(len(rs)-neg)):.3f}")
    time.sleep(0.5)
# radio: send and listen for a while
os.close(os.open(D+'d8', os.O_WRONLY | os.O_NONBLOCK))  # just check open
fd = os.open(D+'d8', os.O_WRONLY | os.O_NONBLOCK)
os.write(fd, b"HELLO\n"); os.close(fd)
t0=time.time(); got=[]
while time.time()-t0 < 5:
    m = read('d10', 0.3)
    if m: got.append(m)
print("d10 messages in 5s:", got)
print("d11:", read('d11'), "d0:", read('d0'), "d5:", read('d5'))
