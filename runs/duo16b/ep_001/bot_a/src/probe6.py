import os, select, time

D = '/dev/robot/'
def read(p, timeout=0.25):
    fd = os.open(D+p, os.O_RDONLY | os.O_NONBLOCK)
    r,_,_ = select.select([fd], [], [], timeout)
    out = ''
    if r:
        try: out = os.read(fd, 200000).decode().strip()
        except Exception: out = ''
    os.close(fd)
    return out

s = read('d2')
pts = [p for p in s.split(';') if p.strip()]
print("npoints:", len(pts))
rows = [tuple(map(float,p.split(','))) for p in pts]
# structure of col2 and col3
c2 = [r[1] for r in rows]; c3 = [r[2] for r in rows]
print("col2 first 40:", c2[:40])
print("col2 last 5:", c2[-5:])
print("col3 first 40:", c3[:40])
print("col3 last 5:", c3[-5:])
print("col1 first 40:", [r[0] for r in rows[:40]])
print("col1 stats: min %.3f max %.3f" % (min(r[0] for r in rows), max(r[0] for r in rows)))
print("d0:", read('d0'), "d5:", read('d5'), "d6:", read('d6'), "d9:", read('d9'), "d11:", read('d11'), "d4:", read('d4'))
