import os, select, time

D = '/dev/robot/'
def read(p, timeout=0.2):
    fd = os.open(D+p, os.O_RDONLY | os.O_NONBLOCK)
    r,_,_ = select.select([fd], [], [], timeout)
    out = None
    if r:
        try: out = os.read(fd, 4096).decode().strip()
        except Exception: out = ''
    os.close(fd)
    return out

def w(p, msg):
    try:
        fd = os.open(D+p, os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, (str(msg)).encode())
        os.close(fd)
    except Exception as e:
        print("werr", p, e)

def short_lidar():
    s = read('d2')
    if not s: return []
    pts = [p for p in s.split(';') if p.strip()]
    out = []
    for p in pts:
        try: out.append(tuple(map(float, p.split(','))))
        except: pass
    return out

def stats():
    s = short_lidar()
    if not s: return "no lidar"
    rs = [p[0] for p in s]
    return f"n={len(s)} min={min(rs):.3f} max={max(rs):.3f} mean={sum(rs)/len(rs):.3f}"

w('d1', 0); w('d7', 0)
time.sleep(0.3)
print("enc d6,d9:", read('d6'), read('d9'))
print("lidar:", stats())
w('d1', 100); w('d7', 100)
time.sleep(2.0)
w('d1', 0); w('d7', 0)
time.sleep(0.3)
print("after fwd100: enc:", read('d6'), read('d9'), "head:", read('d4'))
print("lidar:", stats())
w('d1', 100); w('d7', -100)
time.sleep(2.0)
w('d1', 0); w('d7', 0)
time.sleep(0.3)
print("after spin: enc:", read('d6'), read('d9'), "head:", read('d4'))
print("lidar:", stats())
