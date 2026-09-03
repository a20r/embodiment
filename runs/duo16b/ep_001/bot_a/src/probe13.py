import os, select, time

D = '/dev/robot/'
def read(p, timeout=0.2):
    fd = os.open(D+p, os.O_RDONLY | os.O_NONBLOCK)
    r,_,_ = select.select([fd], [], [], timeout)
    out = ''
    if r:
        try: out = os.read(fd, 200000).decode().strip()
        except Exception: out = ''
    os.close(fd)
    return out

def w(p, msg):
    if isinstance(msg,(int,float)): msg = f"{msg}\n"
    try:
        fd = os.open(D+p, os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, msg.encode()); os.close(fd)
    except Exception as e: print("werr", p, e)

# drive forward 3s, log d11, d0, d5, encoders, lidar mean
def lidar_mean():
    s = read('d2')
    pts = [p for p in s.split(';') if p.strip()]
    rs = []
    for p in pts:
        try:
            v = float(p.split(',')[0])
            if v > 0: rs.append(v)
        except: pass
    return sum(rs)/len(rs) if rs else -1

print("pre: enc", read('d6'), read('d9'), "d11", read('d11'), "lidar_mean %.3f" % lidar_mean())
t0 = time.time(); log = []
while time.time()-t0 < 3.0:
    w('d1', 60); w('d7', 60)
    time.sleep(0.3)
    log.append((read('d6'), read('d9'), read('d11'), read('d0'), read('d5'), read('d4')))
w('d1', 0); w('d7', 0)
for l in log: print(l)
print("post: lidar_mean %.3f" % lidar_mean())
