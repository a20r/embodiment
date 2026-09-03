import os, select, time, math

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
    if isinstance(msg, (int, float)): msg = f"{msg}\n"
    try:
        fd = os.open(D+p, os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, msg.encode()); os.close(fd)
    except Exception as e:
        print("werr", p, e)

# radio test: transmit, then listen
w('d8', "PING1\n")
time.sleep(0.3)
print("d10 after PING1:", repr(read('d10')))

# spin scan: rotate ~360 deg over 8s, log sensors
t0 = time.time()
samples = []
while time.time() - t0 < 9.0:
    w('d1', 30); w('d7', -30)
    time.sleep(0.4)
    h = read('d4'); s0 = read('d0'); s5 = read('d5'); s11 = read('d11'); st = read('d3')
    samples.append((h, s0, s5, s11, st))
w('d1', 0); w('d7', 0)
for s in samples: print(s)
