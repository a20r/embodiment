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
    if isinstance(msg, int): msg = f"{msg}\n"
    try:
        fd = os.open(D+p, os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, msg.encode())
        os.close(fd)
    except Exception as e:
        print("werr", p, e)

def enc(): return read('d6'), read('d9')

def drive(l, r, dur):
    t0 = time.time()
    while time.time() - t0 < dur:
        w('d1', l); w('d7', r)
        time.sleep(0.1)
    w('d1', 0); w('d7', 0)

print("enc before:", enc(), "head:", read('d4'))
drive(60, 60, 2.0)
time.sleep(0.3)
print("after fwd60 2s: enc:", enc(), "head:", read('d4'))
drive(-60, -60, 1.0)
time.sleep(0.3)
print("after rev60 1s: enc:", enc(), "head:", read('d4'))
drive(60, -60, 1.0)
time.sleep(0.3)
print("after spin 1s: enc:", enc(), "head:", read('d4'))
