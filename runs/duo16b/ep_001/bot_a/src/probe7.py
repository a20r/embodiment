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
    try:
        fd = os.open(D+p, os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, (str(msg)).encode())
        os.close(fd)
    except Exception as e:
        print("werr", p, e)

def enc(): return read('d6'), read('d9')

def drive(v, dur=1.5, refresh=0.1):
    t0 = time.time()
    while time.time() - t0 < dur:
        w('d1', v); w('d7', v)
        time.sleep(refresh)
    w('d1', 0); w('d7', 0)

print("enc before:", enc())
for v in [10, 200, 500, 1000, -500, 1.0, 0.5]:
    drive(v, 1.0)
    time.sleep(0.2)
    print(f"v={v}: enc after:", enc(), "head:", read('d4'))
