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
        os.write(fd, (msg if isinstance(msg,str) else str(msg)).encode())
        os.close(fd)
    except Exception as e:
        print("werr", p, e)

def status():
    return read('d0'), read('d3'), read('d4'), read('d5'), read('d6'), read('d9'), read('d11')

print("base:", status())
w('d1', 30); w('d7', 30)
for i in range(8):
    time.sleep(0.5)
    print("fwd:", status())
w('d1', 0); w('d7', 0)
time.sleep(0.5)
print("stop:", status())
w('d1', -30); w('d7', -30)
for i in range(4):
    time.sleep(0.5)
    print("rev:", status())
w('d1', 0); w('d7', 0)
print("end:", status())
