import os, sys, time
def try_read(dev, timeout=1.0):
    try:
        fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
    except OSError as e:
        return f"open-err {e}"
    out = b""
    end = time.time()+timeout
    while time.time() < end:
        try:
            b = os.read(fd, 4096)
            if b: out += b
            else: time.sleep(0.05)
        except BlockingIOError:
            time.sleep(0.05)
    os.close(fd)
    return out.decode(errors="replace")
for i in range(12):
    dev=f"/dev/robot/d{i}"
    r=try_read(dev,0.8)
    print(i, repr(r)[:300])
