import os, time, select, sys

DEV = "/dev/robot/d%d"

def read_line(port, timeout=2.0):
    """Read one line from a FIFO port with timeout. Returns str or None."""
    fd = None
    try:
        fd = os.open(DEV % port, os.O_RDONLY | os.O_NONBLOCK)
        buf = b""
        t0 = time.time()
        while time.time() - t0 < timeout:
            r, _, _ = select.select([fd], [], [], 0.05)
            if r:
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if not chunk:
                    # EOF: writer closed. if we have data, return
                    if buf:
                        break
                    # re-open to wait for new writer
                    os.close(fd)
                    fd = os.open(DEV % port, os.O_RDONLY | os.O_NONBLOCK)
                    time.sleep(0.01)
                    continue
                buf += chunk
                if b"\n" in buf:
                    break
        line = buf.split(b"\n")[0].decode(errors="replace") if buf else None
        return line
    finally:
        if fd is not None:
            os.close(fd)

def write_line(port, s, timeout=2.0):
    fd = None
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            fd = os.open(DEV % port, os.O_WRONLY | os.O_NONBLOCK)
            break
        except OSError:
            time.sleep(0.02)
    if fd is None:
        return False
    try:
        os.write(fd, (s.rstrip("\n") + "\n").encode())
        return True
    finally:
        os.close(fd)

def scan():
    l = read_line(2)
    if l is None: return None
    return [float(x) for x in l.split(",")]

def heading():
    l = read_line(4)
    return float(l) if l else None

def status():
    l = read_line(3)
    if not l: return None
    d = {}
    for kv in l.split():
        k, v = kv.split("=")
        d[k] = int(v)
    return d

def rx(timeout=1.0):
    return read_line(10, timeout)

def tx(msg):
    return write_line(8, msg)

def d(port):
    return read_line(port)

if __name__ == "__main__":
    print("scan", scan())
    print("heading", heading())
    print("status", status())
    for p in (0,5,6,9,11):
        print("d%d" % p, d(p))
