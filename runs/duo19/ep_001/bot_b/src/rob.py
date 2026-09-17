import os, time, sys, select

DEV = '/dev/robot/d%d'

def read_line(port, timeout=1.0):
    """Read one line from a port with timeout. Returns None on timeout."""
    fd = os.open(DEV % port, os.O_RDONLY | os.O_NONBLOCK)
    try:
        buf = b''
        end = time.time() + timeout
        while time.time() < end:
            r, _, _ = select.select([fd], [], [], max(0, end - time.time()))
            if not r:
                break
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if not chunk:
                # writer closed; if we have data return it
                if buf:
                    break
                time.sleep(0.005)
                continue
            buf += chunk
            if b'\n' in buf:
                break
        if not buf:
            return None
        return buf.split(b'\n')[0].decode(errors='replace')
    finally:
        os.close(fd)

def write_line(port, s, timeout=1.0):
    fd = os.open(DEV % port, os.O_WRONLY | os.O_NONBLOCK)
    try:
        os.write(fd, (s.rstrip('\n') + '\n').encode())
        return True
    except BlockingIOError:
        return False
    finally:
        os.close(fd)

def snapshot():
    d = {}
    for p in (0, 2, 3, 4, 5, 6, 9, 11):
        d[p] = read_line(p, 0.5)
    return d

def show(d=None):
    if d is None:
        d = snapshot()
    for k in sorted(d):
        print(f"d{k}: {d[k]}")

if __name__ == '__main__':
    show()
