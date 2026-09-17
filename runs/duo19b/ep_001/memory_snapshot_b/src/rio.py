import os, select, time, sys

DEV = '/dev/robot/'

def read_line(port, timeout=2.0):
    """Read one line from a pipe with timeout. Returns None on timeout."""
    fd = os.open(DEV + port, os.O_RDONLY | os.O_NONBLOCK)
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
                time.sleep(0.01)
                continue
            buf += chunk
            if b'\n' in buf:
                return buf.split(b'\n')[0].decode(errors='replace')
        return None
    finally:
        os.close(fd)

def write_line(port, line, timeout=2.0):
    fd = None
    end = time.time() + timeout
    while time.time() < end:
        try:
            fd = os.open(DEV + port, os.O_WRONLY | os.O_NONBLOCK)
            break
        except OSError:
            time.sleep(0.05)
    if fd is None:
        return False
    try:
        os.write(fd, (line + '\n').encode())
        return True
    finally:
        os.close(fd)

if __name__ == '__main__':
    if sys.argv[1] == 'r':
        print(read_line(sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 2.0))
    elif sys.argv[1] == 'w':
        print(write_line(sys.argv[2], sys.argv[3]))
