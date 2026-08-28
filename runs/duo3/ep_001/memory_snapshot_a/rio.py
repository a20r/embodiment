import os, select, time

def read_port(p, timeout=2.0):
    fd = os.open(f"/dev/robot/{p}", os.O_RDONLY | os.O_NONBLOCK)
    buf = b""
    t0 = time.time()
    try:
        while time.time() - t0 < timeout:
            r,_,_ = select.select([fd], [], [], 0.1)
            if fd in r:
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if chunk:
                    buf += chunk
                    if b"\n" in buf:
                        return buf.split(b"\n")[0].decode()
    finally:
        os.close(fd)
    return None

def write_port(p, line):
    with open(f"/dev/robot/{p}", "w") as f:
        f.write(line.rstrip("\n") + "\n")

def lidar():
    s = read_port("d1")
    return [float(x) for x in s.split(",")] if s else None

def compass():
    s = read_port("d2")
    return float(s) if s else None
