#!/usr/bin/env python3
"""Robot I/O helpers. Ports are FIFOs under /dev/robot/. One line per open."""
import os, select, time, sys

DEV = "/dev/robot/d%d"

def read_port(n, timeout=1.0):
    """Read one line from port n with timeout. Returns str or None."""
    path = DEV % n
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return None
    try:
        buf = b""
        deadline = time.time() + timeout
        while time.time() < deadline:
            r, _, _ = select.select([fd], [], [], max(0, deadline - time.time()))
            if not r:
                break
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                time.sleep(0.005); continue
            if not chunk:
                if buf:
                    break
                time.sleep(0.005)
                continue
            buf += chunk
            if b"\n" in buf:
                break
        return buf.decode(errors="replace").strip() if buf else None
    finally:
        os.close(fd)

def write_port(n, line, timeout=1.0):
    path = DEV % n
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            fd = os.open(path, os.O_WRONLY | os.O_NONBLOCK)
            break
        except OSError:
            time.sleep(0.01)
    else:
        return False
    try:
        os.write(fd, (line.rstrip("\n") + "\n").encode())
        return True
    finally:
        os.close(fd)

def snapshot(ports=(0,2,3,4,5,6,9,11), timeout=0.5):
    return {n: read_port(n, timeout) for n in ports}

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "w":
        print(write_port(int(sys.argv[2]), " ".join(sys.argv[3:])))
    elif len(sys.argv) > 1 and sys.argv[1] == "r":
        print(read_port(int(sys.argv[2]), float(sys.argv[3]) if len(sys.argv) > 3 else 1.0))
    else:
        for k, v in snapshot().items():
            print(f"d{k}: {v}")
