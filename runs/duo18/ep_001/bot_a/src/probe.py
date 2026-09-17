import os, sys, time, select
def readline_timeout(path, timeout=2.0):
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError as e:
        return f"ERR open {e}"
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        r,_,_ = select.select([fd],[],[],0.1)
        if r:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if not chunk:
                time.sleep(0.05); continue
            buf += chunk
            if b"\n" in buf:
                break
    os.close(fd)
    return buf.decode(errors="replace").strip()
if __name__ == "__main__":
    ports = sys.argv[1:] or ["d0","d3","d4","d5","d6","d9","d10","d11"]
    for i in range(3):
        print(f"--- sample {i} t={time.time():.1f}")
        for p in ports:
            print(f"{p}: {readline_timeout('/dev/robot/'+p, 1.5)!r}")
        time.sleep(0.5)
