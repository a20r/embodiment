import os, select, signal, time

def read_port(p, timeout=0.3):
    try:
        fd = os.open(p, os.O_RDONLY | os.O_NONBLOCK)
    except Exception as e:
        return f"open-err {e}"
    r,_,_ = select.select([fd], [], [], timeout)
    out = None
    if r:
        try: out = os.read(fd, 4096)
        except Exception as e: out = f"read-err {e}"
    os.close(fd)
    return out

def try_write(p, msg=b"0\n"):
    def handler(s,f): raise TimeoutError()
    signal.signal(signal.SIGALRM, handler)
    signal.alarm(2)
    try:
        fd = os.open(p, os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, msg)
        os.close(fd)
        res = "write ok"
    except Exception as e:
        res = f"write fail: {type(e).__name__} {e}"
    finally:
        signal.alarm(0)
    return res

for i in range(12):
    p = f"/dev/robot/d{i}"
    print(f"--- d{i} ---")
    for k in range(3):
        print("  read:", read_port(p))
        time.sleep(0.2)
    print("  write:", try_write(p))
