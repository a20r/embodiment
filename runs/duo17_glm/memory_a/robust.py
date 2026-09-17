import os, time
R="/dev/robot/"
def rline(port, timeout=2.0):
    fd=os.open(R+port, os.O_RDONLY|os.O_NONBLOCK)
    try:
        buf=b""; t0=time.time()
        while time.time()-t0<timeout:
            try:
                d=os.read(fd,256)
                if d:
                    buf+=d
                    if b"\n" in buf: return buf.split(b"\n")[0].decode()
            except BlockingIOError: pass
            time.sleep(0.01)
        return buf.decode() if buf else None
    finally: os.close(fd)
