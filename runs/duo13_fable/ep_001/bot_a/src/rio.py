import os, select, time, sys
DEV='/dev/robot/d%d'
def readline(port, timeout=2.0):
    """Read one line from a FIFO port with timeout. Returns None on timeout."""
    fd = os.open(DEV % port, os.O_RDONLY | os.O_NONBLOCK)
    try:
        buf = b''
        end = time.time() + timeout
        while time.time() < end:
            r,_,_ = select.select([fd],[],[],max(0,end-time.time()))
            if not r: break
            try: chunk = os.read(fd, 4096)
            except BlockingIOError: chunk=None
            if not chunk:
                if buf: break
                time.sleep(0.01); continue
            buf += chunk
            if b'\n' in buf: break
        return buf.decode(errors='replace').split('\n')[0] if buf else None
    finally:
        os.close(fd)
def writeline(port, s, timeout=2.0):
    end=time.time()+timeout
    while time.time()<end:
        try:
            fd=os.open(DEV%port, os.O_WRONLY|os.O_NONBLOCK); break
        except OSError:
            time.sleep(0.05)
    else:
        return False
    try:
        os.write(fd,(s+'\n').encode()); return True
    finally: os.close(fd)
def ranges():
    s=readline(2); return [float(x) for x in s.split(',')] if s else None
def heading():
    s=readline(4); return float(s) if s else None
def status():
    s=readline(3); return dict(kv.split('=') for kv in s.split()) if s else None
if __name__=='__main__':
    if sys.argv[1]=='r': print(readline(int(sys.argv[2])))
    elif sys.argv[1]=='w': print(writeline(int(sys.argv[2]), sys.argv[3]))
