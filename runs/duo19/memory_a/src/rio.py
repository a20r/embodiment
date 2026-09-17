import os, time, select, sys

DEV='/dev/robot/'
def rd(port, timeout=1.0):
    """Read one line from a port (non-blocking open with timeout)."""
    try:
        fd = os.open(DEV+port, os.O_RDONLY | os.O_NONBLOCK)
    except OSError as e:
        return None
    try:
        buf=b''
        t0=time.time()
        while time.time()-t0 < timeout:
            r,_,_ = select.select([fd],[],[],0.05)
            if r:
                try:
                    c = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if not c:
                    # writer closed; if we have data return it
                    if buf: break
                    # else keep waiting for a writer
                    time.sleep(0.01)
                    continue
                buf += c
                if b'\n' in buf: break
        return buf.decode(errors='replace').strip()
    finally:
        os.close(fd)

def wr(port, line, timeout=1.0):
    """Write one line to a port with timeout."""
    try:
        fd = os.open(DEV+port, os.O_WRONLY | os.O_NONBLOCK)
    except OSError as e:
        return 'ERR open: %s' % e
    try:
        os.write(fd, (line+'\n').encode())
        return 'ok'
    except OSError as e:
        return 'ERR write: %s' % e
    finally:
        os.close(fd)

if __name__=='__main__':
    if sys.argv[1]=='r':
        print(rd(sys.argv[2], float(sys.argv[3]) if len(sys.argv)>3 else 1.0))
    elif sys.argv[1]=='w':
        print(wr(sys.argv[2], sys.argv[3]))
