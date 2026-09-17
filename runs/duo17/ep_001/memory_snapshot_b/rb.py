import os, sys, time, select
D='/dev/robot/'
def rd(name, timeout=1.0):
    """read one line from a pipe with timeout"""
    fd = os.open(D+name, os.O_RDONLY|os.O_NONBLOCK)
    try:
        buf=b''
        t0=time.time()
        while time.time()-t0 < timeout:
            r,_,_ = select.select([fd],[],[],0.05)
            if r:
                try:
                    c=os.read(fd,4096)
                except BlockingIOError:
                    continue
                if not c:
                    if buf: break
                    time.sleep(0.01); continue
                buf+=c
                if b'\n' in buf: break
        return buf.decode(errors='replace').split('\n')[0]
    finally:
        os.close(fd)
def wr(name, s, timeout=1.0):
    fd = os.open(D+name, os.O_WRONLY|os.O_NONBLOCK)
    try:
        os.write(fd, (s+'\n').encode())
        return True
    except Exception as e:
        return str(e)
    finally:
        os.close(fd)
if __name__=='__main__':
    cmd=sys.argv[1]
    if cmd=='rd': print(rd(sys.argv[2]))
    elif cmd=='wr': print(wr(sys.argv[2], sys.argv[3]))
