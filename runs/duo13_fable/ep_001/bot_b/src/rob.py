import os, select, time, sys
D='/dev/robot/'
def rd(port, timeout=1.0):
    """Read one line from a port with timeout. Returns str or None."""
    fd = os.open(D+port, os.O_RDONLY|os.O_NONBLOCK)
    try:
        buf=b''
        end=time.time()+timeout
        while time.time()<end:
            r,_,_=select.select([fd],[],[],max(0,end-time.time()))
            if not r: break
            try:
                c=os.read(fd,4096)
            except BlockingIOError:
                continue
            if not c:
                time.sleep(0.01); continue
            buf+=c
            if b'\n' in buf:
                return buf.split(b'\n')[0].decode(errors='replace')
        return buf.decode(errors='replace') if buf else None
    finally:
        os.close(fd)
def wr(port, s, timeout=1.0):
    fd=None
    try:
        fd=os.open(D+port, os.O_WRONLY|os.O_NONBLOCK)
        os.write(fd,(s+'\n').encode())
        return True
    except OSError as e:
        return False
    finally:
        if fd is not None: os.close(fd)
def snap():
    out={}
    for p in ['d0','d2','d3','d4','d5','d6','d9','d11']:
        out[p]=rd(p,0.5)
    return out
def ranges():
    s=rd('d2',1.0)
    return [float(x) for x in s.split(',')] if s else None
if __name__=='__main__':
    if len(sys.argv)>2 and sys.argv[1]=='w':
        print(wr(sys.argv[2],sys.argv[3]))
    elif len(sys.argv)>1 and sys.argv[1]=='snap':
        for k,v in snap().items(): print(k,v)
    else:
        print(rd(sys.argv[1]))
