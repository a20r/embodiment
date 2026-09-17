import time, os, select
NB=os.O_NONBLOCK
def rd(p,wait=0.15):
    fd=os.open(f'/dev/robot/{p}', os.O_RDONLY|NB)
    out=b''; t0=time.time()
    while time.time()-t0<wait:
        r,_,_=select.select([fd],[],[],0.05)
        if r:
            try:
                d=os.read(fd,65536)
                if d: out=d
            except BlockingIOError: pass
    os.close(fd)
    if not out: return ''
    lines=[l for l in out.decode().strip().split('\n') if l]
    return lines[-1] if lines else ''
def wr(p,v):
    fd=os.open(f'/dev/robot/{p}', os.O_WRONLY|NB)
    os.write(fd, f'{v}\n'.encode()); os.close(fd)
