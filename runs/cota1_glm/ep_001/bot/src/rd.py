import os, time, select
def rd(p,wait=0.25):
    fd=os.open(f'/dev/robot/{p}', os.O_RDONLY|os.O_NONBLOCK)
    out=b''; t0=time.time()
    while time.time()-t0<wait:
        r,_,_=select.select([fd],[],[],0.1)
        if r:
            try:
                d=os.read(fd,8192)
                if d: out=d
            except BlockingIOError:
                pass
    os.close(fd)
    return out.decode().strip()
def wr(p,v):
    fd=os.open(f'/dev/robot/{p}', os.O_WRONLY|os.O_NONBLOCK)
    os.write(fd, f'{v}\n'.encode()); os.close(fd)
