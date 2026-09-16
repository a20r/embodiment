import os, time, errno

BASE="/dev/robot/"
def rd(p, timeout=0.5):
    fd=os.open(BASE+p, os.O_RDONLY|os.O_NONBLOCK)
    try:
        end=time.time()+timeout
        buf=b""
        while time.time()<end:
            try:
                c=os.read(fd,4096)
                if c:
                    buf+=c
                    if b"\n" in buf: return buf.decode().strip()
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK): time.sleep(0.02)
                else: return None
        return buf.decode().strip() if buf else None
    finally:
        os.close(fd)

def wr(p, line, timeout=0.5):
    fd=os.open(BASE+p, os.O_WRONLY|os.O_NONBLOCK)
    try:
        end=time.time()+timeout
        data=(line+"\n").encode()
        while time.time()<end:
            try:
                os.write(fd,data); return True
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK): time.sleep(0.02)
                else: return False
        return False
    finally:
        os.close(fd)

def motors(l, r):
    wr("d1", str(l)); wr("d7", str(r))

def status():
    s=rd("d3")
    d={}
    if s:
        for kv in s.split():
            k,_,v=kv.partition("=")
            try: d[k]=int(v)
            except: d[k]=v
    return d

def lidar():
    s=rd("d2")
    if not s: return None
    try: return [float(x) for x in s.split(",")]
    except: return None

_lasth=[180.0]
def heading():
    s=rd("d4")
    try:
        v=float(s)
        _lasth[0]=v
        return v
    except:
        return _lasth[0]
