import time

DEV='/dev/robot/d'

def read(n, timeout=1.0):
    import os, select
    fd = os.open(DEV+str(n), os.O_RDONLY | os.O_NONBLOCK)
    try:
        buf = b''
        end = time.time()+timeout
        while time.time() < end:
            r,_,_ = select.select([fd],[],[],0.1)
            if r:
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if chunk:
                    buf += chunk
                    if b'\n' in buf:
                        return buf.split(b'\n')[0].decode()
        return None
    finally:
        os.close(fd)

def write(n, s, timeout=1.0):
    import os
    fd = os.open(DEV+str(n), os.O_WRONLY)
    try:
        os.write(fd, (str(s)+'\n').encode())
    finally:
        os.close(fd)

def lidar():
    s = read(1)
    return [float(x) for x in s.split(',')] if s else None

def heading():
    s = read(2)
    return float(s) if s else None

def status():
    return read(9)

def drive(turn=None, fwd=None):
    if turn is not None: write(4, turn)
    if fwd is not None: write(5, fwd)

if __name__=='__main__':
    import sys
    print(status(), heading(), lidar())
